/**
*
*   File: homing.cpp
*
*   Created 3-Nov-2011 by Hawkeye King
*
*      Based on concept by UCSC, I implement a procedure for joint position discovery from relative encoders.
*
*/
#include <stdlib.h>

#include "trajectory.h"
#include "pid_control.h"
#include "defines.h"
#include "inv_cable_coupling.h"
#include "fwd_cable_coupling.h"
#include "t_to_DAC_val.h"
#include "homing.h"
#include "state_estimate.h"
#include "log.h"
#include "offsets.h"


int homing_condition_met(struct DOF *_joint);
int set_joints_known_pos(struct mechanism* _mech, int tool_only);

extern int NUM_MECH;
extern unsigned long int gTime;
extern struct DOF_type DOF_types[];
extern unsigned int soft_estopped;

/*
*  raven_homing()
*    1- Discover joint position by running to hard stop
*    2- Move joints to "home" position.
*
*/
int raven_homing(struct device *device0, struct param_pass *currParams, int begin_homing)
{
    static int homing_inited = 0;
    static unsigned long int delay, delay2;
    struct DOF *_joint = NULL;
    struct mechanism* _mech = NULL;
    int i=0,j=0;

    // Only run in init mode
    if ( ! (currParams->runlevel == RL_INIT && currParams->sublevel == SL_AUTO_INIT ))
    {
        homing_inited = 0;
        delay = gTime;
        return 0;
    }

    // Wait a short time for amps to turn on
    if (gTime - delay < 1000)
    {
        return 0;
    }
    // Initialize the homing sequence.
    if (begin_homing || !homing_inited)
    {
        // Zero out joint torques, and control inputs. Set joint.state=not_ready.
        _mech = NULL;  _joint = NULL;
        while (loop_over_joints(device0, _mech, _joint, i,j) )
        {
            _joint->tau_d  = 0;
            _joint->mpos_d = _joint->mpos;
            _joint->jpos_d = _joint->jpos;
            _joint->jvel_d = 0;
            _joint->state  = jstate_not_ready;

            if (is_toolDOF(_joint))
                jvel_PI_control(_joint, 1);  // reset PI control integral term
            homing_inited = 1;
        }
    }

    // Specify motion commands
    _mech = NULL;  _joint = NULL;
    while ( loop_over_joints(device0, _mech, _joint, i,j) )
    {
        // Initialize tools first.
        if ( is_toolDOF(_joint) || tools_ready( &(device0->mech[i]) ) )
        {
            homing(_joint);
        }
    }

    //Inverse Cable Coupling
    invCableCoupling(device0, currParams->runlevel);

    // Do PD control on all joints
    _mech = NULL;  _joint = NULL;
    while ( loop_over_joints(device0, _mech, _joint, i,j) )
    {
        mpos_PD_control( _joint );
    }

    // Calculate output DAC values
    TorqueToDAC(device0);

    // Check homing conditions and set joint angles appropriately.
    _mech = NULL;  _joint = NULL;
    while ( loop_over_joints(device0, _mech, _joint, i,j) )
    {
        struct DOF * _joint =  &(_mech->joint[j]);

        // Check to see if we've reached the joint limit.
        if( check_homing_condition(_joint) )
        {
            // log_msg("Found limit on joint %d", _joint->type, _joint->current_cmd, DOF_types[_joint->type].DAC_max);
            _joint->state = jstate_hard_stop;
            _joint->current_cmd = 0;
            stop_trajectory(_joint);
        }

        // For each mechanism, check to see if the mech is finished homing.
        if ( j == (MAX_DOF_PER_MECH-1) )
        {
            /// if we're homing tools, wait for tools to be finished
            if ((  !tools_ready(_mech) &&
                   _mech->joint[TOOL_ROT].state==jstate_hard_stop &&
                   _mech->joint[WRIST   ].state==jstate_hard_stop &&
                   _mech->joint[GRASP1  ].state==jstate_hard_stop )
                    ||
                (  tools_ready( _mech ) &&
                   _mech->joint[SHOULDER].state==jstate_hard_stop &&
                   _mech->joint[ELBOW   ].state==jstate_hard_stop &&
                   _mech->joint[Z_INS   ].state==jstate_hard_stop ))
            {
                if (delay2==0)
                    delay2=gTime;

                if (gTime > delay2 + 200)
                {
                    set_joints_known_pos(_mech, !tools_ready(_mech) );
                    delay2 = 0;
                }
            }
        }

    }
    
    saveOffsets(*device0);

    return 0;
}

/**
*    set_joints_known_pos()
*
*       Set all the mechanism joints to known reference angles.
*       Propogate the joint angle to motor position and encoder offset.
*/
int set_joints_known_pos(struct mechanism* _mech, int tool_only)
{
    struct DOF* _joint=NULL;
    int j=0;

    /// Set joint position reference for just tools, or all DOFS
    _joint = NULL;
    while ( loop_over_joints(_mech, _joint ,j) )
    {
        if ( tool_only  && ! is_toolDOF( _joint->type))
        {
            // Set jpos_d to the joint limit value.
            _joint->jpos_d = DOF_types[ _joint->type ].home_position;
        }

        else if (!tool_only && is_toolDOF(_joint->type) )
            _joint->jpos_d = _joint->jpos;

        else
        {
            // Set jpos_d to the joint limit value.
            _joint->jpos_d = DOF_types[ _joint->type ].max_position;

            // Initialize a trajectory to operating angle
            _joint->state = jstate_homing1;
        }
    }

    /// Inverse cable coupling: jpos_d  ---> mpos_d
    invMechCableCoupling(_mech, 1);

    _joint = NULL;
    while ( loop_over_joints(_mech, _joint ,j) )
    {
        // Reset the state-estimate filter
        _joint->mpos = _joint->mpos_d;
        resetFilter( _joint );

        // Convert the motor position to an encoder offset.
        // mpos = k * (enc_val - enc_offset)  --->  enc_offset = enc_val - mpos/k
        float f_enc_val = _joint->enc_val;

        // Encoder values on Gold arm are reversed.  See also state_machine.cpp
        if ( _mech->type == GOLD_ARM)
             f_enc_val *= -1.0;

        /// Set the joint offset in encoder space.
        float cc = ENC_CNTS_PER_REV / (2*M_PI);
        _joint->enc_offset = f_enc_val - (_joint->mpos_d * cc);
        getStateLPF(_joint);
    }

    fwdMechCableCoupling(_mech);
    return 0;
}

/**
*     homing()
*
*   set trajectory behavior for each joint during the homing process.
*/
void homing(struct DOF* _joint)
{
    const float f_period[MAX_MECH*MAX_DOF_PER_MECH] = {1, 1, 1, 9999999, 1, 1, 1, 1,
                                                        1, 1, 1, 9999999, 1, 1, 1, 1};
    const float f_magnitude[MAX_MECH*MAX_DOF_PER_MECH] = {-10 DEG2RAD, 10 DEG2RAD, 0.02, 9999999, 80 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD, 40 DEG2RAD,
                                                          -10 DEG2RAD, 10 DEG2RAD, 0.02, 9999999, 80 DEG2RAD, 40 DEG2RAD, -40 DEG2RAD, -40 DEG2RAD};

    switch (_joint->state)
    {
        case jstate_wait:
            break;
        case jstate_not_ready:
            // Initialize velocity trajectory
            //log_msg("Starting homing on joint %d", _joint->type);
            _joint->state = jstate_pos_unknown;
            start_trajectory_mag(_joint, f_magnitude[_joint->type], f_period[_joint->type]);
            break;

        case jstate_pos_unknown:
            // Set desired joint trajectory
            update_linear_sinusoid_position_trajectory(_joint);
            break;

        case jstate_hard_stop:
            // Wait for all joints. No trajectory here.
            break;

        case jstate_homing1:
            start_trajectory( _joint , DOF_types[_joint->type].home_position, 2.5 );
            _joint->state = jstate_homing2;

        case jstate_homing2:
            // Move to start position
            // Update position trajectory
            if ( !update_position_trajectory(_joint) )
            {
                _joint->state = jstate_ready;
                log_msg("Joint %d ready", _joint->type);
            }
            break;

        default:
            // not doing joint homing.
            break;

    } // switch

    return;
}



const int homing_max_dac[8] = {2500,  //shoulder
                            2500,  //elbow
                            1900,  //tool_rot
                            0,
                            1400,  //z_ins
                            1900,  //wrist
                            1900,  //grasp1
                            1900};  // grasp2


int check_homing_condition(struct DOF *_joint)
{
    if ( _joint->state != jstate_pos_unknown)
        return 0;

    // check if the DAC output is greater than the maximum allowable.
    // Note, current_cmd is an integer, so using abs (not fabs) is okay.
    if( abs(_joint->current_cmd) >= homing_max_dac[_joint->type%8]  )
    {
        return 1;
    }

   return 0;
}








//    // --- check the forward path ---
//
//    // set motor position from encoder space
//    encToMPos(_joint);
//
//    // execute inverse cable coupling to find the matching motor position.
//    fwdCableCoupling(device0, device0->runlevel);
//
//    float temp_motorPos = (2*PI) * (float)(1/((float)ENC_CNTS_PER_REV)) * (float)(_joint->enc_val - _joint->enc_offset);
//
//    log_msg("typ:%d: jpd:%0.3f \tjp:%0.3f \t mpd:%0.3f \t tmp:%0.3f \t mpo:%0.3f \t enc:%d \t eno:%d \t cpr:%0.3f",
//            _joint->type, _joint->jpos_d, _joint->jpos, _joint->mpos_d, temp_motorPos, _joint->mpos, _joint->enc_val , _joint->enc_offset, (float)ENC_CNT_PER_RAD);
//
////    log_msg("jpd:%0.3f \tjp:%0.3f \t mpd:%0.3f \t enc:%d \t eno:%d \t cpr:%0.3f \t tmp:%0.3f", _joint->jpos_d, _joint->jpos, _joint->mpos_d, _joint->enc_val , _joint->enc_offset, (float)ENC_CNT_PER_RAD, temp_motorPos);
