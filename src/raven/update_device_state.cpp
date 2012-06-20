/*
 * update_device_state.c
 *
 */

#include "update_device_state.h"
#include "log.h"

extern struct DOF_type DOF_types[];
extern struct traj trajectory[];
extern int NUM_MECH;
extern volatile int isUpdated;

unsigned int newDofTorqueSetting = 0;   // for setting torque from console
unsigned int newDofTorqueMech = 0;      // for setting torque from console
unsigned int newDofTorqueDof = 0;       //
int newDofTorqueTorque = 0;             // float for torque value in mNm
//t_controlmode newRobotControlMode = motor_pd_control;
t_controlmode newRobotControlMode = homing_mode;

/*
 * updateDeviceState - Function that update the device state based on parameters passed from
 *       the user interface
 *
 * inputs - params_current - the current set of parameters
 *          params_update - the new set of parameters
 *          device0 - pointer to device informaiton
 *
 */
int updateDeviceState(struct param_pass *currParams, struct param_pass *rcvdParams, struct device *device0)
{
    for (int i = 0; i < NUM_MECH; i++)
    {
        currParams->xd[i].x = rcvdParams->xd[i].x;
        currParams->xd[i].y = rcvdParams->xd[i].y;
        currParams->xd[i].z = rcvdParams->xd[i].z;
        currParams->rd[i].yaw   = rcvdParams->rd[i].yaw;
        currParams->rd[i].pitch = rcvdParams->rd[i].pitch * WRIST_SCALE_FACTOR;
        currParams->rd[i].roll  = rcvdParams->rd[i].roll;
        currParams->rd[i].grasp = rcvdParams->rd[i].grasp;
    }

    // set desired mech position in pedal_down runlevel
    if (currParams->runlevel == RL_PEDAL_DN)
    {
        for (int i = 0; i < NUM_MECH; i++)
        {
            device0->mech[i].pos_d.x = rcvdParams->xd[i].x;
            device0->mech[i].pos_d.y = rcvdParams->xd[i].y;
            device0->mech[i].pos_d.z = rcvdParams->xd[i].z;
            device0->mech[i].ori_d.grasp  = rcvdParams->rd[i].grasp;

            for (int j=0;j<3;j++)
                for (int k=0;k<3;k++)
                device0->mech[i].ori_d.R[j][k]  = rcvdParams->rd[i].R[j][k];
        }
    }

    // Switch control modes only in pedal up or init.
    if (99 == (int)newRobotControlMode) {
        log_msg("Current control mode: %d",currParams->robotControlMode);
        newRobotControlMode = (t_controlmode)currParams->robotControlMode;
    } else if ( (currParams->runlevel == RL_E_STOP)   &&
         (currParams->robotControlMode != (int)newRobotControlMode) ) {
        currParams->robotControlMode = (int)newRobotControlMode;
        log_msg("Control mode updated");
    }

    // Set new torque command from console user input
    if ( newDofTorqueSetting )
    {
        // reset all other joints to zero
        for (unsigned int idx=0; idx<MAX_MECH_PER_DEV*MAX_DOF_PER_MECH; idx++)
        {
            if ( idx == MAX_DOF_PER_MECH*newDofTorqueMech + newDofTorqueDof )
                currParams->torque_vals[idx] = newDofTorqueTorque;
            else
                currParams->torque_vals[idx] = 0;
        }
        newDofTorqueSetting = 0;
        log_msg("DOF Torque updated\n");
    }

    // Set new surgeon mode
    if ( device0->surgeon_mode != rcvdParams->surgeon_mode)
    {
        device0->surgeon_mode=rcvdParams->surgeon_mode; //store the surgeon_mode to DS0
    }

    return 0;
}

/*
*  setRobotControlMode()
*       Change controller mode, i.e. position control, velocity control, visual servoing, etc
*/
void setRobotControlMode(t_controlmode in_controlMode){
    if (99 != in_controlMode) {
        log_msg("Robot control mode: %d",in_controlMode);
    }
    newRobotControlMode = in_controlMode;
    isUpdated = TRUE;
}

/*
*  setDofTorque()
*    Set a torque to output on a joint.
*
*     Torque input is mNm
*/
void setDofTorque(unsigned int in_mech, unsigned int in_dof, int in_torque){
    if (    ((int)in_mech < NUM_MECH)        &&
            ((int)in_dof  < MAX_DOF_PER_MECH) )
    {
        newDofTorqueMech    = in_mech;
        newDofTorqueDof     = in_dof;
        newDofTorqueTorque  = in_torque;
        newDofTorqueSetting = 1;
    }
    isUpdated = TRUE;
}
