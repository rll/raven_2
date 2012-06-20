/**
* File: rt_raven.cpp
* Created 13-oct-2011 by Hawkeye
*
*   Runs all raven control functions.
*   Code split out from rt_process_preempt.cpp, in order to provide more flexibility.
*
*/

#include <ros/ros.h>

#include <openrave/openrave.h>
#include <openrave-core.h>
#include "stdafx.h"
#include "GeneralIK.h"

#include "rt_raven.h"
#include "defines.h"

#include "init.h"             // for initSurgicalArms()
#include "inv_kinematics.h"
#include "inv_cable_coupling.h"
#include "state_estimate.h"
#include "pid_control.h"
#include "grav_comp.h"
#include "t_to_DAC_val.h"
#include "fwd_cable_coupling.h"
#include "fwd_kinematics.h"
#include "trajectory.h"
#include "homing.h"
#include "local_io.h"
#include "update_device_state.h"
#include <cmath>

using namespace std;
using namespace OpenRAVE;


#define STRINGIFY(x) #x
#define EXPAND(x) STRINGIFY(x)

#define OPEN_RAVE_IK false
#define OPEN_RAVE_J false

extern int NUM_MECH;
extern unsigned long int gTime;
extern struct DOF_type DOF_types[];
extern t_controlmode newRobotControlMode;

int raven_cartesian_space_command(struct device *device0, struct param_pass *currParams);
int raven_joint_velocity_control(struct device *device0, struct param_pass *currParams);
int raven_motor_position_control(struct device *device0, struct param_pass *currParams);
int raven_homing(struct device *device0, struct param_pass *currParams, int begin_homing=0);
int applyTorque(struct device *device0, struct param_pass *currParams);
int raven_sinusoidal_joint_motion(struct device *device0, struct param_pass *currParams);
int raven_joint_torque_command(struct device *device0, struct param_pass *currParams);
NEWMAT::Matrix calculateJacobian(float theta_s, float theta_e, float d);

extern int initialized;

void turnOffSpeedyJoints(device& dev) {
	return;
	extern DOF_type DOF_types[];
	for (int iMech = 0; iMech < 2; iMech++) {
		for (int iJoint = 0; iJoint < 8; iJoint++) {
			DOF& dof = dev.mech[iMech].joint[iJoint];
			if (fabs(dof.mvel) > 10) {
				log_msg("Turning off speedy joint %d on mech %d",iJoint,iMech);
				dof.current_cmd = 0;
			}
		}
	}
}

/**
* controlRaven()
*   Implements control for one loop cycle.
*     precondition:  encoders values have been read,
*                    runlevel has been set
*     postcondition: robot state is reflected in device0,
*                    DAC outputs are set in device0
*
*/
int controlRaven(struct device *device0, struct param_pass *currParams){
	int ret = 0;
    t_controlmode controlmode = (t_controlmode)currParams->robotControlMode;

    //Initi zalization code
    initRobotData(device0, currParams->runlevel, currParams);

    //Compute Mpos & Velocities
    stateEstimate(device0);

    //Foward Cable Coupling
    fwdCableCoupling(device0, currParams->runlevel);

    //Forward kinematics
    fwdKin(device0, currParams->runlevel);

    //log_msg("0 (%d,%d,%d)",device0->mech[0].pos.x,device0->mech[0].pos.y,device0->mech[0].pos.z);
    //log_msg("1 (%d,%d,%d)",device0->mech[1].pos.x,device0->mech[1].pos.y,device0->mech[1].pos.z);

    //log_msg("control mode %d",(int)controlmode);
    switch (controlmode){
        case no_control:
            break;

        case cartesian_space_control:
            //initialized = false;
        	ret = raven_cartesian_space_command(device0,currParams);
            break;

        case motor_pd_control:
            initialized = false;
            ret = raven_motor_position_control(device0,currParams);
            break;

        case joint_velocity_control:
            initialized = false;
            ret = raven_joint_velocity_control(device0, currParams);
            break;

        case homing_mode:
            initialized = false;
            //initialized = robot_ready(device0) ? true:false;
            ret = raven_homing(device0, currParams);
            set_posd_to_pos(device0);
            updateMasterRelativeOrigin(device0);
            if (robot_ready(device0))
            {
                log_msg("Homing finished, switching to cartesian space control");
                currParams->robotControlMode = cartesian_space_control;
                newRobotControlMode = cartesian_space_control;
            }
            break;

        case apply_arbitrary_torque:
            initialized = false;
            ret = applyTorque(device0, currParams);
            break;

        case multi_dof_sinusoid:
            initialized = false;
            ret = raven_sinusoidal_joint_motion(device0, currParams);
            break;

        case joint_torque_control:
        	ret = raven_joint_torque_command(device0,currParams);
            break;
        default:
            printf("got control mode %i\n", (int)controlmode);
            ROS_ERROR("Error: unknown control mode in controlRaven (rt_raven.cpp)");
            ret = -1;
            break;
    }

    if (controlmode != homing_mode) {
    	turnOffSpeedyJoints(*device0);
    }

    return ret;
}

EnvironmentBasePtr env;
RobotBasePtr raven;
vector<RobotBase::ManipulatorPtr> manips;
RobotBase::ManipulatorPtr leftarm;
RobotBase::ManipulatorPtr rightarm;
GeneralIK* iksolver;
int ik_test_counter;

void init_open_rave() {
	if (OPEN_RAVE_IK) {
		log_msg("init openrave");
		RaveInitialize(true);
		env = RaveCreateEnvironment();
		env->Load(EXPAND(RAVEN_DATA_DIR) "/ravenII_2arm.xml");
		raven = env->GetRobot("raven_2");
		manips = 	raven->GetManipulators();
		leftarm = manips[0];
		rightarm = manips[1];
		iksolver = new GeneralIK(env);
		iksolver->Init(leftarm);
		ik_test_counter = 0;
	}
}

/**
* raven_cartesian_space_command()
*     runs pd_control on motor position.
*     Why is it called "end_effector_control?  Why's your mom so fat?
*/
int raven_cartesian_space_command(struct device *device0, struct param_pass *currParams){
	ik_test_counter++;

    struct DOF *_joint = NULL;
    struct mechanism* _mech = NULL;
    int i=0,j=0;

    if (currParams->runlevel != RL_PEDAL_DN) {
        set_posd_to_pos(device0);
        updateMasterRelativeOrigin(device0);
    }

    // Set desired transform to straight down
    /*
    for (int i=0;i<NUM_MECH;i++)
    {
        _mech = &(device0->mech[i]);
        _mech->ori_d.R[0][0] = -1.0;
        _mech->ori_d.R[0][1] = 0.0;
        _mech->ori_d.R[0][2] = 0.0;

        _mech->ori_d.R[1][0] = 0.0;
        _mech->ori_d.R[1][1] = -1.0;
        _mech->ori_d.R[1][2] = 0.0;

        _mech->ori_d.R[2][0] = 0.0;
        _mech->ori_d.R[2][1] = 0.0;
        _mech->ori_d.R[2][2] = 1.0;

    }
    */
    //cout << "ori_d" << device0->mech[0].ori_d.grasp << " " << device0->mech[1].ori_d.grasp << endl;
    //device0->mech[0].ori_d.grasp = 900;
    //device0->mech[1].ori_d.grasp = -900;




    //Inverse kinematics
    invKin(device0, currParams);

    //Inverse Cable Coupling
    invCableCoupling(device0, currParams->runlevel);

    // Set all joints to zero torque
    _mech = NULL;  _joint = NULL;
    while (loop_over_joints(device0, _mech, _joint, i,j) )
    {
        if (currParams->runlevel != RL_PEDAL_DN || _mech->type == GREEN_ARM) {
            _joint->tau_d=0;
        } else {
            mpos_PD_control(_joint);
        }
//        if (_joint->type == TOOL_ROT_GREEN)
//            log_msg("trg: jp:%0.4f\t jpd:%0.4f\t taud:%0.4f",
//                _joint->jpos, _joint->jpos_d, _joint->tau_d);

        TorqueToDAC(device0);
    }
    //    gravComp(device0);

    return 0;
}

device* device0ptr;
int raven_joint_torque_command(struct device *device0, struct param_pass *currParams){
  device0->surgeon_mode=1;
    //Inverse Cable Coupling
    invCableCoupling(device0, currParams->runlevel);

    device0ptr = device0;
    ros::spinOnce();
    TorqueToDAC(device0);

    //    gravComp(device0);

    return 0;
}


/**
* raven_sinusoidal_joint_motion()
*    Applies a sinusoidal trajectory to all joints
*/
int raven_sinusoidal_joint_motion(struct device *device0, struct param_pass *currParams){
    static int controlStart = 0;
    static unsigned long int delay=0;
    const float f_period[8] = {6, 7, 10, 9999999, 10, 5, 10, 6};
//    const float f_magnitude[8] = {0 DEG2RAD, 0 DEG2RAD, 0.0, 9999999, 0 DEG2RAD, 25 DEG2RAD, 0 DEG2RAD, 0 DEG2RAD};
    const float f_magnitude[8] = {10 DEG2RAD, 10 DEG2RAD, 0.02, 9999999, 20 DEG2RAD, 10 DEG2RAD, 10 DEG2RAD, 10 DEG2RAD};

    // If we're not in pedal down or init.init then do nothing.
    if (! ( currParams->runlevel == RL_INIT && currParams->sublevel == SL_AUTO_INIT ))
    {
        controlStart = 0;
        delay = gTime;
        // Set all joints to zero torque, and mpos_d = mpos
        for (int i=0; i < NUM_MECH; i++)
        {
            for (int j = 0; j < MAX_DOF_PER_MECH; j++)
            {
                struct DOF* _joint =  &(device0->mech[i].joint[j]);
                _joint->mpos_d = _joint->mpos;
                _joint->jpos_d = _joint->jpos;
                _joint->tau_d = 0;
            }
        }
        return 0;
    }

    // Wait for amplifiers to power up
    if (gTime - delay < 800)
        return 0;

    // Set trajectory on all the joints
    for (int i=0; i < NUM_MECH; i++)
    {
        for (int j = 0; j < MAX_DOF_PER_MECH; j++)
        {
            struct DOF * _joint =  &(device0->mech[i].joint[j]);
            int sgn = 1;

            if (device0->mech[i].type == GREEN_ARM)
                sgn = -1;

            // initialize trajectory
            if (!controlStart)
                start_trajectory(_joint, (_joint->jpos + sgn*f_magnitude[j]), f_period[j]);

            // Get trajectory update
            update_sinusoid_position_trajectory(_joint);
        }
    }

    //Inverse Cable Coupling
    invCableCoupling(device0, currParams->runlevel);

    // Do PD control on all the joints
    for (int i=0; i < NUM_MECH; i++)
    {
        for (int j = 0; j < MAX_DOF_PER_MECH; j++)
        {
            struct DOF * _joint =  &(device0->mech[i].joint[j]);

            // Do PD control
            mpos_PD_control(_joint);
	    if (_joint->type == TOOL_ROT_GREEN || _joint->type == TOOL_ROT_GOLD)
		_joint->tau_d = 0;
        }
    }


    TorqueToDAC(device0);

    controlStart = 1;
    return 0;
}


/**
*  applyTorque()
*    For debugging robot.  Apply a set torque command (tau_d) to a joint.
*
*/
int applyTorque(struct device *device0, struct param_pass *currParams)
{
    // Only run in runlevel 1.2
    if ( ! (currParams->runlevel == RL_INIT && currParams->sublevel == SL_AUTO_INIT ))
        return 0;

    for (int i=0;i<NUM_MECH;i++)
    {
        for (int j=0;j<MAX_DOF_PER_MECH;j++)
        {
            if (device0->mech[i].type == GOLD_ARM)
            {
                device0->mech[i].joint[j].tau_d = (1.0/1000.0) * (float)(currParams->torque_vals[j]);  // convert from mNm to Nm
            }
            else
            {
                device0->mech[i].joint[j].tau_d = (1.0/1000.0) * (float)(currParams->torque_vals[MAX_DOF_PER_MECH+j]);
            }
        }
    }
    // gravComp(device0);
    TorqueToDAC(device0);

    return 0;
}


/**
* raven_motor_position_control()
*     runs pd control on motor position
*/
int raven_motor_position_control(struct device *device0, struct param_pass *currParams)
{
    static int controlStart = 0;
    static unsigned long int delay=0;

    struct DOF *_joint = NULL;
    struct mechanism* _mech = NULL;
    int i=0,j=0;

    // If we're not in pedal down or init.init then do nothing.
    if (! ( currParams->runlevel == RL_PEDAL_DN ||
          ( currParams->runlevel == RL_INIT     && currParams->sublevel == SL_AUTO_INIT ))) {
        controlStart = 0;
        delay = gTime;

        // Set all joints to zero torque, and mpos_d = mpos
        _mech = NULL;  _joint = NULL;
        while (loop_over_joints(device0, _mech, _joint, i,j) )
        {
            _joint->mpos_d = _joint->mpos;
            _joint->tau_d = 0;
        }
        return 0;
    }

    if (gTime - delay < 800)
        return 0;

    // Set trajectory on all the joints
    _mech = NULL;  _joint = NULL;
    while (loop_over_joints(device0, _mech, _joint, i,j) )
    {

    	// initialize trajectory
    	if (!controlStart && _joint->type == Z_INS_GREEN) {
    		start_trajectory(_joint, 0.08, 8);
    	} else if (!controlStart) {
    		_joint->jpos_d = _joint->jpos;
    	}
    	// Get trajectory update
    	if (_joint->type == Z_INS_GREEN) {
    		update_sinusoid_position_trajectory(_joint);
    	}
    }

    //Inverse Cable Coupling
    invCableCoupling(device0, currParams->runlevel);

    // Do PD control on all the joints
    _mech = NULL;  _joint = NULL;
    while (loop_over_joints(device0, _mech, _joint, i,j) )
    {
        // Do PD control
        mpos_PD_control(_joint);

        if (device0->mech[i].type == GOLD_ARM) {
            _joint->tau_d=0;
        }
    }

    TorqueToDAC(device0);

    controlStart = 1;
    return 0;
}

/**
* raven_joint_velocity_control()
*     runs pi_control on joint velocity.
*/
int raven_joint_velocity_control(struct device *device0, struct param_pass *currParams)
{
    static int controlStart;
    static unsigned long int delay=0;

    // Run velocity control
    if ( currParams->runlevel == RL_PEDAL_DN ||
            ( currParams->runlevel == RL_INIT &&
              currParams->sublevel == SL_AUTO_INIT ))
    {
        // delay the start of control for 300ms b/c the amps have to turn on.
        if (gTime - delay < 800)
            return 0;

        for (int i=0; i < NUM_MECH; i++)
        {
            for (int j = 0; j < MAX_DOF_PER_MECH; j++)
            {
                struct DOF * _joint =  &(device0->mech[i].joint[j]);

                if (device0->mech[i].type == GOLD_ARM)
                {
                    // initialize velocity trajectory
                    if (!controlStart)
                        start_trajectory(_joint);

                    // Get the desired joint velocities
                    update_linear_sinusoid_velocity_trajectory(_joint);

                    // Run PI control
                    jvel_PI_control(_joint, !controlStart);

                }
                else
                {
                    _joint->tau_d = 0;
                }
            }
        }

        if (!controlStart)
            controlStart = 1;

        // Convert joint torque to DAC value.
        TorqueToDAC(device0);
    }

    else
    {
        delay=gTime;
        controlStart = 0;
        for (int i=0; i < NUM_MECH; i++)
            for (int j = 0; j < MAX_DOF_PER_MECH; j++)
            {
                device0->mech[i].joint[j].tau_d=0;
            }
        TorqueToDAC(device0);
    }

    return 0;
}

inline NEWMAT::ColumnVector cross(NEWMAT::ColumnVector a, NEWMAT::ColumnVector b) {
	NEWMAT::ColumnVector result(3);
	result(1) = a(2)*b(3) - a(3)*b(2);
	result(2) = a(3)*b(1) - a(1)*b(3);
	result(3) = a(1)*b(2) - a(2)*b(1);
	return result;
}

NEWMAT::Matrix calculateJacobian(float theta_s, float theta_e, float d) {
	NEWMAT::Matrix U01(3,3);
	NEWMAT::Matrix U12(3,3);
	NEWMAT::IdentityMatrix U23(3);
	NEWMAT::IdentityMatrix U3E(3);

	U01(1,1) = cos(theta_s);
	U01(1,2) = -sin(theta_s) * cos(A12);
	U01(1,3) = sin(theta_s) * sin(A12);
	U01(2,1) = sin(theta_s);
	U01(2,2) = cos(theta_s) * cos(A12);
	U01(2,3) = -cos(theta_s) * sin(A12);
	U01(3,1) = 0;
	U01(3,2) = sin(A12);
	U01(3,3) = cos(A12);

	U12(1,1) = cos(theta_e);
	U12(1,2) = -sin(theta_e) * cos(A23);
	U12(1,3) = sin(theta_e) * sin(A23);
	U12(2,1) = sin(theta_e);
	U12(2,2) = cos(theta_e) * cos(A23);
	U12(2,3) = -cos(theta_e) * sin(A23);
	U12(3,1) = 0;
	U12(3,2) = sin(A23);
	U12(3,3) = cos(A23);

	NEWMAT::ColumnVector p1(3);
	NEWMAT::ColumnVector p2(3);
	NEWMAT::ColumnVector p3(3);
	NEWMAT::ColumnVector pE(3);
	p1 = 0.;
	p2 = 0.;
	p3 = 0.; p3(3) = d;
	pE = 0.;

	NEWMAT::IdentityMatrix UEE(3);

	NEWMAT::Matrix UE3 = UEE * U3E.t();
	NEWMAT::Matrix UE2 = UE3 * U23.t();
	NEWMAT::Matrix UE1 = UE2 * U12.t();
	NEWMAT::Matrix UE0 = UE1 * U01.t();

	NEWMAT::ColumnVector zhat(3);
	zhat(1) = 0; zhat(2) = 0; zhat(3) = 1;

	NEWMAT::ColumnVector gammaE1 = UE0 * zhat;
	NEWMAT::ColumnVector gammaE2 = UE1 * zhat;
	NEWMAT::ColumnVector gammaE3(3);

	NEWMAT::ColumnVector rEE(3);
	rEE = 0.0;

	NEWMAT::ColumnVector rE3 = rEE - UEE * pE;
	NEWMAT::ColumnVector rE2 = rE3 - UE3 * p3;
	NEWMAT::ColumnVector rE1 = rE2 - UE2 * p2;
	NEWMAT::ColumnVector rE0 = rE1 - UE1 * p1;

	NEWMAT::ColumnVector betaE1 = cross(gammaE1, -1 * rE0);
	NEWMAT::ColumnVector betaE2 = cross(gammaE2, -1 * rE1);
	NEWMAT::ColumnVector betaE3 = UE2 * zhat;

	NEWMAT::Matrix J(6,3);
	for (int i=1;i<=3;i++) { J(i,1) = gammaE1(i); }
	for (int i=1;i<=3;i++) { J(i+3,1) = betaE1(i); }
	for (int i=1;i<=3;i++) { J(i,2) = gammaE2(i); }
	for (int i=1;i<=3;i++) { J(i+3,2) = betaE2(i); }
	for (int i=1;i<=3;i++) { J(i,3) = gammaE3(i); }
	for (int i=1;i<=3;i++) { J(i+3,3) = betaE3(i); }

	return J;
}



