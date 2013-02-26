/**
* File: rt_raven.cpp
* Created 13-oct-2011 by Hawkeye
*
*   Runs all raven control functions.
*   Code split out from rt_process_preempt.cpp, in order to provide more flexibility.
*
*/

#include <ros/ros.h>
#include <math.h>

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

#include "shared_modes.h"
#include "saveload.h"

#include <raven/state/runlevel.h>
#include <raven/state/device.h>
#include <raven/control/control_input.h>

using namespace std;


#ifndef STRINGIFY
#define STRINGIFY(s) STRINGIFY_HELPER(s)
#define STRINGIFY_HELPER(s) #s
#endif

extern bool disable_arm_id[2];
extern int NUM_MECH;
extern unsigned long int gTime;
extern struct DOF_type DOF_types[];
extern t_controlmode newRobotControlMode;

typedef int(*controller)(struct device*,struct param_pass*);

int raven_cartesian_space_command (struct device *device0, struct param_pass *currParams);
int raven_joint_velocity_control  (struct device *device0, struct param_pass *currParams);
int raven_motor_position_control  (struct device *device0, struct param_pass *currParams);
int raven_homing                  (struct device *device0, struct param_pass *currParams, int begin_homing=0);
int applyTorque                   (struct device *device0, struct param_pass *currParams);
int raven_sinusoidal_joint_motion (struct device *device0, struct param_pass *currParams);
int raven_joint_torque_command    (struct device *device0, struct param_pass *currParams);
//NEWMAT::Matrix calculateJacobian(float theta_s, float theta_e, float d);

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

controller getController(t_controlmode mode) {
	switch (mode) {
	case no_control:
		return 0;
	case end_effector_control:
	case cartesian_space_control:
		return &raven_cartesian_space_command;
	case motor_pd_control:
//		initialized = false;
		return &raven_motor_position_control;
	case joint_velocity_control:
//		initialized = false;
		return &raven_joint_velocity_control;
	case apply_arbitrary_torque:
//		initialized = false;
		return &applyTorque;
	case multi_dof_sinusoid:
//		initialized = false;
		return &raven_sinusoidal_joint_motion;
	case joint_torque_control:
		return &raven_joint_torque_command;
	default:
		printf("got control mode %i\n", (int)mode);
		ROS_ERROR("Error: unknown control mode %i in controlRaven (rt_raven.cpp)",(int)mode);
		return 0;
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
	int ret = -1;
    //t_controlmode controlmode = (t_controlmode)currParams->robotControlMode;
	t_controlmode controlmode = getControlMode();

    //Initi zalization code
    initRobotData(device0, currParams->runlevel, currParams);

    //Compute Mpos & Velocities
    stateEstimate(device0);

    //Foward Cable Coupling
    fwdCableCoupling(device0, currParams->runlevel);

    {
    	static bool printed_warning = false;
    	if (!printed_warning) {
    		log_err("************DISABLING GRASP2***************");
    		printed_warning = true;
    	}
    	struct mechanism* _mech = NULL;
    	struct DOF *_joint = NULL;
		int i=0,j=0;
		float jp;
		while (loop_over_mechs(device0,_mech,i)) {
			if (armIdFromMechType(_mech->type) == GOLD_ARM_ID) {
				_mech->joint[GRASP2].jpos = - _mech->joint[GRASP1].jpos;
				break;
			}
		}
		DevicePtr dev = Device::currentNoCloneMutable();
		FOREACH_ARM_IN_DEVICE(arm,dev) {
			if (arm->isGold()) {
				arm->getJointById(Joint::IdType::FINGER2_)->setPosition(-arm->getJointById(Joint::IdType::FINGER1_)->position());
				break;
			}
		}
    }


    //Forward kinematics
    fwdKin(device0, currParams->runlevel);

    //log_msg("0 (%d,%d,%d)",device0->mech[0].pos.x,device0->mech[0].pos.y,device0->mech[0].pos.z);
    //log_msg("1 (%d,%d,%d)",device0->mech[1].pos.x,device0->mech[1].pos.y,device0->mech[1].pos.z);

    if (controlmode == homing_mode) {
#ifdef USE_NEW_RUNLEVEL
    	RunLevel::setInitialized(false);
#else
    	initialized = false;
#endif
		//initialized = robot_ready(device0) ? true:false;
		ret = raven_homing(device0, currParams);
		set_posd_to_pos(device0);
		updateMasterRelativeOrigin(device0);
		if (robot_ready(device0))
		{
			log_msg("Homing finished, switching to cartesian space control");
			saveOffsets(*device0);
			//currParams->robotControlMode = cartesian_space_control;
			setRobotControlMode(cartesian_space_control);
		}
    } else {
    	bool skip = false;
    	bool traj_ctrl = false;
    	TrajectoryStatus param_status;
    	if (controlmode == trajectory_control) {
    		traj_ctrl = true;

    		param_pass traj_params;
    		param_status = getCurrentTrajectoryParams(controlmode,traj_params);
    		//printf("Got traj with status %i\n",param_status.value());

    		switch (param_status.value()) {
    		case TrajectoryStatus::OK:
#ifdef USE_NEW_RUNLEVEL
    			RunLevel::get().getNumbers<u_08>(traj_params.runlevel,traj_params.sublevel);
#else
    			traj_params.runlevel = currParams->runlevel;
    			traj_params.sublevel = currParams->sublevel;
#endif
    			*currParams = traj_params;
    			break;
    		case TrajectoryStatus::BEFORE_START:
    			set_posd_to_pos(device0);
    			skip = true;
    			break;
    		case TrajectoryStatus::NO_TRAJECTORY:
    		default:
    			log_msg_throttle(0.25,"Trajectory control has no trajectory!");
    			/* no break */
    		case TrajectoryStatus::ENDED:
    			//printf("Trajectory ended\n");
    			set_posd_to_pos(device0);
    			printf("Setting control mode to cartestian space\n");
#ifdef USE_NEW_RUNLEVEL
    			//RunLevel::setPedal(false);
    			RunLevel::setArmActive(Arm::ALL_ARMS,false);
#else
    			device0->surgeon_mode = false;
    			currParams->runlevel = RL_PEDAL_UP;
#endif
    			setRobotControlMode(cartesian_space_control);
    			skip = true;
    			break;
    		}
    	}/* else {
    		clearTrajectory();
    	}*/

    	if (!skip) {
			controller ctlr = getController(controlmode);

			if (ctlr) {
				if (traj_ctrl) {
					//log_msg_throttle(0.25,"Traj ctrl with controller %i %i RL(%i,%i)\n",(int) controlmode,(int)currParams->surgeon_mode,currParams->runlevel,currParams->sublevel);
					log_msg_throttle(0.25,"control pt: (%i,%i,%i)",currParams->xd[0].x,currParams->xd[0].y,currParams->xd[0].z);
				}
				ret = (*ctlr)(device0,currParams);
			} else {
				if (traj_ctrl) {
#ifdef USE_NEW_RUNLEVEL
					log_msg_throttle(0.25,"Traj ctrl didn't find controller %i %i RL: %s\n",(int) controlmode,(int)RunLevel::getPedal(),RunLevel::get().str().c_str());
#else
					log_msg_throttle(0.25,"Traj ctrl didn't find controller %i %i RL(%i,%i)\n",(int) controlmode,(int)currParams->surgeon_mode,currParams->runlevel,currParams->sublevel);
#endif
				}
				set_posd_to_pos(device0);
			}
    	} else if (traj_ctrl) {
    		log_msg_throttle(0.25,"Traj ctrl with status %i\n",param_status.value());
    	}
    }

    /*
    //log_msg("control mode %d",(int)controlmode);
    switch (controlmode){
        case no_control:
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
                resetControlMode();
            }
            break;

        case end_effector_control:
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
    */

    if (controlmode != homing_mode) {
    	turnOffSpeedyJoints(*device0);
    }

    return ret;
}

/**
* raven_cartesian_space_command()
*     runs pd_control on motor position.
*     Why is it called "end_effector_control?  Why's your mom so fat?
*/
int raven_cartesian_space_command(struct device *device0, struct param_pass *currParams){
	struct DOF *_joint = NULL;
    struct mechanism* _mech = NULL;
    int i=0,j=0;

#ifdef USE_NEW_RUNLEVEL
    if (!RunLevel::get().isPedalDown()) {
#else
    if (currParams->runlevel != RL_PEDAL_DN) {
#endif
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

    static ros::NodeHandle* nh = 0;
    if (!nh) {
    	nh = new ros::NodeHandle("/");
    }

    /*
    while (loop_over_joints(device0, _mech, _joint, i,j) ) {
    	static double* smd = 0;
    	static double* emd = 0;
    	static double* imd = 0;
    	static double* rmd = 0;
    	static double* wmd = 0;
    	static double* gmd = 0;

    	int type = jointTypeFromCombinedType(_joint->type);
    	double max_diff = 10000;
    	switch (type) {
    	case SHOULDER:
    		if (!smd) {
    			smd = new double;
    			nh->param("ik/max_diff/shoulder",*smd,10.);
    			*smd = *smd DEG2RAD;
    		}
    		max_diff = *smd;
    		break;
    	case ELBOW:
    		if (!emd) {
    			emd = new double;
    			nh->param("ik/max_diff/elbow",*emd,10.);
    			*emd = *emd DEG2RAD;
    		}
    		max_diff = *emd;
    		break;
    	case Z_INS:
    		if (!imd) {
    			imd = new double;
    			nh->param("ik/max_diff/insertion",*imd,0.005);
    		}
    		max_diff = *imd;
    		break;
    	case TOOL_ROT:
    		if (!rmd) {
    			rmd = new double;
    			nh->param("ik/max_diff/rotation",*rmd,10.);
    			*rmd = *rmd DEG2RAD;
    		}
    		max_diff = *rmd;
    		break;
    	case WRIST:
    		if (!wmd) {
    			wmd = new double;
    			nh->param("ik/max_diff/wrist",*wmd,10.);
    			*wmd = *wmd DEG2RAD;
    		}
    		max_diff = *wmd;
    		break;
    	case GRASP1:
    	case GRASP2:
    		if (!gmd) {
    			gmd = new double;
    			nh->param("ik/max_diff/grasp",*gmd,10.);
    			*gmd = *gmd DEG2RAD;
    		}
    		max_diff = *gmd;
    		break;
    	}

    	if (fabs(_joint->jpos_d - _joint->jpos) > max_diff) {
			if (_joint->jpos_d > _joint->jpos) {
				_joint->jpos_d = _joint->jpos + max_diff;
			} else {
				_joint->jpos_d = _joint->jpos - max_diff;
			}
		}
    }
    */

    //Inverse Cable Coupling
    invCableCoupling(device0, currParams->runlevel);

    // Set all joints to zero torque
    _mech = NULL;  _joint = NULL;
    while (loop_over_joints(device0, _mech, _joint, i,j) )
    {

    	static bool printed_warning = false;
    	if (_joint->type == GRASP2_GOLD) {
    		if (!printed_warning) {
    			log_err("************DISABLING GRASP2***************");
    			printed_warning = true;
    		}
    		_joint->mpos_d = _joint->mpos;
    	}
#ifdef USE_NEW_RUNLEVEL
    	if (!RunLevel::get().isPedalDown() || disable_arm_id[armIdFromMechType(_mech->type)]) {
#else
    	if (currParams->runlevel != RL_PEDAL_DN || disable_arm_id[armIdFromMechType(_mech->type)]) {
#endif
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

//device* device0ptr;
int raven_joint_torque_command(struct device *device0, struct param_pass *currParams){
  device0->surgeon_mode=1;
    //Inverse Cable Coupling
    invCableCoupling(device0, currParams->runlevel);

    //device0ptr = device0;
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

    struct DOF *_joint = NULL;
    struct mechanism* _mech = NULL;
    int i=0,j=0;

    // If we're not in pedal down or init.init then do nothing.
#ifdef USE_NEW_RUNLEVEL
    if (! RunLevel::get().isInitSublevel(3))
#else
    if (! ( currParams->runlevel == RL_INIT && currParams->sublevel == SL_AUTO_INIT ))
#endif
    {
        controlStart = 0;
        delay = gTime;
        // Set all joints to zero torque, and mpos_d = mpos
        while (loop_over_joints(device0, _mech, _joint, i,j) ) {
                _joint->mpos_d = _joint->mpos;
                _joint->jpos_d = _joint->jpos;
                _joint->tau_d = 0;
        }

#ifdef USE_NEW_DEVICE
        OldControlInputPtr input = ControlInput::oldControlInputUpdateBegin();
    	FOREACH_ARM_IN_CONST_DEVICE(arm,Device::currentNoClone()) {
    		OldArmInputData& armData = input->armById(arm->id());
    		for (size_t i=0;i<arm->motors().size();i++) {
    			armData.motorTorque(i) = 0;
    			armData.motorPosition(i) = arm->motor(i)->position();
    		}
    		for (size_t i=0;i<arm->joints().size();i++) {
    			armData.jointPosition(i) = arm->joint(i)->position();
    		}
    	}
    	ControlInput::oldControlInputUpdateEnd();
#endif
        return 0;
    }

    // Wait for amplifiers to power up
    if (gTime - delay < 800)
        return 0;

    _joint = NULL;
    _mech = NULL;
    // Set trajectory on all the joints
    while (loop_over_joints(device0, _mech, _joint, i,j) ) {
            int sgn = 1;

            if (_mech->type == GREEN_ARM)
                sgn = -1;

            // initialize trajectory
            if (!controlStart)
                start_trajectory(_joint, (_joint->jpos + sgn*f_magnitude[j]), f_period[j]);

            // Get trajectory update
            update_sinusoid_position_trajectory(_joint);
    }

    //Inverse Cable Coupling
    invCableCoupling(device0, currParams->runlevel);

    _joint = NULL;
    _mech = NULL;
    // Do PD control on all the joints
    while (loop_over_joints(device0, _mech, _joint, i,j) ) {
    	// Do PD control
    	mpos_PD_control(_joint);
    	if (_joint->type == TOOL_ROT_GREEN || _joint->type == TOOL_ROT_GOLD) {
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
#ifdef USE_NEW_RUNLEVEL
	if ( !RunLevel::get().isInitSublevel(3))
#else
	if ( ! (currParams->runlevel == RL_INIT && currParams->sublevel == SL_AUTO_INIT ))
#endif
		return 0;

	struct DOF *_joint = NULL;
	struct mechanism* _mech = NULL;
	int i=0,j=0;
	while (loop_over_joints(device0, _mech, _joint, i,j) ) {
		if (_mech->type == GOLD_ARM) {
			_joint->tau_d = (1.0/1000.0) * (float)(currParams->torque_vals[j]);  // convert from mNm to Nm
		} else {
			_joint->tau_d = (1.0/1000.0) * (float)(currParams->torque_vals[MAX_DOF_PER_MECH+j]);
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
#ifdef USE_NEW_RUNLEVEL
    RunLevel rl = RunLevel::get();
    if (!( rl.isPedalDown() || rl.isInitSublevel(3))) {
#else
    if (! ( currParams->runlevel == RL_PEDAL_DN ||
          ( currParams->runlevel == RL_INIT     && currParams->sublevel == SL_AUTO_INIT ))) {
#endif
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

//        if (device0->mech[i].type == GOLD_ARM) {
//            _joint->tau_d=0;
//        }
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

    struct DOF *_joint = NULL;
    struct mechanism* _mech = NULL;
    int i=0,j=0;

    // Run velocity control
#ifdef USE_NEW_RUNLEVEL
    RunLevel rl = RunLevel::get();
    if ( rl.isPedalDown() || rl.isInitSublevel(3)) {
#else
    if ( currParams->runlevel == RL_PEDAL_DN ||
            ( currParams->runlevel == RL_INIT &&
              currParams->sublevel == SL_AUTO_INIT )) {
#endif
        // delay the start of control for 300ms b/c the amps have to turn on.
        if (gTime - delay < 800)
        	return 0;

        while (loop_over_joints(device0, _mech, _joint, i,j) ) {
        	if (device0->mech[i].type == GOLD_ARM) {
        		// initialize velocity trajectory
        		if (!controlStart)
        			start_trajectory(_joint);

        		// Get the desired joint velocities
        		update_linear_sinusoid_velocity_trajectory(_joint);

        		// Run PI control
        		jvel_PI_control(_joint, !controlStart);

        	} else {
        		_joint->tau_d = 0;
        	}
        }

        if (!controlStart)
            controlStart = 1;

        // Convert joint torque to DAC value.
        TorqueToDAC(device0);
    } else {
    	delay=gTime;
    	controlStart = 0;
    	while (loop_over_joints(device0, _mech, _joint, i,j) ) {
    		_joint->tau_d=0;
    	}
    	TorqueToDAC(device0);
    }

    return 0;
}



