/*
 * init.c - contains functions for initializing the robot
 *
 * Kenneth Fodero
 * Mitch Lum
 * Biorobotics Lab
 * Started July 29, 2005
 *
 * Modified by Hawkeye King, Peter Felts, Brian and Jason.
 *
 */

#include <ros/console.h>

#include "init.h"
#include "USB_init.h"
#include "local_io.h"
#include "saveload.h"

#include <raven/state/device.h>
#include <raven/state/runlevel.h>
#include <raven/control/control_input.h>

#include <ros/ros.h>

extern int initialized;

extern struct traj trajectory[];
extern struct DOF_type DOF_types[];

extern char usb_board_count;
extern USBStruct USBBoards;
extern int NUM_MECH;
extern int soft_estopped;

/**
 * init() - intializes the DOF structure AND runs initialization routine.  *sigh*
 *
 * \param device0 pointer to device struct.
 * \param runlevel Runlevel
 * \param currParams pointer to param struct containing current params?
 */

void initRobotData(struct device *device0, int runlevel, struct param_pass *currParams)
{
    // init_wait_loop is a klugy way to wait a few times through the loop for our kinematics to propogate.
    static int init_wait_loop=0;

    RunLevel rl = RunLevel::get();
    //log_msg_throttle(0.25,"rl %s",rl.str().c_str());

    //In ESTOP reset initialization
#ifdef USE_NEW_RUNLEVEL
    if (rl.isEstop()) {
    	RunLevel::setInitialized(false);
    }
#else
    if (runlevel == RL_E_STOP) {
    	initialized = FALSE;
    }
#endif

#ifdef USE_NEW_RUNLEVEL
    if (rl.isSoftwareEstop()) {
#else
    if (soft_estopped) {
#endif
    	//printf("SOFT ESTOPPED iRD\n");
        device0->mech[0].joint[0].state=jstate_pos_unknown;
#ifdef USE_NEW_RUNLEVEL
        return;
#endif
    }

    //Do nothing if we are not in the init runlevel
#ifdef USE_NEW_RUNLEVEL
    if (!rl.isInit()) {
#else
    if (runlevel != RL_INIT) {
#endif
        return;
    }

#ifdef USE_NEW_RUNLEVEL
    if (rl.isInitSublevel(0)) {
    	//printf("init 0\n");
    	RunLevel::setSublevel(1);
    } else if (rl.isInitSublevel(1)) {
    	if (RunLevel::isInitialized()) {
    		return;
    	}
    	initDOFs(device0);
    	setStartXYZ(device0);      // Set pos_d = current position

    	RunLevel::setSublevel(2);
    	//log_msg("    -> sublevel %d", currParams->sublevel);
    } else if (rl.isInitSublevel(2)) {
    	// Automatically jump to next sublevel after small delay
    	init_wait_loop++;
    	setStartXYZ(device0);                 // set cartesian pos_d = current position

    	if ( init_wait_loop > 10) {
    		// Go to auto init sublevel
    		RunLevel::setSublevel(3);
    		init_wait_loop=0;
    	}
    } else if (rl.isInitSublevel(3)) {
    	RunLevel::setInitialized(true);
    }

    return;
#else
    switch (currParams->sublevel)
    {
    case 0:
        {
            currParams->sublevel = 1;     // Goto sublevel 1 to allow initial jpos_d setup by inv_kin.
            RunLevel::setSublevel(1);
        }
    case 1:     // Initialization off all joint variables
        if (initialized)     //If already initialized do nothing
        {
            break;
        }

        initDOFs(device0);
        setStartXYZ(device0);      // Set pos_d = current position

        currParams->sublevel = 2;     // Goto sublevel 1 to allow initial jpos_d setup by inv_kin.
        RunLevel::setSublevel(2);
        log_msg("    -> sublevel %d", currParams->sublevel);
        break;

    case 2:
        // Automatically jump to next sublevel after small delay
        init_wait_loop++;
        setStartXYZ(device0);                 // set cartesian pos_d = current position

        if ( init_wait_loop > 10)
        {
            // Go to auto init sublevel
            currParams->sublevel = SL_AUTO_INIT;
            RunLevel::setSublevel(SL_AUTO_INIT);
            init_wait_loop=0;
        }
        break;

    case SL_AUTO_INIT:
        initialized = TRUE;                  // Set initialized flag
    }

    return;
#endif
}

/**
 * initDOFStructs() - intializes all structures which are not DOF specific
 *
 * \param device0 pointer to device struct.
 *
 */
  
void initDOFs(struct device *device0)
{
    static int dofs_inited=0;
    if (dofs_inited)
        return;
    /// Set transmission ratios
    //    Yes, the numbering is wierd (TOOL_ROT and Z_INS are physically 4th & 3rd respectively
    //    See defines.h for explaination
    DOF_types[SHOULDER_GOLD].TR   = SHOULDER_TR_GOLD_ARM;
    DOF_types[ELBOW_GOLD].TR      = ELBOW_TR_GOLD_ARM;
    DOF_types[Z_INS_GOLD].TR      = Z_INS_TR_GOLD_ARM;
    DOF_types[TOOL_ROT_GOLD].TR   = TOOL_ROT_TR_GOLD_ARM;
    DOF_types[WRIST_GOLD].TR      = WRIST_TR_GOLD_ARM;
    DOF_types[GRASP1_GOLD].TR     = GRASP1_TR_GOLD_ARM;
    DOF_types[GRASP2_GOLD].TR     = GRASP2_TR_GOLD_ARM;

    DOF_types[SHOULDER_GREEN].TR  = SHOULDER_TR_GREEN_ARM;
    DOF_types[ELBOW_GREEN].TR     = ELBOW_TR_GREEN_ARM;
    DOF_types[Z_INS_GREEN].TR     = Z_INS_TR_GREEN_ARM;
    DOF_types[TOOL_ROT_GREEN].TR  = TOOL_ROT_TR_GREEN_ARM;
    DOF_types[WRIST_GREEN].TR     = WRIST_TR_GREEN_ARM;
    DOF_types[GRASP1_GREEN].TR    = GRASP1_TR_GOLD_ARM;
    DOF_types[GRASP2_GREEN].TR    = GRASP2_TR_GREEN_ARM;

    /// Initialize current limits
    DOF_types[SHOULDER_GOLD].DAC_max  = SHOULDER_MAX_DAC;
    DOF_types[SHOULDER_GREEN].DAC_max = SHOULDER_MAX_DAC;
    DOF_types[ELBOW_GOLD].DAC_max     = ELBOW_MAX_DAC;
    DOF_types[ELBOW_GREEN].DAC_max    = ELBOW_MAX_DAC;
    DOF_types[TOOL_ROT_GOLD].DAC_max  = TOOL_ROT_MAX_DAC;
    DOF_types[TOOL_ROT_GREEN].DAC_max = TOOL_ROT_MAX_DAC;
    DOF_types[Z_INS_GOLD].DAC_max     = Z_INS_MAX_DAC;
    DOF_types[Z_INS_GREEN].DAC_max    = Z_INS_MAX_DAC;
    DOF_types[WRIST_GOLD].DAC_max     = WRIST_MAX_DAC;
    DOF_types[WRIST_GREEN].DAC_max    = WRIST_MAX_DAC;
    DOF_types[GRASP1_GOLD].DAC_max    = GRASP1_MAX_DAC;
    DOF_types[GRASP1_GREEN].DAC_max   = GRASP1_MAX_DAC;
    DOF_types[GRASP2_GOLD].DAC_max    = GRASP2_MAX_DAC;
    DOF_types[GRASP2_GREEN].DAC_max   = GRASP2_MAX_DAC;

#define SHOULDER_MAX_ANGLE   0.0
#define ELBOW_MAX_ANGLE      3*M_PI/4
#define Z_INS_MAX_ANGLE      0.1    // NOT THE REAL LIMIT
#define TOOL_ROT_MAX_ANGLE   317 DEG2RAD
#define WRIST_MAX_ANGLE      105 DEG2RAD
#define GRASP1_MAX_ANGLE     120 DEG2RAD //120 DEG2RAD
#define GRASP2_MAX_ANGLE     120 DEG2RAD //130 DEG2RAD


    ros::NodeHandle nh("/");

    double rotation_max_adj_gold = 0;
    double rotation_max_adj_green = 0;

    if (nh.hasParam("max_position/rotation/gold")) {
    	nh.getParam("max_position/rotation/gold",rotation_max_adj_gold);
    }
    if (nh.hasParam("max_position/rotation/green")) {
    	nh.getParam("max_position/rotation/green",rotation_max_adj_green);
    }

    double wrist_max_adj_gold = 0;
	double wrist_max_adj_green = 0;
    if (nh.hasParam("max_position/wrist/gold")) {
    	nh.getParam("max_position/wrist/gold",wrist_max_adj_gold);
    }
    if (nh.hasParam("max_position/wrist/green")) {
    	nh.getParam("max_position/wrist/green",wrist_max_adj_green);
    }

    double grasp_max_adj_gold = 0;
	double grasp_max_adj_green = 0;
	if (nh.hasParam("max_position/grasp/gold")) {
		nh.getParam("max_position/grasp/gold",grasp_max_adj_gold);
	}
	if (nh.hasParam("max_position/grasp/green")) {
		nh.getParam("max_position/grasp/green",grasp_max_adj_green);
	}

    DOF_types[SHOULDER_GOLD].max_position  = SHOULDER_MAX_ANGLE;
    DOF_types[SHOULDER_GREEN].max_position = SHOULDER_MAX_ANGLE;
    DOF_types[ELBOW_GOLD].max_position     = ELBOW_MAX_ANGLE;
    DOF_types[ELBOW_GREEN].max_position    = ELBOW_MAX_ANGLE;
    DOF_types[TOOL_ROT_GOLD].max_position  = TOOL_ROT_MAX_ANGLE + rotation_max_adj_gold DEG2RAD;
    DOF_types[TOOL_ROT_GREEN].max_position = TOOL_ROT_MAX_ANGLE + rotation_max_adj_green DEG2RAD;
    DOF_types[Z_INS_GOLD].max_position     = Z_INS_MAX_ANGLE;
    DOF_types[Z_INS_GREEN].max_position    = Z_INS_MAX_ANGLE;
    DOF_types[WRIST_GOLD].max_position     = WRIST_MAX_ANGLE + wrist_max_adj_gold DEG2RAD;
    DOF_types[WRIST_GREEN].max_position    = WRIST_MAX_ANGLE + wrist_max_adj_green DEG2RAD;
    DOF_types[GRASP1_GOLD].max_position    = GRASP1_MAX_ANGLE + grasp_max_adj_gold DEG2RAD;
    DOF_types[GRASP1_GREEN].max_position   = -GRASP1_MAX_ANGLE - grasp_max_adj_gold DEG2RAD;
    DOF_types[GRASP2_GOLD].max_position    = GRASP2_MAX_ANGLE + grasp_max_adj_green DEG2RAD;
    DOF_types[GRASP2_GREEN].max_position   = -GRASP2_MAX_ANGLE - grasp_max_adj_green DEG2RAD;

    DOF_types[SHOULDER_GOLD].home_position  = SHOULDER_HOME_ANGLE;
    DOF_types[SHOULDER_GREEN].home_position = SHOULDER_HOME_ANGLE;
    DOF_types[ELBOW_GOLD].home_position     = ELBOW_HOME_ANGLE;
    DOF_types[ELBOW_GREEN].home_position    = ELBOW_HOME_ANGLE;
    DOF_types[TOOL_ROT_GOLD].home_position  = TOOL_ROT_HOME_ANGLE;
    DOF_types[TOOL_ROT_GREEN].home_position = TOOL_ROT_HOME_ANGLE;
    DOF_types[Z_INS_GOLD].home_position     = Z_INS_HOME_ANGLE;
    DOF_types[Z_INS_GREEN].home_position    = Z_INS_HOME_ANGLE;
    DOF_types[WRIST_GOLD].home_position     = WRIST_HOME_ANGLE;
    DOF_types[WRIST_GREEN].home_position    = WRIST_HOME_ANGLE;
    DOF_types[GRASP1_GOLD].home_position    = GRASP1_HOME_ANGLE;
    DOF_types[GRASP1_GREEN].home_position   = -GRASP1_HOME_ANGLE;
    DOF_types[GRASP2_GOLD].home_position    = GRASP2_HOME_ANGLE;
    DOF_types[GRASP2_GREEN].home_position   = -GRASP2_HOME_ANGLE;

    /// Initialize values of joint and DOF structures
    for (int i = 0; i < NUM_MECH; i++)
    {

    	btTransform basePose;

        /// Initialize joint types
        if ( device0->mech[i].type == GOLD_ARM)
        {
            log_msg("    Initing gold arm");
            //Set DOF type to unique index
            device0->mech[i].joint[SHOULDER].type = SHOULDER_GOLD;
            device0->mech[i].joint[ELBOW].type    = ELBOW_GOLD;
            device0->mech[i].joint[Z_INS].type    = Z_INS_GOLD;
            device0->mech[i].joint[TOOL_ROT].type = TOOL_ROT_GOLD;
            device0->mech[i].joint[WRIST].type    = WRIST_GOLD;
            device0->mech[i].joint[GRASP1].type   = GRASP1_GOLD;
            device0->mech[i].joint[GRASP2].type   = GRASP2_GOLD;
            device0->mech[i].joint[NO_CONNECTION].type   = NO_CONNECTION_GOLD;

            basePose = btTransform::getIdentity();
        }
        else if (device0->mech[i].type == GREEN_ARM)
        {
            log_msg("    Initing green arm");
            device0->mech[i].joint[SHOULDER].type = SHOULDER_GREEN;
            device0->mech[i].joint[ELBOW].type    = ELBOW_GREEN;
            device0->mech[i].joint[Z_INS].type    = Z_INS_GREEN;
            device0->mech[i].joint[TOOL_ROT].type = TOOL_ROT_GREEN;
            device0->mech[i].joint[WRIST].type    = WRIST_GREEN;
            device0->mech[i].joint[GRASP1].type   = GRASP1_GREEN;
            device0->mech[i].joint[GRASP2].type   = GRASP2_GREEN;
            device0->mech[i].joint[NO_CONNECTION].type   = NO_CONNECTION_GREEN;

            basePose = GREEN_ARM_BASE_POSE;
        }

        device0->mech[i].base_pos.x = basePose.getOrigin().x();
        device0->mech[i].base_pos.y = basePose.getOrigin().y();
        device0->mech[i].base_pos.z = basePose.getOrigin().z();

        for (int i_ = 0; i_ < 3; i_++) {
        	for (int j = 0; j < 3; j++) {
        		device0->mech[i_].base_ori.R[i_][j] = basePose.getBasis()[i_][j];
        	}
        }


        for (int j = 0; j < MAX_DOF_PER_MECH; j++)
        {
            struct DOF *_joint    = &(device0->mech[i].joint[j]);
            int dofindex = _joint->type;
            struct DOF_type *_dof = &(DOF_types[dofindex]);

            _dof->speed_limit = fabs(_dof->max_position - _dof->home_position)/5; // total range of motion in 5 seconds

            _joint->cmd_type = JOINT_COMMAND_TYPE_TORQUE;

            //Initialize joint and motor position variables
            _joint->jpos = 0;
            _joint->mpos = 0;
            _joint->jpos_d = 0;
            _joint->jpos_d_old = 0;
            _joint->mpos_d = 0;
            _joint->mpos_d_old = 0;

            //Initialize the velocity storage elements
            _joint->jvel_d = 0;
            _joint->jvel = 0;
            _joint->mvel_d = 0;
            _joint->mvel = 0;

            //Initialize the position and velocity history for filter
            for (int k = 0; k < HISTORY_SIZE; k++)
            {
                _dof->old_mpos[k] = 0;
                _dof->old_mpos_d[k] = 0;
                _dof->old_mvel[k] = 0;
                _dof->old_mvel_d[k] = 0;
            }
            _dof->filterRdy=0;

            //Set inital current command to zero
            _joint->current_cmd = 0;

            // on R_II torque convention is opposite for green/gold arms
            float torque_sign = 1;
#ifdef RAVEN_II

            // Positive torque -> positive joint angle change
            // Make sure encoders line up the same way.
            if (device0->mech[i].type == GOLD_ARM)
                torque_sign = -1;
            else if (device0->mech[i].type == GREEN_ARM)
                torque_sign = 1;
            else
                err_msg("Unknown mech type ini init!");
#endif
            // Set i-max and current-torque conversion constants
            if ( (j==SHOULDER) || (j==ELBOW) || (j==Z_INS) )
            {
                _dof->tau_per_amp = torque_sign * (float)(T_PER_AMP_BIG_MOTOR * GEAR_BOX_TR_BIG_MOTOR);  // Amps to torque
                _dof->DAC_per_amp = (float)(K_DAC_PER_AMP_HIGH_CURRENT);                   // DAC counts to AMPS
                _dof->i_max = (float)(I_MAX_BIG_MOTOR);
                _dof->i_cont = (float)(I_CONT_BIG_MOTOR);
            }
            else
            {
//                _dof->tau_per_amp = torque_sign * (float)(T_PER_AMP_SMALL_MOTOR  * GEAR_BOX_TR_SMALL_MOTOR);  // Amps to torque
                _dof->tau_per_amp = torque_sign * (float)(T_PER_AMP_SMALL_MOTOR  * GEAR_BOX_TR_SMALL_MOTOR);  // Amps to torque
                _dof->DAC_per_amp = (float)(K_DAC_PER_AMP_LOW_CURRENT);                         // DAC counts to AMPS
                _dof->i_max = (float)(I_MAX_SMALL_MOTOR);
                _dof->i_cont = (float)(I_CONT_SMALL_MOTOR);
            }

            //Set encoder offset
            _joint->enc_offset = _joint->enc_val;

#ifdef USE_NEW_DEVICE
            int joint_ind = jointTypeFromCombinedType(_joint->type);
            if (joint_ind != 3) {
            	if (joint_ind > 3) joint_ind--;
            	//printf("Beginning update j %i init.cpp\n",joint_ind);
            	//Device::beginUpdate(Device::currentNoClone()->timestamp());
            	Device::beginCurrentUpdate(ros::Time(0));
            	//printf("Update begun     j %i init.cpp\n",joint_ind);
            	ArmPtr arm = Device::currentNoClone()->getArmById(device0->mech[i].type);
            	MotorPtr motor = arm->motor(joint_ind);
            	motor->setEncoderOffset(_joint->enc_val);
            	//printf("Finishing update j %i init.cpp\n",joint_ind);
            	Device::finishCurrentUpdate();
            	//printf("Update finished  j %i init.cpp\n",joint_ind);
            }
#endif

            // Set DOF state ready to go.
            _joint->state = jstate_not_ready;
        }

        /// Set the encoder offset from Kinematic Zero
        //   Note: enc_offset initialized first above
        if ( device0->mech[i].type == GOLD_ARM)
        {
            device0->mech[i].joint[SHOULDER].enc_offset += SHOULDER_GOLD_KIN_OFFSET * ENC_CNT_PER_DEG * DOF_types[SHOULDER_GOLD].TR; // Degrees * enc/degree *
            device0->mech[i].joint[ELBOW].enc_offset    += ELBOW_GOLD_KIN_OFFSET    * ENC_CNT_PER_DEG * DOF_types[ELBOW_GOLD].TR;
            device0->mech[i].joint[Z_INS].enc_offset    += Z_INS_GOLD_KIN_OFFSET    * DOF_types[Z_INS_GOLD].TR * ENC_CNT_PER_RAD;  // use enc/rad because conversion from meters to revolutions is in radians
        }
        else if (device0->mech[i].type == GREEN_ARM)
        {
            device0->mech[i].joint[SHOULDER].enc_offset += SHOULDER_GREEN_KIN_OFFSET * ENC_CNT_PER_DEG * DOF_types[SHOULDER_GREEN].TR;
            device0->mech[i].joint[ELBOW].enc_offset    += ELBOW_GREEN_KIN_OFFSET    * ENC_CNT_PER_DEG * DOF_types[ELBOW_GREEN].TR;
            device0->mech[i].joint[Z_INS].enc_offset    += Z_INS_GREEN_KIN_OFFSET    * DOF_types[Z_INS_GREEN].TR * ENC_CNT_PER_RAD;  // use enc/rad because conversion from meters to revolutions is in radians
        }

        /// Initialize some more mechanism stuff
        device0->mech[i].pos_d.x = 0;
        device0->mech[i].pos_d.y = 0;
        device0->mech[i].pos_d.z = 0;

        device0->mech[i].ori_d.yaw = 0;
        device0->mech[i].ori_d.pitch = 0;
        device0->mech[i].ori_d.roll = 0;

		device0->mech[i].tool_type = TOOL_GRASPER_10MM;
    }
    
    
    loadOffsets(*device0);

    dofs_inited=1;
}

/**
 * init_ravengains( ros::NodeHandle n)
 *
 *  Get ravengains from ROS parameter server.
 *
 *  WARNING:
 *      The order of gains in the parameter is very important.
 *      Make sure that numerical order of parameters matches the numerical order of the dof types.
 *
 *  postcondition: dof_types[].kp and dof_types[].kd have been set from ROS parameters
 */
int init_ravengains(ros::NodeHandle n, struct device *device0)
{
    XmlRpc::XmlRpcValue kp_green, kp_gold, kd_green, kd_gold, ki_green, ki_gold;
    bool res=0;
    ROS_INFO("Getting gains params...");

    // initialize all gains to zero
    for (int i = 0; i < MAX_MECH * MAX_DOF_PER_MECH; i++)
    {
        DOF_types[i].KP = DOF_types[i].KD = DOF_types[i].KI  = 0.0;
    }

    /// Get gains from parameter server, use default value of 0.0 if parameters ain't found
    if (n.hasParam("/gains_gold_kp") &&
            n.hasParam("/gains_gold_kd") &&
            n.hasParam("/gains_gold_ki") &&
            n.hasParam("/gains_green_kp") &&
            n.hasParam("/gains_green_kd") &&
            n.hasParam("/gains_green_ki") )
    {
        res =  n.getParam("/gains_green_kp", kp_green);
        res &= n.getParam("/gains_green_kd", kd_green);
        res &= n.getParam("/gains_green_ki", ki_green);
        res &= n.getParam("/gains_gold_kp", kp_gold);
        res &= n.getParam("/gains_gold_kd", kd_gold);
        res &= n.getParam("/gains_gold_ki", ki_gold);
    }

    // Did we get the gains??
    if ( !res ||
            (kp_green.size() != MAX_DOF_PER_MECH) ||
            (kd_green.size() != MAX_DOF_PER_MECH) ||
            (ki_green.size() != MAX_DOF_PER_MECH) ||
            (kp_gold.size()  != MAX_DOF_PER_MECH) ||
            (kd_gold.size()  != MAX_DOF_PER_MECH) ||
            (ki_gold.size()  != MAX_DOF_PER_MECH) )
    {
        ROS_ERROR("Gains parameters failed.  Setting zero gains");
    }
    else
    {
        bool initgold=0, initgreen=0;
        for (int i = 0; i < NUM_MECH; i++)
        {
            for (int j = 0; j < MAX_DOF_PER_MECH; j++)
            {
                int dofindex =i*MAX_DOF_PER_MECH + j;

                // Set gains for gold and green arms
                if ( device0->mech[i].type == GOLD_ARM)
                {
                    initgold=true;
                    DOF_types[dofindex].KP = (double)kp_gold[j];   // Cast XMLRPC value to a double and set gain
                    DOF_types[dofindex].KD = (double)kd_gold[j];   //   ""
                    DOF_types[dofindex].KI = (double)ki_gold[j];   //   ""
                }
                else if ( device0->mech[i].type == GREEN_ARM)
                {
                    initgreen=true;
                    DOF_types[dofindex].KP = (double)kp_green[j];  //   ""
                    DOF_types[dofindex].KD = (double)kd_green[j];  //   ""
                    DOF_types[dofindex].KI = (double)ki_green[j];  //   ""
                }
                else
                {
                    ROS_ERROR("What device is this?? %d\n",device0->mech[i].type);
                }
            }
        }
        if (!initgold){
            ROS_ERROR("Failed to set gains for gold arm (ser:%d not %d).  Set to zero", device0->mech[0].type, GOLD_ARM);
        }
        if (!initgreen){
            ROS_ERROR("Failed to set gains for green arm (ser:%d not %d).  Set to zero", device0->mech[1].type, GREEN_ARM);
        }
        ROS_INFO("  PD gains set to");
        ROS_INFO("    green: %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf",
            DOF_types[0].KP, DOF_types[0].KD, DOF_types[0].KI,
            DOF_types[1].KP, DOF_types[1].KD, DOF_types[1].KI,
            DOF_types[2].KP, DOF_types[2].KD, DOF_types[2].KI,
            DOF_types[3].KP, DOF_types[3].KD, DOF_types[3].KI,
            DOF_types[4].KP, DOF_types[4].KD, DOF_types[4].KI,
            DOF_types[5].KP, DOF_types[5].KD, DOF_types[5].KI,
            DOF_types[6].KP, DOF_types[6].KD, DOF_types[6].KI,
            DOF_types[7].KP, DOF_types[7].KD, DOF_types[7].KI);
        ROS_INFO("    gold: %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf, %.3lf/%.3lf/%.3lf",
            DOF_types[8].KP, DOF_types[8].KD, DOF_types[8].KI,
            DOF_types[9].KP, DOF_types[9].KD, DOF_types[9].KI,
            DOF_types[10].KP, DOF_types[10].KD, DOF_types[10].KI,
            DOF_types[11].KP, DOF_types[11].KD, DOF_types[11].KI,
            DOF_types[12].KP, DOF_types[12].KD, DOF_types[12].KI,
            DOF_types[13].KP, DOF_types[13].KD, DOF_types[13].KI,
            DOF_types[14].KP, DOF_types[14].KD, DOF_types[14].KI,
            DOF_types[15].KP, DOF_types[15].KD, DOF_types[15].KI);
    }
    return 0;
}

/**
 * setStartXYZ() - set the starting xyz coordinate (pos_d = pos)
 *
 * \param device0 - pointer to device
 *
 */
void setStartXYZ(struct device *device0)
{
    int i;
    static int j;
    j++;

    //Get the forward kinematics of this position
    fwdCableCoupling(device0, RL_INIT);
    fwdKin(device0, RL_INIT);

    //Set XYZ offsets
    for (i = 0; i < NUM_MECH; i++)
    {
#ifdef USE_NEW_DEVICE
    	ArmPtr arm = Device::currentNoClone()->getArmById(device0->mech[i].type);
    	btTransform tf = toBt(device0->mech[i].pos,device0->mech[i].ori);
    	ControlInput::getOldControlInput()->armById(device0->mech[i].type).pose() = tf;
    	ControlInput::getOldControlInput()->armById(device0->mech[i].type).grasp() = arm->joint(Joint::Type::GRASP_)->position();
#endif

        device0->mech[i].pos_d.x = device0->mech[i].pos.x;
        device0->mech[i].pos_d.y = device0->mech[i].pos.y;
        device0->mech[i].pos_d.z = device0->mech[i].pos.z;

        device0->mech[i].ori_d.yaw = device0->mech[i].ori.yaw;
        device0->mech[i].ori_d.pitch = device0->mech[i].ori.pitch;
        device0->mech[i].ori_d.roll = device0->mech[i].ori.roll;
        device0->mech[i].ori_d.grasp = device0->mech[i].ori.grasp;

        for (int j=0;j<3;j++)
            for (int k=0;k<3;k++)
                device0->mech[i].ori_d.R[j][k] = device0->mech[i].ori.R[j][k];

    }

    // Update the origin, to which master-side deltas are added.
    updateMasterRelativeOrigin( device0 );
}






/*

    int i=0, j=0;
    struct mechanism* _mech=NULL;
    struct DOF* _joint=NULL;
    float amps_on_wait = 0.9; // 900ms
    float bump_encoder_wait = amps_on_wait + 0.05; // 50ms
    static int startflag=0;

    static ros::Time t1 = t1.now();
    static ros::Time t2 = t2.now();
    static ros::Duration d;

   case 0:     // Automatically "bump" encoders
    {
        // run once to initialize initialization *sigh*
        if (!startflag)
        {
            log_msg("  Starting Initialization");
            startflag=1;
            t1=t1.now();
        }

        t2 = t2.now();
        d = t2-t1;

        // wait for motor amplifiers to turn on
        if (d.toSec() < amps_on_wait)
        {
            _mech=NULL;
            _joint=NULL;
            while(loop_over_joints(device0, _mech, _joint, i, j))
                _joint->current_cmd = 0;
        }

        // apply displacement current for short time
        else if (d.toSec() < bump_encoder_wait)
        {
            log_msg("bump");
            // apply first one direction, then the other.
            int sign = d.toSec() < ((bump_encoder_wait - amps_on_wait)/2) ? 1:-1;

            _mech=NULL;
            _joint=NULL;
            while(loop_over_joints(device0, _mech, _joint, i, j))
            {
                if (is_toolDOF(_joint->type))
                    _joint->current_cmd = sign * TOOL_ROT_MAX_DAC;
                else
                    _joint->current_cmd = sign * -1499;
            }
        }
        // finished bumping encoders.  Continue initialization
        else
        {
            log_msg("    Encoders bumped.");
            _mech=NULL;
            _joint=NULL;
            while(loop_over_joints(device0, _mech, _joint, i, j))
                _joint->current_cmd = 0;

            usb_reset_encoders(GOLD_ARM_SERIAL);
            usb_reset_encoders(GREEN_ARM_SERIAL);
            init_wait_loop=0;
            currParams->sublevel = 1;
            startflag=0;
        }
        break;
    }
 */
