/*
 * ros_io.cpp
 *
 *  Created on: Aug 6, 2012
 *      Author: benk
 */

#include "ros_io.h"

#include <string.h>
#include <pthread.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <iostream>

#include <raven/state/runlevel.h>
#include <raven/state/device.h>

#include "log.h"
#include "utils.h"
#include "mapping.h"
#include <raven/kinematics/kinematics_defines.h>
#include "itp_teleoperation.h"
#include "shared_modes.h"
#include "trajectory.h"

#include <tf/transform_datatypes.h>
#include <raven_2_control/raven_state.h>
#include <raven_2_control/torque_command.h>
#include <raven_2_control/joint_command.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <raven_2_msgs/Constants.h>
#include <raven_2_msgs/RavenCommand.h>
#include <raven_2_msgs/RavenState.h>
#include <raven_2_msgs/RavenArrayState.h>
#include <raven_2_msgs/RavenTrajectoryCommand.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <raven/util/stringify.h>

extern int NUM_MECH;
extern USBStruct USBBoards;
extern unsigned long int gTime;

extern btVector3 master_raw_position[2];
extern btVector3 master_position[2];
extern btMatrix3x3 master_raw_orientation[2];
extern btMatrix3x3 master_orientation[2];

static int _localio_counter;
static const int PRINT_EVERY_PEDAL_UP   = 1000000000;
static const int PRINT_EVERY_PEDAL_DOWN = 1000;
static int PRINT_EVERY = PRINT_EVERY_PEDAL_DOWN;

struct robot_device* device0ptr;

using namespace raven_2_control;
// Global publisher for raven data
ros::Publisher pub_raven_state;
ros::Publisher pub_raven_array_state;
ros::Publisher joint_publisher;

ros::Publisher pub_tool_pose_array;
ros::Publisher pub_tool_pose[2];

ros::Publisher pub_tool_pose_command_array;
ros::Publisher pub_tool_pose_command[2];

ros::Publisher pub_master_pose[2];
ros::Publisher pub_master_pose_raw[2];

ros::Publisher vis_pub1;
ros::Publisher vis_pub2;
ros::Publisher pub_ravenstate_old;

ros::Subscriber cmd_sub;
ros::Subscriber cmd_traj_sub;
ros::Subscriber cmd_pose_sub[2];
ros::Subscriber torque_sub1;
ros::Subscriber torque_sub2;
ros::Subscriber joint_sub;

tf::TransformListener* tf_listener;

void publish_joints(struct robot_device*);
void publish_command_pose(struct robot_device*);
void publish_tool_pose(struct robot_device*);
void publish_master_pose(struct robot_device*);
void publish_marker(struct robot_device*);

#define APPEND_TOPIC(base,topic) (std::string(base) + "/" + std::string(topic))

#define RAVEN_STATE_TOPIC "raven_state"
#define RAVEN_ARRAY_STATE_TOPIC APPEND_TOPIC(RAVEN_STATE_TOPIC,"array")

#define TOOL_POSE_TOPIC "tool_pose"
#define TOOL_POSE_SIDE_TOPIC(side) APPEND_TOPIC(TOOL_POSE_TOPIC,side)

#define TOOL_POSE_COMMAND_TOPIC APPEND_TOPIC(TOOL_POSE_TOPIC,"command")
#define TOOL_POSE_COMMAND_SIDE_TOPIC(side) APPEND_TOPIC(TOOL_POSE_COMMAND_TOPIC,side)

#define MASTER_POSE_TOPIC "master_pose"
#define MASTER_POSE_SIDE_TOPIC(side) APPEND_TOPIC(MASTER_POSE_TOPIC,side)

#define MASTER_POSE_RAW_TOPIC APPEND_TOPIC(MASTER_POSE_TOPIC,"raw")
#define MASTER_POSE_RAW_SIDE_TOPIC(side) APPEND_TOPIC(MASTER_POSE_RAW_TOPIC,side)

#define RAVEN_COMMAND_TOPIC "raven_command"
#define RAVEN_COMMAND_TRAJECTORY_TOPIC APPEND_TOPIC(RAVEN_COMMAND_TOPIC,"trajectory")
#define RAVEN_COMMAND_POSE_TOPIC(side) APPEND_TOPIC(RAVEN_COMMAND_TOPIC,side)

bool checkRate(ros::Time& last_pub,ros::Duration interval,ros::Duration& since_last_pub) {
	ros::Time now = ros::Time::now();
	since_last_pub = (now-last_pub);
	if (last_pub.isZero()) {
		last_pub = now;
		return false;
	} else if (since_last_pub > interval) {
		last_pub = now;
		return true;
	} else {
		return false;
	}
}

inline geometry_msgs::Pose toRos(position pos,orientation ori,btMatrix3x3 transform=btMatrix3x3::getIdentity()) {
	btQuaternion rot;
	(toBt(ori.R) * transform).getRotation(rot);
	geometry_msgs::Point p;
	p.x = ((float)pos.x) / MICRON_PER_M;
	p.y = ((float)pos.y) / MICRON_PER_M;
	p.z = ((float)pos.z) / MICRON_PER_M;
	geometry_msgs::Quaternion q;
	rot.normalize();
	q.x = rot.x();
	q.y = rot.y();
	q.z = rot.z();
	q.w = rot.w();
	geometry_msgs::Pose pose;
	pose.position = p;
	pose.orientation  = q;
	return pose;
}

float rosGraspFromMech(int armId,int grasp) {
	if (armId == GOLD_ARM_ID) {
		return ((float)grasp) / 1000.;
	} else {
		return -((float)grasp) / 1000.;
	}
}

std::string rosJointName(int jointType) {
	switch(jointType) {
	case SHOULDER: return "shoulder";
	case ELBOW: return "elbow";
	case Z_INS: return "insertion";
	case TOOL_ROT: return "tool_roll";
	case WRIST: return "wrist_joint";
	case GRASP1: return "grasper_joint_1";
	case GRASP2: return "grasper_joint_2";
	default: return "UNKNOWN";
	}
}

std::string rosArmName(int armId) {
	switch(armId) {
	case GOLD_ARM_ID: return "L";
	case GREEN_ARM_ID: return "R";
	default: return "UNKNOWN";
	}
}

void processRavenCmd(const raven_2_msgs::RavenCommand& cmd);
void processCartesianSpaceControl(const raven_2_msgs::RavenCommand& cmd,param_pass& params);

void cmd_callback(const raven_2_msgs::RavenCommand& cmd) {
	if (!checkMasterMode(RAVEN_COMMAND_TOPIC)) { return; }
	processRavenCmd(cmd);
}

void cmd_pose_callback(const geometry_msgs::PoseStampedConstPtr& pose,int armId) {
	//printf("cmd pose callback\n");
	if (!checkMasterMode(RAVEN_COMMAND_POSE_TOPIC(armNameFromId(armId)))) { return; }

	raven_2_msgs::RavenCommand cmd;
	cmd.header = pose->header;
	cmd.pedal_down = true;
	//FIXME: needs to work
	//cmd.controller = raven_2_msgs::Constants::CONTROLLER_END_EFFECTOR;
	cmd.controller = raven_2_msgs::Constants::CONTROLLER_CARTESIAN_SPACE;
	raven_2_msgs::ArmCommand arm_cmd;
	arm_cmd.active = true;
	arm_cmd.tool_command.pose_option = raven_2_msgs::ToolCommand::POSE_ABSOLUTE;
	arm_cmd.tool_command.pose = pose->pose;

	arm_cmd.tool_command.grasp_option = raven_2_msgs::ToolCommand::GRASP_OFF;

	arm_cmd.tool_command.grasp = 0;
	cmd.arm_names.push_back(armNameFromId(armId));
	cmd.arms.push_back(arm_cmd);
	//cmd.arms[armId] = arm_cmd;
	processRavenCmd(cmd);
}

void processRavenCmd(const raven_2_msgs::RavenCommand& cmd1) {
	raven_2_msgs::RavenCommand cmd = cmd1;
    if (cmd.controller == raven_2_msgs::Constants::CONTROLLER_NONE) {
    	t_controlmode currentMode = getControlMode();
    	if (currentMode == no_control) {
    		//default to cartesian space control
    		cmd.controller = raven_2_msgs::Constants::CONTROLLER_CARTESIAN_SPACE;
    	} else {
    		cmd.controller = (uint8_t) currentMode;
    	}
    } else if (!checkControlMode((t_controlmode)cmd.controller)) {
    	return;
    }

	_localio_counter++;

	//printf("cmd callback!\n");

	param_pass params;
	getRcvdParams(&params);

	if (!cmd.pedal_down) {
		if (_localio_counter % PRINT_EVERY == 0) { printf("inactive!\n"); }
		params.surgeon_mode =  SURGEON_DISENGAGED;
	} else {
		params.surgeon_mode = SURGEON_ENGAGED;
	}

	//RunLevel::setPedal(cmd.pedal_down);
	for (size_t i=0;i<cmd.arms.size();i++) {
		Arm::IdType armId = Device::getArmIdByName(cmd.arm_names[i]);
		RunLevel::setArmActive(armId,cmd.arms[i].active);
	}

	if (cmd.pedal_down) {
		switch (cmd.controller) {
		case raven_2_msgs::Constants::CONTROLLER_END_EFFECTOR:
		case raven_2_msgs::Constants::CONTROLLER_CARTESIAN_SPACE:
			processCartesianSpaceControl(cmd,params);
			break;
		default:
			log_err("Unknown control mode %s",controlModeToString((t_controlmode) cmd.controller).c_str());
			break;
		}
	}
	//isUpdated = TRUE;
	writeUpdate(&params);
}

void processCartesianSpaceControl(const raven_2_msgs::RavenCommand& cmd,param_pass& params) {
	static ros::Time last_call(0);

    ros::Time now = ros::Time::now();
    if (last_call.toSec() == 0) { last_call = now; return; }
    ros::Duration since_last_call = (now-last_call);
    last_call = now;

    float max_interval = 0.5;
    if (since_last_call.toSec() > max_interval) {
    	since_last_call = ros::Duration(max_interval);
    }

	for (int mech_ind=0;mech_ind<NUM_MECH;mech_ind++) {
		int arm_serial = USBBoards.boards[mech_ind];
		int arm_id = armIdFromSerial(arm_serial);

		raven_2_msgs::ArmCommand armCmd;
		bool found = false;
		for (int i=0;i<cmd.arm_names.size();i++) {
			if (cmd.arm_names.at(i) == armNameFromId(arm_id)) {
				armCmd = cmd.arms.at(i);
				found = true;
				break;
			}
		}
		if (!found || !armCmd.active) {
			continue;
		}

		if (_localio_counter % PRINT_EVERY == 0 || true) {
			if (arm_id == 0) {
				//printf("*********active %d!-----------------------------------\n",i);
			} else {
				//printf("*********active %d!XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n",i);
			}
		}

		tf::Transform tool_pose_raw;
		tf::poseMsgToTF(armCmd.tool_command.pose,tool_pose_raw);

		tf::StampedTransform to0link;
		try {
			tf_listener->lookupTransform("0_link",cmd.header.frame_id,ros::Time(0),to0link);
		} catch (tf::TransformException& ex) {
			ROS_ERROR("%s",ex.what());
			continue;
		}

		btTransform tool_pose;

		tf::Transform tool_pose_tf = to0link * tool_pose_raw;

		tf::Quaternion tf_q = tool_pose_tf.getRotation();
		btQuaternion tool_pose_q(tf_q.x(),tf_q.y(),tf_q.z(),tf_q.w());

		tf::Vector3 tf_p = tool_pose_tf.getOrigin();
		btVector3 tool_pose_p(tf_p.x(),tf_p.y(),tf_p.z());
		tool_pose = btTransform(tool_pose_q,tool_pose_p);

		btVector3 p = tool_pose.getOrigin();

		uint8_t pose_option = armCmd.tool_command.pose_option;
		if (pose_option & raven_2_msgs::ToolCommand::POSE_OPTION_FLAG_POSITION_ENABLED) {
			if (pose_option & raven_2_msgs::ToolCommand::POSE_OPTION_FLAG_POSITION_RELATIVE) {
				master_raw_position[arm_id] += p;
			} else {
				master_raw_position[arm_id] = p;
			}
		}

		btQuaternion rot = tool_pose.getRotation();

		master_raw_orientation[arm_id].setRotation(rot);

		btMatrix3x3 rot_mx_temp(rot);

		if (pose_option & raven_2_msgs::ToolCommand::POSE_OPTION_FLAG_POSITION_ENABLED) {
			if (pose_option & raven_2_msgs::ToolCommand::POSE_OPTION_FLAG_POSITION_RELATIVE) {
				params.xd[mech_ind].x += p.x() * MICRON_PER_M;
				params.xd[mech_ind].y += p.y() * MICRON_PER_M;
				params.xd[mech_ind].z += p.z() * MICRON_PER_M;

				master_position[arm_id] += p;
			} else {
				params.xd[mech_ind].x = p.x() * MICRON_PER_M;
				params.xd[mech_ind].y = p.y() * MICRON_PER_M;
				params.xd[mech_ind].z = p.z() * MICRON_PER_M;

				master_position[arm_id] = p;

			}
		} else {

		}

		if (pose_option & raven_2_msgs::ToolCommand::POSE_OPTION_FLAG_ORIENTATION_ENABLED) {
			if (pose_option & raven_2_msgs::ToolCommand::POSE_OPTION_FLAG_ORIENTATION_RELATIVE) {
				ROS_ERROR_THROTTLE(0.1,"Relative orientation not implemented!");
			} else {
				rot_mx_temp = rot_mx_temp * TOOL_POSE_AXES_TRANSFORM.getBasis().inverse();
				master_orientation[arm_id] = rot_mx_temp;
			}
		}

		const int graspmax = (M_PI/2 * 1000);
		const int graspmin = (-20.0 * 1000.0 DEG2RAD);
		if (armCmd.tool_command.grasp_option == raven_2_msgs::ToolCommand::GRASP_SET) {
			if (arm_serial == GOLD_ARM_SERIAL) {
				params.rd[mech_ind].grasp = saturate(1000*armCmd.tool_command.grasp,graspmin,graspmax);
			} else {
				params.rd[mech_ind].grasp = saturate(-1000*armCmd.tool_command.grasp,-graspmax,-graspmin);
			}
		} else if (armCmd.tool_command.grasp_option == raven_2_msgs::ToolCommand::GRASP_INCREMENT) {
			if (arm_serial == GOLD_ARM_SERIAL) {
				params.rd[mech_ind].grasp = saturate(params.rd[mech_ind].grasp + 1000*armCmd.tool_command.grasp,graspmin,graspmax);
			} else {
				params.rd[mech_ind].grasp = saturate(params.rd[mech_ind].grasp - 1000*armCmd.tool_command.grasp,-graspmax,-graspmin);
			}
		} else if (armCmd.tool_command.grasp_option == raven_2_msgs::ToolCommand::GRASP_INCREMENT_SIGN) {
			float grasp_scale_factor = 500 * since_last_call.toSec();
			if (grasp_scale_factor < 1) {
				grasp_scale_factor = 1;
			}
			if (arm_serial == GOLD_ARM_SERIAL) {
				params.rd[mech_ind].grasp = saturate(params.rd[mech_ind].grasp + grasp_scale_factor*armCmd.tool_command.grasp,graspmin,graspmax);
			} else {
				params.rd[mech_ind].grasp = saturate(params.rd[mech_ind].grasp - grasp_scale_factor*armCmd.tool_command.grasp,-graspmax,-graspmin);
			}
		}

		if (cmd.controller == raven_2_msgs::Constants::CONTROLLER_CARTESIAN_SPACE) {
			float insertion_scale = (Z_INS_MAX_LIMIT - Z_INS_MIN_LIMIT) / 10.;
			int mechId = -1;
			for (int m=0;m<NUM_MECH;m++) {
				if (armIdFromMechType(device0ptr->mech[m].type) == armIdFromSerial(arm_serial)) {
					mechId = m;
					break;
				}
			}

			for (size_t i=0;i<armCmd.joint_types.size();i++) {
				if (armCmd.joint_types[i] == raven_2_msgs::Constants::JOINT_TYPE_INSERTION
						&& armCmd.joint_commands[i].command_type == raven_2_msgs::JointCommand::COMMAND_TYPE_VELOCITY) {
					float ins_amt = since_last_call.toSec() * insertion_scale * armCmd.joint_commands[i].value;
					btTransform ins_tf = actual_world_to_ik_world(arm_id)
											* Tw2b
											* Zs(THS_TO_IK(arm_id,device0ptr->mech[mechId].joint[SHOULDER].jpos))
					* Xu
					* Ze(THE_TO_IK(arm_id,device0ptr->mech[mechId].joint[ELBOW].jpos))
					* Xf
					* Zr(THR_TO_IK(arm_id,device0ptr->mech[mechId].joint[TOOL_ROT].jpos))
					* Zi(D_TO_IK(arm_id,device0ptr->mech[mechId].joint[Z_INS].jpos));

					ins_tf.setOrigin(btVector3(0,0,0)); //only need rotation

					btVector3 z_ins_in_world = ins_amt * (ins_tf * btVector3(0,0,1));

					params.xd[mech_ind].x += z_ins_in_world.x() * MICRON_PER_M;
					params.xd[mech_ind].y += z_ins_in_world.y() * MICRON_PER_M;
					params.xd[mech_ind].z += z_ins_in_world.z() * MICRON_PER_M;
				}
			}
		}



		for (int j=0;j<3;j++) {
			for (int k=0;k<3;k++) {
				params.rd[mech_ind].R[j][k] = rot_mx_temp[j][k];
			}
		}
	}
}

void cmd_trajectory_callback(const raven_2_msgs::RavenTrajectoryCommand& traj_msg) {
	log_msg("trajectory callback!\n");
	if (!checkMasterMode(RAVEN_COMMAND_TRAJECTORY_TOPIC)) { return; }
	if (!checkControlMode(trajectory_control)) { return; }

	t_controlmode controlmode = (t_controlmode)traj_msg.controller;
	if (traj_msg.controller == raven_2_msgs::Constants::CONTROLLER_NONE) {
		t_controlmode currentMode = getControlMode();
		if (currentMode == no_control) {
			//default to cartesian space control
			controlmode = (t_controlmode)raven_2_msgs::Constants::CONTROLLER_CARTESIAN_SPACE;
		} else {
			controlmode = currentMode;
		}
	}

	log_msg("Got trajectory with control %s",controlModeToString(controlmode).c_str());

	param_pass_trajectory traj;
	if (true || traj_msg.header.stamp.isZero()) {
		ros::Time now = ros::Time::now();
		traj.begin_time = now.toSec();
	} else {
		traj.begin_time = traj_msg.header.stamp.toSec();
	}

	traj.total_duration = 0;
	traj.control_mode = controlmode;

	param_pass params;
	getRcvdParams(&params);
	params.surgeon_mode = SURGEON_ENGAGED;
	for (size_t i=0;i<traj_msg.commands.size();i++) {
		param_pass_trajectory_pt pt;
		pt.time_from_start = traj_msg.commands[i].time_from_start.toSec();

		raven_2_msgs::RavenCommand cmd;
		cmd.header.stamp = ros::Time(traj.begin_time + pt.time_from_start);
		cmd.header.frame_id = traj_msg.header.frame_id;
		cmd.controller = controlmode;
		cmd.pedal_down = true;
		cmd.arm_names = traj_msg.commands[i].arm_names;
		cmd.arms = traj_msg.commands[i].arms;

		params.surgeon_mode = cmd.pedal_down;

		switch (cmd.controller) {
		case raven_2_msgs::Constants::CONTROLLER_END_EFFECTOR:
		case raven_2_msgs::Constants::CONTROLLER_CARTESIAN_SPACE:
			processCartesianSpaceControl(cmd,params);
			break;
		default:
			log_err("Unknown control mode for trajectory %s",controlModeToString((t_controlmode) cmd.controller).c_str());
			break;
		}
		pt.param = params;

		printf("(%i,%i,%i)\n",pt.param.xd[0].x,pt.param.xd[0].y,pt.param.xd[0].z);

		traj.pts.push_back(pt);
	}

	traj.total_duration = traj.pts.back().time_from_start + 1;

	params = traj.pts[0].param;
	writeUpdate(&params);

	setTrajectory(traj);
}

void publish_ravenstate_old(struct robot_device *device0,u_08 runlevel,u_08 sublevel,ros::Duration since_last_pub);

void publish_ros(struct robot_device *device0,param_pass currParams) {
	u_08 runlevel;
	u_08 sublevel;
#ifdef USE_NEW_RUNLEVEL
	RunLevel currRunlevel = RunLevel::get();
	currRunlevel.getNumbers<u_08>(runlevel,sublevel);
#else
	runlevel = currParams.runlevel;
	sublevel = currParams.sublevel;
#endif
	static bool hasHomed = false;
	static raven_2_msgs::RavenState raven_state;
	raven_state.arms.clear();

	if (
#ifdef USE_NEW_RUNLEVEL
			RunLevel::isInitialized()
#else
			runlevel == RL_PEDAL_DN || runlevel == RL_PEDAL_UP
#endif
			) {
		hasHomed = true;
	}

	static ros::Time last_pub;
	static ros::Duration interval(0.01);
	static ros::Duration since_last_pub;
	if (!checkRate(last_pub,interval,since_last_pub)) { return; }

	publish_master_pose(device0);

	//Only publish these if we've homed already
	if (true || hasHomed) {
		publish_joints(device0);
		//publish_marker(device0);
		if (
#ifdef USE_NEW_RUNLEVEL
				!currRunlevel.isEstop()
#else
				runlevel != RL_E_STOP
#endif
				) {
			publish_command_pose(device0);
		}
		publish_tool_pose(device0);
	}

	publish_ravenstate_old(device0,runlevel,sublevel,since_last_pub);

	//raven_state

#ifdef USE_NEW_DEVICE
	raven_state.header.stamp = Device::currentTimestamp();
#else
	raven_state.header.stamp = ros::Time::now();
#endif
	raven_state.header.frame_id = "/0_link";
#ifdef USE_NEW_RUNLEVEL
	RunLevel rl = RunLevel::get();
	rl.getNumbers<uint8_t>(raven_state.runlevel,raven_state.sublevel);
	raven_state.pedal_down = RunLevel::getPedal();
#else
	raven_state.runlevel = runlevel;
	raven_state.sublevel = sublevel;
	raven_state.pedal_down = device0->surgeon_mode;
#endif

	raven_state.master = getMasterModeString();

	//t_controlmode controlMode = (t_controlmode)currParams.robotControlMode;
	t_controlmode controlMode = getControlMode();
	raven_state.controller = (uint8_t) controlMode;

	mechanism* _mech=NULL;
	DOF* _joint=NULL;
	int mechnum, jnum;

	while (loop_over_mechs(device0,_mech,mechnum)) {
		raven_2_msgs::ArmState arm_state;
		uint8_t arm_type;
		btMatrix3x3 transform;
		switch (_mech->type) {
		case GOLD_ARM:
			arm_type = raven_2_msgs::Constants::ARM_TYPE_GOLD;
			transform = btMatrix3x3(1,0,0,  0,-1,0,  0,0,-1);
			break;
		case GREEN_ARM:
			arm_type = raven_2_msgs::Constants::ARM_TYPE_GREEN;
			transform = btMatrix3x3(1,0,0,  0,-1,0,  0,0,-1);
			break;
		default:
			ROS_ERROR_STREAM("Unknown mech type"<<_mech->type);
			continue;
		}
		arm_state.name = armNameFromMechType(_mech->type);
		arm_state.type = arm_type;

		arm_state.base_pose = toRos(_mech->base_pos,_mech->base_ori,transform);

		switch (_mech->type) {
		case TOOL_NONE: arm_state.tool_type = raven_2_msgs::Constants::TOOL_TYPE_NONE; break;
		case TOOL_GRASPER_10MM: arm_state.tool_type = raven_2_msgs::Constants::TOOL_TYPE_GRASPER_10MM; break;
		case TOOL_GRASPER_8MM: arm_state.tool_type = raven_2_msgs::Constants::TOOL_TYPE_GRASPER_8MM; break;
		}

		arm_state.tool_pose = toRos(_mech->pos,_mech->ori,transform);
		arm_state.tool_pose_desired = toRos(_mech->pos_d,_mech->ori_d,transform);

		arm_state.grasp = rosGraspFromMech(armIdFromMechType(_mech->type),_mech->ori.grasp);
		arm_state.grasp_desired = rosGraspFromMech(armIdFromMechType(_mech->type),_mech->ori_d.grasp);

		_joint = NULL;
		int grasp1_index=-1;
		int grasp2_index=-1;
		while (loop_over_joints(_mech,_joint,jnum)) {
			raven_2_msgs::JointState joint_state;
			int joint_type = jointTypeFromCombinedType(_joint->type);
			switch (joint_type) {
			case SHOULDER: joint_state.type = raven_2_msgs::Constants::JOINT_TYPE_SHOULDER; break;
			case ELBOW: joint_state.type = raven_2_msgs::Constants::JOINT_TYPE_ELBOW; break;
			case Z_INS: joint_state.type = raven_2_msgs::Constants::JOINT_TYPE_INSERTION; break;
			case TOOL_ROT: joint_state.type = raven_2_msgs::Constants::JOINT_TYPE_ROTATION; break;
			case WRIST: joint_state.type = raven_2_msgs::Constants::JOINT_TYPE_PITCH; break;
			case GRASP1: joint_state.type = raven_2_msgs::Constants::JOINT_TYPE_GRASP_FINGER1; grasp1_index = jnum; break;
			case GRASP2: joint_state.type = raven_2_msgs::Constants::JOINT_TYPE_GRASP_FINGER2; grasp2_index = jnum; break;
			case NO_CONNECTION: continue;
			}

			switch (_joint->state) {
			case jstate_not_ready: joint_state.state = raven_2_msgs::JointState::STATE_NOT_READY; break;
			case jstate_pos_unknown: joint_state.state = raven_2_msgs::JointState::STATE_POS_UNKNOWN; break;
			case jstate_homing1: joint_state.state = raven_2_msgs::JointState::STATE_HOMING1; break;
			case jstate_homing2: joint_state.state = raven_2_msgs::JointState::STATE_HOMING2; break;
			case jstate_ready: joint_state.state = raven_2_msgs::JointState::STATE_READY; break;
			case jstate_wait: joint_state.state = raven_2_msgs::JointState::STATE_WAIT; break;
			case jstate_hard_stop: joint_state.state = raven_2_msgs::JointState::STATE_HARD_STOP; break;
			default: joint_state.state = raven_2_msgs::JointState::STATE_LAST_TYPE; break;
			}

			joint_state.encoder_value = _joint->enc_val;
			joint_state.encoder_offset = _joint->enc_offset;
			joint_state.dac_command = _joint->current_cmd;

			joint_state.position = _joint->jpos;
			joint_state.velocity = _joint->jvel;

			joint_state.motor_position = _joint->mpos;
			joint_state.motor_velocity = _joint->mvel;

			joint_state.torque = _joint->tau_d;
			joint_state.gravitation_torque_estimate = _joint->tau_g;

			joint_state.integrated_position_error = _joint->perror_int;

			raven_2_msgs::JointCommand joint_cmd;
			joint_cmd.command_type = raven_2_msgs::JointCommand::COMMAND_TYPE_POSITION;
			joint_cmd.value = _joint->jpos_d;
			/*
	    		joint_cmd.position = _joint->jpos;
	    		joint_cmd.velocity = _joint->jvel;

	    		joint_cmd.motor_position = _joint->mpos;
	    		joint_cmd.motor_velocity = _joint->mvel;

	    		joint_cmd.torque = _joint->tau;
			 */

			joint_state.command = joint_cmd;

			arm_state.joints.push_back(joint_state);
		}

		raven_2_msgs::JointState yaw_state;
		raven_2_msgs::JointState grasp_state;
		yaw_state.type = raven_2_msgs::Constants::JOINT_TYPE_YAW;
		grasp_state.type = raven_2_msgs::Constants::JOINT_TYPE_GRASP;
		jointState grasp_combined_state;
		if (_mech->joint[grasp1_index].state < _mech->joint[grasp2_index].state) {
			grasp_combined_state = _mech->joint[grasp1_index].state;
		} else {
			grasp_combined_state = _mech->joint[grasp2_index].state;
		}

		switch (grasp_combined_state) {
		case jstate_not_ready: yaw_state.state = raven_2_msgs::JointState::STATE_NOT_READY; break;
		case jstate_pos_unknown: yaw_state.state = raven_2_msgs::JointState::STATE_POS_UNKNOWN; break;
		case jstate_homing1: yaw_state.state = raven_2_msgs::JointState::STATE_HOMING1; break;
		case jstate_homing2: yaw_state.state = raven_2_msgs::JointState::STATE_HOMING2; break;
		case jstate_ready: yaw_state.state = raven_2_msgs::JointState::STATE_READY; break;
		case jstate_wait: yaw_state.state = raven_2_msgs::JointState::STATE_WAIT; break;
		case jstate_hard_stop: yaw_state.state = raven_2_msgs::JointState::STATE_HARD_STOP; break;
		default: yaw_state.state = raven_2_msgs::JointState::STATE_LAST_TYPE; break;
		}
		grasp_state.state = yaw_state.state;

		yaw_state.position = THY_MECH_FROM_FINGERS(armIdFromMechType(_mech->type),_mech->joint[grasp1_index].jpos,_mech->joint[grasp2_index].jpos);
		yaw_state.velocity = THY_MECH_FROM_FINGERS(armIdFromMechType(_mech->type),_mech->joint[grasp1_index].jvel,_mech->joint[grasp2_index].jvel);

		grasp_state.position = rosGraspFromMech(armIdFromMechType(_mech->type),_mech->ori.grasp);

		arm_state.joints.push_back(yaw_state);
		arm_state.joints.push_back(grasp_state);

		raven_state.arms.push_back(arm_state);
	}

	pub_raven_state.publish(raven_state);

	raven_2_msgs::RavenArrayState array_state;
	array_state.header = raven_state.header;
	array_state.runlevel = raven_state.runlevel;
	array_state.sublevel = raven_state.sublevel;
	array_state.pedal_down = raven_state.pedal_down;
	array_state.master = raven_state.master;
	array_state.controller = raven_state.controller;

	for (int i=0;i<(int)raven_state.arms.size();i++) {
		raven_2_msgs::ArmState arm = raven_state.arms[i];
		array_state.arm_names.push_back(arm.name);
		array_state.arm_types.push_back(arm.type);
		array_state.base_poses.push_back(arm.base_pose);
		array_state.tool_types.push_back(arm.tool_type);
		array_state.tool_poses.push_back(arm.tool_pose);
		array_state.tool_poses_desired.push_back(arm.tool_pose_desired);
		array_state.grasps.push_back(arm.grasp);
		array_state.grasps_desired.push_back(arm.grasp_desired);

		for (int j=0;j<(int)arm.joints.size();j++) {
			raven_2_msgs::JointState joint = arm.joints[j];
			array_state.joint_arm_inds.push_back(i);
			array_state.joint_types.push_back(joint.type);
			array_state.joint_states.push_back(joint.state);
			array_state.joint_encoder_values.push_back(joint.encoder_value);
			array_state.joint_encoder_offsets.push_back(joint.encoder_offset);
			array_state.joint_dac_commands.push_back(joint.dac_command);
			array_state.joint_positions.push_back(joint.position);
			array_state.joint_velocities.push_back(joint.velocity);
			array_state.motor_positions.push_back(joint.motor_position);
			array_state.motor_velocities.push_back(joint.motor_velocity);
			array_state.torques.push_back(joint.torque);
			array_state.gravitation_torque_estimates.push_back(joint.gravitation_torque_estimate);
			array_state.joint_command_types.push_back(joint.command.command_type);
			array_state.joint_commands.push_back(joint.command.value);
			array_state.integrated_position_errors.push_back(joint.integrated_position_error);
		}

		array_state.input_pins.push_back(arm.input_pins);
		array_state.output_pins.push_back(arm.output_pins);
	}

	pub_raven_array_state.publish(array_state);
}

void publish_command_pose(struct robot_device* device0) {
	static ros::Time last_pub;
	static ros::Duration interval(0.1);
	static ros::Duration since_last_pub;
	if (!checkRate(last_pub,interval,since_last_pub)) { return; }

#ifdef USE_NEW_DEVICE
	ros::Time now = Device::currentTimestamp();
#else
	ros::Time now = ros::Time::now();
#endif

	geometry_msgs::PoseArray command_pose_array;
	command_pose_array.header.stamp = now;
	command_pose_array.header.frame_id = "/0_link";

	geometry_msgs::PoseStamped command_pose;
	command_pose.header = command_pose_array.header;

		mechanism* _mech = NULL;
	int mechnum = 0;
	while (loop_over_mechs(device0,_mech,mechnum)) {
		int armId = armIdFromMechType(_mech->type);
		btTransform pose;

		btVector3 pos = btVector3((float)_mech->pos_d.x,(float)_mech->pos_d.y,(float)_mech->pos_d.z)/MICRON_PER_M;
		btQuaternion rot;
		(toBt(_mech->ori_d.R) * TOOL_POSE_AXES_TRANSFORM.getBasis()).getRotation(rot);
		pose = btTransform(rot,pos);

		//pose = ik_world_to_actual_world(armId) * pose * Tg.inverse();

		tf::poseTFToMsg(pose,command_pose.pose);

		command_pose_array.poses.push_back(command_pose.pose);

		pub_tool_pose_command[armId].publish(command_pose);
	}

	pub_tool_pose_command_array.publish(command_pose_array);
}

void publish_tool_pose(struct robot_device* device0) {
	static ros::Time last_pub;
	static ros::Duration interval(0.1);
	static ros::Duration since_last_pub;
	if (!checkRate(last_pub,interval,since_last_pub)) { return; }

#ifdef USE_NEW_DEVICE
	ros::Time now = Device::currentTimestamp();
#else
	ros::Time now = ros::Time::now();
#endif

	geometry_msgs::PoseArray tool_pose_array;
	tool_pose_array.header.stamp = now;
	tool_pose_array.header.frame_id = "/0_link";

	geometry_msgs::PoseStamped tool_pose;
	tool_pose.header = tool_pose_array.header;

	mechanism* _mech = NULL;
	int mechnum = 0;
	while (loop_over_mechs(device0,_mech,mechnum)) {
		int armId = armIdFromMechType(_mech->type);
		btTransform pose;

		btVector3 pos = btVector3((float)_mech->pos.x,(float)_mech->pos.y,(float)_mech->pos.z)/MICRON_PER_M;
		btQuaternion rot;
		toBt(_mech->ori.R).getRotation(rot);
		pose = btTransform(rot,pos) * TOOL_POSE_AXES_TRANSFORM;

		//pose = ik_world_to_actual_world(armId) * pose * Tg.inverse();

		//Use this to check intermediary poses
		/*pose = actual_world_to_ik_world(armId)
				* Tw2b
				* Zs(THS_TO_IK(armId,_mech->joint[SHOULDER].jpos))
				* Xu
				* Ze(THE_TO_IK(armId,_mech->joint[ELBOW].jpos))
				* Xf
				* Zr(THR_TO_IK(armId,_mech->joint[TOOL_ROT].jpos))
				* Zi(D_TO_IK(armId,_mech->joint[Z_INS].jpos))
				* Xip
				* Zp(THP_TO_IK(armId,_mech->joint[WRIST].jpos))
				* Xpy
				* Zy(THY_TO_IK_FROM_FINGERS(armId,_mech->joint[GRASP1].jpos,_mech->joint[GRASP2].jpos))
				* Tg
				* TOOL_POSE_AXES_TRANSFORM;
		*/

		tf::poseTFToMsg(pose,tool_pose.pose);

		tool_pose_array.poses.push_back(tool_pose.pose);

		pub_tool_pose[armId].publish(tool_pose);
	}

	pub_tool_pose_array.publish(tool_pose_array);
}

void publish_master_pose(struct robot_device* device0) {
	static ros::Time last_pub;
	static ros::Duration interval(0.1);
	static ros::Duration since_last_pub;
	if (!checkRate(last_pub,interval,since_last_pub)) { return; }

	btQuaternion q;

#ifdef USE_NEW_DEVICE
	ros::Time now = Device::currentTimestamp();
#else
	ros::Time now = ros::Time::now();
#endif

	geometry_msgs::PoseStamped master_pose;
	master_pose.header.stamp = now;
	master_pose.header.frame_id = "/0_link";

	geometry_msgs::PoseStamped master_pose_raw;
	master_pose_raw.header = master_pose.header;

	mechanism* _mech = NULL;
	int mechnum = 0;
	while (loop_over_mechs(device0,_mech,mechnum)) {
		int armId = armIdFromMechType(_mech->type);

		master_pose.pose.position.x = master_position[armId].x();
		master_pose.pose.position.y = master_position[armId].y();
		master_pose.pose.position.z = master_position[armId].z();

		master_orientation[armId].getRotation(q);

		master_pose.pose.orientation.x = q.x();
		master_pose.pose.orientation.y = q.y();
		master_pose.pose.orientation.z = q.z();
		master_pose.pose.orientation.w = q.w();


		master_pose_raw.pose.position.x = master_raw_position[armId].x();
		master_pose_raw.pose.position.y = master_raw_position[armId].y();
		master_pose_raw.pose.position.z = master_raw_position[armId].z();

		master_raw_orientation[armId].getRotation(q);

		master_pose_raw.pose.orientation.x = q.x();
		master_pose_raw.pose.orientation.y = q.y();
		master_pose_raw.pose.orientation.z = q.z();
		master_pose_raw.pose.orientation.w = q.w();

		pub_master_pose[armId].publish(master_pose);
		pub_master_pose_raw[armId].publish(master_pose_raw);
	}
}

/**
*   publish_joints() - publish joint angles to visualization
*/
void publish_joints(struct robot_device* device0){

    static int count=0;
    static ros::Time last_pub;
    static ros::Duration interval(0.01);
    static ros::Duration since_last_pub;
    if (!checkRate(last_pub,interval,since_last_pub)) { return; }

#ifdef USE_NEW_DEVICE
	ros::Time now = Device::currentTimestamp();
#else
	ros::Time now = ros::Time::now();
#endif

    sensor_msgs::JointState joint_state;
    //update joint_state
    joint_state.header.stamp = now;
//    joint_state.name.resize(32);
//    joint_state.position.resize(32);
//    joint_state.name.resize(14);
//    joint_state.position.resize(14);
    int left, right;
    if (device0->mech[0].type == GOLD_ARM) {
        left = 0;
        right = 1;
    } else {
        left = 1;
        right = 0;
    }

    mechanism* _mech=NULL;
    DOF* _joint=NULL;
    int mechnum, jnum;
    int grasp1_index, grasp2_index;

    grasp1_index=-1;
	grasp2_index=-1;
    while (loop_over_mechs(device0,_mech,mechnum)) {
    	_joint = NULL;
    	std::string armName = rosArmName(armIdFromMechType(_mech->type));
    	while (loop_over_joints(_mech,_joint,jnum)) {
    		joint_state.name.push_back(rosJointName(jointTypeFromCombinedType(_joint->type)) + "_" + armName);
    		joint_state.position.push_back(_joint->jpos);
    		switch (jointTypeFromCombinedType(_joint->type)) {
    		case GRASP1: grasp1_index = jnum; break;
    		case GRASP2: grasp2_index = jnum; break;
    		}
    	}
    	joint_state.name.push_back("grasper_yaw_" + armName);
    	joint_state.position.push_back(THY_MECH_FROM_FINGERS(armIdFromMechType(_mech->type),_mech->joint[grasp1_index].jpos,_mech->joint[grasp2_index].jpos));
    	joint_state.name.push_back("grasper_" + armName);
    	joint_state.position.push_back(rosGraspFromMech(armIdFromMechType(_mech->type),_mech->ori.grasp));
    }

    _mech=NULL;
    _joint=NULL;

    grasp1_index=-1;
    grasp2_index=-1;
    while (loop_over_mechs(device0,_mech,mechnum)) {
    	_joint = NULL;
    	std::string armName = rosArmName(armIdFromMechType(_mech->type));
    	while (loop_over_joints(_mech,_joint,jnum)) {
    		joint_state.name.push_back(rosJointName(jointTypeFromCombinedType(_joint->type)) + "_" + armName + "2");
    		joint_state.position.push_back(_joint->jpos_d);
    		switch (jointTypeFromCombinedType(_joint->type)) {
    		case GRASP1: grasp1_index = jnum; break;
    		case GRASP2: grasp2_index = jnum; break;
    		}
    	}
    	joint_state.name.push_back("grasper_yaw_" + armName + "2");
    	joint_state.position.push_back(THY_MECH_FROM_FINGERS(armIdFromMechType(_mech->type),_mech->joint[grasp1_index].jpos_d,_mech->joint[grasp2_index].jpos_d));
    	joint_state.name.push_back("grasper_" + armName + "2");
    	joint_state.position.push_back(rosGraspFromMech(armIdFromMechType(_mech->type),_mech->ori_d.grasp));
    }

    //Publish the joint states
    joint_publisher.publish(joint_state);

}

int init_pubs(ros::NodeHandle &n,struct robot_device *device0) {
	pub_raven_state = n.advertise<raven_2_msgs::RavenState>(RAVEN_STATE_TOPIC, 1000);
	pub_raven_array_state = n.advertise<raven_2_msgs::RavenArrayState>(RAVEN_ARRAY_STATE_TOPIC, 1000);
	joint_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1);

	mechanism* _mech = NULL;
	int mechnum = 0;
	while (loop_over_mechs(device0,_mech,mechnum)) {
		int armId = armIdFromMechType(_mech->type);
		std::string armName = rosArmName(armId);
		pub_tool_pose_array = n.advertise<geometry_msgs::PoseArray>(TOOL_POSE_TOPIC,1);
		pub_tool_pose[armId] = n.advertise<geometry_msgs::PoseStamped>(TOOL_POSE_SIDE_TOPIC(armName), 1);

		pub_tool_pose_command_array = n.advertise<geometry_msgs::PoseArray>(TOOL_POSE_COMMAND_TOPIC,1);
		pub_tool_pose_command[armId] = n.advertise<geometry_msgs::PoseStamped>( TOOL_POSE_COMMAND_SIDE_TOPIC(armName), 1);

		pub_master_pose[armId] = n.advertise<geometry_msgs::PoseStamped>( MASTER_POSE_SIDE_TOPIC(armName), 1);
		pub_master_pose_raw[armId] = n.advertise<geometry_msgs::PoseStamped>( MASTER_POSE_RAW_SIDE_TOPIC(armName), 1);
	}

	vis_pub1 = n.advertise<visualization_msgs::Marker>( "visualization_marker1", 0 );
	vis_pub2 = n.advertise<visualization_msgs::Marker>( "visualization_marker2", 0 );

	pub_ravenstate_old = n.advertise<raven_state>("ravenstate_old", 1000);

	return 0;
}

void init_subs(ros::NodeHandle &n,struct robot_device *device0) {
	std::cout << "Initializing ros subscribers" << std::endl;
	cmd_sub = n.subscribe(RAVEN_COMMAND_TOPIC, 1, cmd_callback);
	cmd_traj_sub = n.subscribe(RAVEN_COMMAND_TRAJECTORY_TOPIC, 1, cmd_trajectory_callback);
	mechanism* _mech = NULL;
	int mechnum = 0;
	while (loop_over_mechs(device0,_mech,mechnum)) {
		int armId = armIdFromMechType(_mech->type);
		std::string armName = rosArmName(armId);
		cmd_pose_sub[armId] = n.subscribe<geometry_msgs::PoseStamped>(RAVEN_COMMAND_POSE_TOPIC(armName), 1, boost::bind(cmd_pose_callback,_1,armId));
	}

	tf_listener = new tf::TransformListener();
}

void init_ros_topics(ros::NodeHandle &n,struct robot_device *device0) {
	device0ptr = device0;
	init_subs(n,device0);
	init_pubs(n,device0);
}

void torqueCallback1(const torque_command::ConstPtr& torque_cmd) {
	printf("torque callback!\n");
	for (int i=0; i < 8; i++) {
		device0ptr->mech[1].joint[i].tau_d = torque_cmd->torque[i];
		device0ptr->mech[0].joint[i].tau_d = torque_cmd->torque[i+8];
	}
}

void jointCallback(const joint_command::ConstPtr& joint_cmd) {
	printf("joint callback!\n");
	for (int i=0; i < 8; i++) {
		device0ptr->mech[1].joint[i].jpos_d = joint_cmd->angles[i];
		device0ptr->mech[0].joint[i].jpos_d = joint_cmd->angles[i+8];
	}
}


void publish_ravenstate_old(struct robot_device *device0,u_08 runlevel,u_08 sublevel,ros::Duration since_last_pub) {
	static raven_state msg_ravenstate;  // static variables to minimize memory allocation calls
	msg_ravenstate.dt = since_last_pub;

	// Copy the robot state to the output datastructure.
	int numdof=8;
	for (int j=0; j<NUM_MECH; j++){
		msg_ravenstate.type[j]    = device0->mech[j].type;
		msg_ravenstate.pos[j*3]   = device0->mech[j].pos.x;
		msg_ravenstate.pos[j*3+1] = device0->mech[j].pos.y;
		msg_ravenstate.pos[j*3+2] = device0->mech[j].pos.z;
		msg_ravenstate.pos_d[j*3]   = device0->mech[j].pos_d.x;
		msg_ravenstate.pos_d[j*3+1] = device0->mech[j].pos_d.y;
		msg_ravenstate.pos_d[j*3+2] = device0->mech[j].pos_d.z;
		for (int i=0; i<numdof; i++){
			int jtype = device0->mech[j].joint[i].type;
			msg_ravenstate.encVals[jtype]    = device0->mech[j].joint[i].enc_val;
			msg_ravenstate.tau[jtype]        = device0->mech[j].joint[i].tau_d;
			msg_ravenstate.mpos[jtype]       = device0->mech[j].joint[i].mpos RAD2DEG;
			msg_ravenstate.jpos[jtype]       = device0->mech[j].joint[i].jpos RAD2DEG;
			msg_ravenstate.mvel[jtype]       = device0->mech[j].joint[i].mvel RAD2DEG;
			msg_ravenstate.jvel[jtype]       = device0->mech[j].joint[i].jvel RAD2DEG;
			msg_ravenstate.jpos_d[jtype]     = device0->mech[j].joint[i].jpos_d RAD2DEG;
			msg_ravenstate.mpos_d[jtype]     = device0->mech[j].joint[i].mpos_d RAD2DEG;
			msg_ravenstate.encoffsets[jtype] = device0->mech[j].joint[i].enc_offset;
		}
	}
	msg_ravenstate.f_secs = since_last_pub.toSec();
	msg_ravenstate.runlevel=runlevel;
	msg_ravenstate.sublevel=sublevel;

	// Publish the raven data to ROS
	pub_ravenstate_old.publish(msg_ravenstate);
}


void publish_marker(struct robot_device* device0)
{
	static ros::Time last_pub;
	static ros::Duration interval(0.01);
	static ros::Duration since_last_pub;
	if (!checkRate(last_pub,interval,since_last_pub)) { return; }

    visualization_msgs::Marker marker1, marker2;
    geometry_msgs::Point p, px,py,pz;
    btQuaternion bq;
    struct orientation* _ori;
    btMatrix3x3 xform;

    visualization_msgs::Marker axes[3];
    btQuaternion axq, oriq;

    int left,right;

    if (device0->mech[0].type == GOLD_ARM)
    {
        left = 0;
        right = 1;
    }
    else
    {
        left = 1;
        right = 0;
    }


    // setup marker
    marker1.type = visualization_msgs::Marker::SPHERE;
    marker1.action = visualization_msgs::Marker::ADD;   // Set the marker action.  Options are ADD and DELETE
    marker1.header.stamp = ros::Time::now();
    marker1.ns = "RCM_marker";
    marker1.lifetime = ros::Duration();
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker1.scale.x = 0.020;
    marker1.scale.y = 0.020;
    marker1.scale.z = 0.020;


    // DRAW The SPHERE
    int draw_L_sphere=0;
    if (draw_L_sphere)
    {
        marker1.type = visualization_msgs::Marker::SPHERE;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker1.header.frame_id = "/base_link_L";
        marker1.id = 0;
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        p.x = device0->mech[left].pos.x/1e6;
        p.y = device0->mech[left].pos.y/1e6;
        p.z = device0->mech[left].pos.z/1e6;
        marker1.pose.position.x = device0->mech[left].pos.x/1e6;
        marker1.pose.position.y = device0->mech[left].pos.y/1e6;
        marker1.pose.position.z = device0->mech[left].pos.z/1e6;

        // Get quaternion representation of rotation
        _ori = &(device0->mech[left].ori);
        xform.setValue(_ori->R[0][0], _ori->R[0][1], _ori->R[0][2],
                       _ori->R[1][0], _ori->R[1][1], _ori->R[1][2],
                       _ori->R[2][0], _ori->R[2][1], _ori->R[2][2]);
        xform.getRotation(bq);
        marker1.pose.orientation.x = bq.getX();
        marker1.pose.orientation.y = bq.getY();
        marker1.pose.orientation.z = bq.getZ();
        marker1.pose.orientation.w = bq.getW();

        // Set the color -- be sure to set alpha to something non-zero!
        marker1.color.r = 1.0f;
        marker1.color.g = 0.0f;
        marker1.color.b = 0.0f;
        marker1.color.a = 1.0;
        // Publish the marker
        vis_pub1.publish(marker1);
    }


    int draw_L_axes=1;
    if (draw_L_axes)
    {
        for (int i=0;i<3;i++)
        {
            axes[i].type = visualization_msgs::Marker::ARROW;
            axes[i].action = visualization_msgs::Marker::ADD;   // Set the marker action.  Options are ADD and DELETE
            axes[i].header.stamp = ros::Time::now();
            axes[i].ns = "RCM_marker";
            axes[i].lifetime = ros::Duration();
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            axes[i].scale.x = 0.020;
            axes[i].scale.y = 0.020;
            axes[i].scale.z = 0.020;

            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            axes[i].header.frame_id = "/base_link_L";
            axes[i].id = 10+i;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            axes[i].pose.position.x = device0->mech[left].pos.x/1e6;
            axes[i].pose.position.y = device0->mech[left].pos.y/1e6;
            axes[i].pose.position.z = device0->mech[left].pos.z/1e6;

            // Set the color -- be sure to set alpha to something non-zero!
            axes[i].color.r = 0.0f;
            axes[i].color.g = 0.0f;
            axes[i].color.b = 0.0f;
            axes[i].color.a = 1.0;

        }

        // get the transform rotation
        _ori = &(device0->mech[left].ori);
//        xform.setValue(_ori->R[0][0], _ori->R[1][0], _ori->R[2][0],
//                            _ori->R[0][1], _ori->R[1][1], _ori->R[2][1],
//                            _ori->R[0][2], _ori->R[1][2], _ori->R[2][2]);
        xform.setValue(_ori->R[0][0], _ori->R[0][1], _ori->R[0][2],
                       _ori->R[1][0], _ori->R[1][1], _ori->R[1][2],
                       _ori->R[2][0], _ori->R[2][1], _ori->R[2][2]);
        xform.getRotation(bq);

        // draw the axes
        xform.setValue(1,0,0,   0,1,0,    0,0,1);
        xform.getRotation(axq);
        oriq = bq * axq;
        axes[0].pose.orientation.x = oriq.getX();
        axes[0].pose.orientation.y = oriq.getY();
        axes[0].pose.orientation.z = oriq.getZ();
        axes[0].pose.orientation.w = oriq.getW();
        axes[0].color.r = 1.0f;

        xform.setValue(0,-1,0,    1,0,0,     0,0,1);
        xform.getRotation(axq);
        oriq = bq * axq;
        axes[1].pose.orientation.x = oriq.getX();
        axes[1].pose.orientation.y = oriq.getY();
        axes[1].pose.orientation.z = oriq.getZ();
        axes[1].pose.orientation.w = oriq.getW();
        axes[1].color.g = 1.0f;

        xform.setValue(0,0,-1,   0,1,0,    1,0,0);
        xform.getRotation(axq);
        oriq = bq * axq;
        axes[2].pose.orientation.x = oriq.getX();
        axes[2].pose.orientation.y = oriq.getY();
        axes[2].pose.orientation.z = oriq.getZ();
        axes[2].pose.orientation.w = oriq.getW();
        axes[2].color.b = 1.0f;

        // Publish the marker
        vis_pub1.publish(axes[0]);
        vis_pub1.publish(axes[1]);
        vis_pub1.publish(axes[2]);
    }

    int draw_R_sphere=0;
    if (draw_R_sphere)
    {
        marker1.type = visualization_msgs::Marker::SPHERE;

        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker1.header.frame_id = "/base_link_R";
        marker1.id = 1;
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker1.pose.position.x = device0->mech[right].pos.x/1e6;
        marker1.pose.position.y = device0->mech[right].pos.y/1e6;
        marker1.pose.position.z = device0->mech[right].pos.z/1e6;

        // Get quaternion representation of rotation
        _ori = &(device0->mech[right].ori);
        xform.setValue(_ori->R[0][0], _ori->R[0][1], _ori->R[0][2],
                       _ori->R[1][0], _ori->R[1][1], _ori->R[1][2],
                       _ori->R[2][0], _ori->R[2][1], _ori->R[2][2]);
        xform.getRotation(bq);
        marker1.pose.orientation.x = bq.getX();
        marker1.pose.orientation.y = bq.getY();
        marker1.pose.orientation.z = bq.getZ();
        marker1.pose.orientation.w = bq.getW();

        // Set the color -- be sure to set alpha to something non-zero!
        marker1.color.r = 0.0f;
        marker1.color.g = 1.0f;
        marker1.color.b = 0.0f;
        marker1.color.a = 1.0;
        // Publish the marker
        vis_pub1.publish(marker1);
    }



    int draw_R_axes=1;
    if (draw_R_axes)
    {
        for (int i=0;i<3;i++)
        {
            axes[i].type = visualization_msgs::Marker::ARROW;
            axes[i].action = visualization_msgs::Marker::ADD;   // Set the marker action.  Options are ADD and DELETE
            axes[i].header.stamp = ros::Time::now();
            axes[i].ns = "RCM_marker";
            axes[i].lifetime = ros::Duration();
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            axes[i].scale.x = 0.020;
            axes[i].scale.y = 0.020;
            axes[i].scale.z = 0.020;

            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            axes[i].header.frame_id = "/base_link_R";
            axes[i].id = 30+i;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            axes[i].pose.position.x = device0->mech[right].pos.x/1e6;
            axes[i].pose.position.y = device0->mech[right].pos.y/1e6;
            axes[i].pose.position.z = device0->mech[right].pos.z/1e6;

            // Set the color -- be sure to set alpha to something non-zero!
            axes[i].color.r = 0.0f;
            axes[i].color.g = 0.0f;
            axes[i].color.b = 0.0f;
            axes[i].color.a = 1.0;

        }

        // get the transform rotation
        _ori = &(device0->mech[right].ori);
//        xform.setValue(_ori->R[0][0], _ori->R[1][0], _ori->R[2][0],
//                            _ori->R[0][1], _ori->R[1][1], _ori->R[2][1],
//                            _ori->R[0][2], _ori->R[1][2], _ori->R[2][2]);
        xform.setValue(_ori->R[0][0], _ori->R[0][1], _ori->R[0][2],
                       _ori->R[1][0], _ori->R[1][1], _ori->R[1][2],
                       _ori->R[2][0], _ori->R[2][1], _ori->R[2][2]);
        xform.getRotation(bq);

        // draw the axes
        xform.setValue(1,0,0,   0,1,0,    0,0,1);
        xform.getRotation(axq);
        oriq = bq * axq;
        axes[0].pose.orientation.x = oriq.getX();
        axes[0].pose.orientation.y = oriq.getY();
        axes[0].pose.orientation.z = oriq.getZ();
        axes[0].pose.orientation.w = oriq.getW();
        axes[0].color.r = 1.0f;

        xform.setValue(0,-1,0,    1,0,0,     0,0,1);
        xform.getRotation(axq);
        oriq = bq * axq;
        axes[1].pose.orientation.x = oriq.getX();
        axes[1].pose.orientation.y = oriq.getY();
        axes[1].pose.orientation.z = oriq.getZ();
        axes[1].pose.orientation.w = oriq.getW();
        axes[1].color.g = 1.0f;

        xform.setValue(0,0,-1,   0,1,0,    1,0,0);
        xform.getRotation(axq);
        oriq = bq * axq;
        axes[2].pose.orientation.x = oriq.getX();
        axes[2].pose.orientation.y = oriq.getY();
        axes[2].pose.orientation.z = oriq.getZ();
        axes[2].pose.orientation.w = oriq.getW();
        axes[2].color.b = 1.0f;

        // Publish the marker
        vis_pub1.publish(axes[0]);
        vis_pub1.publish(axes[1]);
        vis_pub1.publish(axes[2]);
    }

    int draw_L2_sphere = 0;
    if (draw_L2_sphere)
    {
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker1.header.frame_id = "/base_link_L2";
        marker1.id = 2;
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker1.pose.position.x = device0->mech[left].pos_d.x/1e6;
        marker1.pose.position.y = device0->mech[left].pos_d.y/1e6;
        marker1.pose.position.z = device0->mech[left].pos_d.z/1e6;
        _ori = &(device0->mech[left].ori_d);
        xform.setValue(_ori->R[0][0], _ori->R[0][1], _ori->R[0][2],
                       _ori->R[1][0], _ori->R[1][1], _ori->R[1][2],
                       _ori->R[2][0], _ori->R[2][1], _ori->R[2][2]);
        xform.getRotation(bq);
        marker1.pose.orientation.x = bq.getX();
        marker1.pose.orientation.y = bq.getY();
        marker1.pose.orientation.z = bq.getZ();
        marker1.pose.orientation.w = bq.getW();

        // Set the color -- be sure to set alpha to something non-zero!
        marker1.color.r = 1.0f;
        marker1.color.g = 0.5f;
        marker1.color.b = 0.0f;
        marker1.color.a = 1.0;
        // Publish the marker
        vis_pub2.publish(marker1);
    }

    int draw_L2_axes = 1;
    if (draw_L2_axes)
    {
        for (int i=0;i<3;i++)
        {
            axes[i].type = visualization_msgs::Marker::ARROW;
            axes[i].action = visualization_msgs::Marker::ADD;   // Set the marker action.  Options are ADD and DELETE
            axes[i].header.stamp = ros::Time::now();
            axes[i].ns = "RCM_marker";
            axes[i].lifetime = ros::Duration();
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            axes[i].scale.x = 0.020;
            axes[i].scale.y = 0.020;
            axes[i].scale.z = 0.020;

            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            axes[i].header.frame_id = "/base_link_L2";
            axes[i].id = 50+i;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            axes[i].pose.position.x = device0->mech[left].pos_d.x/1e6;
            axes[i].pose.position.y = device0->mech[left].pos_d.y/1e6;
            axes[i].pose.position.z = device0->mech[left].pos_d.z/1e6;

            // Set the color -- be sure to set alpha to something non-zero!
            axes[i].color.r = 0.0f;
            axes[i].color.g = 0.0f;
            axes[i].color.b = 0.0f;
            axes[i].color.a = 1.0;

        }
        // Get the device transform
        _ori = &(device0->mech[left].ori_d);
        xform.setValue(_ori->R[0][0], _ori->R[0][1], _ori->R[0][2],
                       _ori->R[1][0], _ori->R[1][1], _ori->R[1][2],
                       _ori->R[2][0], _ori->R[2][1], _ori->R[2][2]);
        xform.getRotation(bq);
        // Draw the axes
        xform.setValue(1,0,0,   0,1,0,    0,0,1);
        xform.getRotation(axq);
        oriq = bq * axq;
        axes[0].pose.orientation.x = oriq.getX();
        axes[0].pose.orientation.y = oriq.getY();
        axes[0].pose.orientation.z = oriq.getZ();
        axes[0].pose.orientation.w = oriq.getW();
        axes[0].color.r = 1.0f;

        xform.setValue(0,-1,0,    1,0,0,     0,0,1);
        xform.getRotation(axq);
        oriq = bq * axq;
        axes[1].pose.orientation.x = oriq.getX();
        axes[1].pose.orientation.y = oriq.getY();
        axes[1].pose.orientation.z = oriq.getZ();
        axes[1].pose.orientation.w = oriq.getW();
        axes[1].color.g = 1.0f;

        xform.setValue(0,0,-1,   0,1,0,    1,0,0);
        xform.getRotation(axq);
        oriq = bq * axq;
        axes[2].pose.orientation.x = oriq.getX();
        axes[2].pose.orientation.y = oriq.getY();
        axes[2].pose.orientation.z = oriq.getZ();
        axes[2].pose.orientation.w = oriq.getW();
        axes[2].color.b = 1.0f;

        // Publish the marker
        vis_pub2.publish(axes[0]);
        vis_pub2.publish(axes[1]);
        vis_pub2.publish(axes[2]);
    }

    int draw_R2_sphere = 0;
    if (draw_R2_sphere)
    {
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker1.header.frame_id = "/base_link_R2";
        marker1.id = 3;
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker1.pose.position.x = device0->mech[right].pos_d.x/1e6;
        marker1.pose.position.y = device0->mech[right].pos_d.y/1e6;
        marker1.pose.position.z = device0->mech[right].pos_d.z/1e6;
        _ori = &(device0->mech[right].ori_d);
        xform.setValue(_ori->R[0][0], _ori->R[0][1], _ori->R[0][2],
                       _ori->R[1][0], _ori->R[1][1], _ori->R[1][2],
                       _ori->R[2][0], _ori->R[2][1], _ori->R[2][2]);
        xform.getRotation(bq);
        marker1.pose.orientation.x = bq.getX();
        marker1.pose.orientation.y = bq.getY();
        marker1.pose.orientation.z = bq.getZ();
        marker1.pose.orientation.w = bq.getW();
        // Set the color -- be sure to set alpha to something non-zero!
        marker1.color.r = 0.5f;
        marker1.color.g = 1.0f;
        marker1.color.b = 0.0f;
        marker1.color.a = 1.0;
        // Publish the marker
        vis_pub2.publish(marker1);
    }

    int draw_R2_axes = 1;
    if (draw_R2_axes)
    {
        for (int i=0;i<3;i++)
        {
            axes[i].type = visualization_msgs::Marker::ARROW;
            axes[i].action = visualization_msgs::Marker::ADD;   // Set the marker action.  Options are ADD and DELETE
            axes[i].header.stamp = ros::Time::now();
            axes[i].ns = "RCM_marker";
            axes[i].lifetime = ros::Duration();
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            axes[i].scale.x = 0.020;
            axes[i].scale.y = 0.020;
            axes[i].scale.z = 0.020;

            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            axes[i].header.frame_id = "/base_link_R2";
            axes[i].id = 40+i;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            axes[i].pose.position.x = device0->mech[right].pos_d.x/1e6;
            axes[i].pose.position.y = device0->mech[right].pos_d.y/1e6;
            axes[i].pose.position.z = device0->mech[right].pos_d.z/1e6;

            // Set the color -- be sure to set alpha to something non-zero!
            axes[i].color.r = 0.0f;
            axes[i].color.g = 0.0f;
            axes[i].color.b = 0.0f;
            axes[i].color.a = 1.0;

        }
        // Get the device transform
        _ori = &(device0->mech[right].ori_d);
        xform.setValue(_ori->R[0][0], _ori->R[0][1], _ori->R[0][2],
                       _ori->R[1][0], _ori->R[1][1], _ori->R[1][2],
                       _ori->R[2][0], _ori->R[2][1], _ori->R[2][2]);
        xform.getRotation(bq);
        // Draw the axes
        xform.setValue(1,0,0,   0,1,0,    0,0,1);
        xform.getRotation(axq);
        oriq = bq * axq;
        axes[0].pose.orientation.x = oriq.getX();
        axes[0].pose.orientation.y = oriq.getY();
        axes[0].pose.orientation.z = oriq.getZ();
        axes[0].pose.orientation.w = oriq.getW();
        axes[0].color.r = 1.0f;

        xform.setValue(0,-1,0,    1,0,0,     0,0,1);
        xform.getRotation(axq);
        oriq = bq * axq;
        axes[1].pose.orientation.x = oriq.getX();
        axes[1].pose.orientation.y = oriq.getY();
        axes[1].pose.orientation.z = oriq.getZ();
        axes[1].pose.orientation.w = oriq.getW();
        axes[1].color.g = 1.0f;

        xform.setValue(0,0,-1,   0,1,0,    1,0,0);
        xform.getRotation(axq);
        oriq = bq * axq;
        axes[2].pose.orientation.x = oriq.getX();
        axes[2].pose.orientation.y = oriq.getY();
        axes[2].pose.orientation.z = oriq.getZ();
        axes[2].pose.orientation.w = oriq.getW();
        axes[2].color.b = 1.0f;

        // Publish the marker
        vis_pub2.publish(axes[0]);
        vis_pub2.publish(axes[1]);
        vis_pub2.publish(axes[2]);
    }

    int draw_xx_axes=0;
    if (draw_xx_axes)
    {
        for (int i=0;i<3;i++)
        {
            axes[i].type = visualization_msgs::Marker::ARROW;
            axes[i].action = visualization_msgs::Marker::ADD;   // Set the marker action.  Options are ADD and DELETE
            axes[i].header.stamp = ros::Time::now();
            axes[i].ns = "RCM_marker";
            axes[i].lifetime = ros::Duration();
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            axes[i].scale.x = 0.020;
            axes[i].scale.y = 0.020;
            axes[i].scale.z = 0.020;

            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            axes[i].header.frame_id = "/link3_L2";
            axes[i].id = 20+i;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            axes[i].pose.position.x = 0;
            axes[i].pose.position.y = 0;
            axes[i].pose.position.z = 0;

            // Set the color -- be sure to set alpha to something non-zero!
            axes[i].color.r = 0.0f;
            axes[i].color.g = 0.0f;
            axes[i].color.b = 0.0f;
            axes[i].color.a = 1.0;

        }

        // get the transform rotation
        _ori = &(device0->mech[left].ori);

        // draw the axes
        xform.setValue(1,0,0,   0,1,0,    0,0,1);
        xform.getRotation(axq);
        oriq = axq;
        axes[0].pose.orientation.x = oriq.getX();
        axes[0].pose.orientation.y = oriq.getY();
        axes[0].pose.orientation.z = oriq.getZ();
        axes[0].pose.orientation.w = oriq.getW();
        axes[0].color.r = 1.0f;

        xform.setValue(0,-1,0,    1,0,0,     0,0,1);
        xform.getRotation(axq);
        oriq = axq;
        axes[1].pose.orientation.x = oriq.getX();
        axes[1].pose.orientation.y = oriq.getY();
        axes[1].pose.orientation.z = oriq.getZ();
        axes[1].pose.orientation.w = oriq.getW();
        axes[1].color.g = 1.0f;

        xform.setValue(0,0,-1,   0,1,0,    1,0,0);
        xform.getRotation(axq);
        oriq = axq;
        axes[2].pose.orientation.x = oriq.getX();
        axes[2].pose.orientation.y = oriq.getY();
        axes[2].pose.orientation.z = oriq.getZ();
        axes[2].pose.orientation.w = oriq.getW();
        axes[2].color.b = 1.0f;

        // Publish the marker
        vis_pub2.publish(axes[0]);
        vis_pub2.publish(axes[1]);
        vis_pub2.publish(axes[2]);
    }



    int draw_xxx_axes=0;
    if (draw_xxx_axes)
    {
        for (int i=0;i<3;i++)
        {
            axes[i].type = visualization_msgs::Marker::ARROW;
            axes[i].action = visualization_msgs::Marker::ADD;   // Set the marker action.  Options are ADD and DELETE
            axes[i].header.stamp = ros::Time::now();
            axes[i].ns = "RCM_marker";
            axes[i].lifetime = ros::Duration();
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            axes[i].scale.x = 0.020;
            axes[i].scale.y = 0.020;
            axes[i].scale.z = 0.020;

            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            axes[i].header.frame_id = "/link3_R2";
            axes[i].id = 20+i;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            axes[i].pose.position.x = 0;
            axes[i].pose.position.y = 0;
            axes[i].pose.position.z = 0;

            // Set the color -- be sure to set alpha to something non-zero!
            axes[i].color.r = 0.0f;
            axes[i].color.g = 0.0f;
            axes[i].color.b = 0.0f;
            axes[i].color.a = 1.0;

        }

        // get the transform rotation
        _ori = &(device0->mech[left].ori);

        // draw the axes
        xform.setValue(1,0,0,   0,1,0,    0,0,1);
        xform.getRotation(axq);
        oriq = axq;
        axes[0].pose.orientation.x = oriq.getX();
        axes[0].pose.orientation.y = oriq.getY();
        axes[0].pose.orientation.z = oriq.getZ();
        axes[0].pose.orientation.w = oriq.getW();
        axes[0].color.r = 1.0f;

        xform.setValue(0,-1,0,    1,0,0,     0,0,1);
        xform.getRotation(axq);
        oriq = axq;
        axes[1].pose.orientation.x = oriq.getX();
        axes[1].pose.orientation.y = oriq.getY();
        axes[1].pose.orientation.z = oriq.getZ();
        axes[1].pose.orientation.w = oriq.getW();
        axes[1].color.g = 1.0f;

        xform.setValue(0,0,-1,   0,1,0,    1,0,0);
        xform.getRotation(axq);
        oriq = axq;
        axes[2].pose.orientation.x = oriq.getX();
        axes[2].pose.orientation.y = oriq.getY();
        axes[2].pose.orientation.z = oriq.getZ();
        axes[2].pose.orientation.w = oriq.getW();
        axes[2].color.b = 1.0f;

        // Publish the marker
        vis_pub2.publish(axes[0]);
        vis_pub2.publish(axes[1]);
        vis_pub2.publish(axes[2]);
    }


}
