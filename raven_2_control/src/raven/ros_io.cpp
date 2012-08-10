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

#include "log.h"
#include "utils.h"
#include "mapping.h"
#include "kinematics_defines.h"
#include "itp_teleoperation.h"

#include <tf/transform_datatypes.h>
#include <raven_2_control/raven_state.h>
#include <raven_2_control/torque_command.h>
#include <raven_2_control/joint_command.h>

#include <raven_2_msgs/Constants.h>
#include <raven_2_msgs/RavenCommand.h>
#include <raven_2_msgs/RavenState.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

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
ros::Publisher pub_ravenstate;
ros::Publisher joint_publisher;

ros::Publisher pub_command_pose[2];
ros::Publisher pub_tool_pose[2];
ros::Publisher pub_master_pose_raw[2];
ros::Publisher pub_master_pose[2];

ros::Publisher vis_pub1;
ros::Publisher vis_pub2;
ros::Publisher pub_ravenstate_old;

ros::Subscriber cmd_sub;
ros::Subscriber cmd_pose_sub[2];
ros::Subscriber torque_sub1;
ros::Subscriber torque_sub2;
ros::Subscriber joint_sub;

tf::TransformListener* tf_listener;

void ravenCmdCallback(const raven_2_msgs::RavenCommand& cmd);
void torqueCallback1(const torque_command::ConstPtr& torque_cmd);
void jointCallback(const joint_command::ConstPtr& joint_cmd);

void publish_joints(struct robot_device*);
void publish_command_pose(struct robot_device*);
void publish_tool_pose(struct robot_device*);
void publish_master_pose(struct robot_device*);
void publish_marker(struct robot_device*);

#define RAVEN_STATE_TOPIC "raven_state"
#define COMMAND_POSE_TOPIC(side) (std::string("tool_pose/command/")+std::string(side))
#define TOOL_POSE_TOPIC(side)    (std::string("tool_pose/")+std::string(side))
#define MASTER_POSE_TOPIC(side) (std::string("master_pose/")+std::string(side))
#define MASTER_POSE_RAW_TOPIC(side) (std::string("master_pose/raw/")+std::string(side))

#define RAVEN_COMMAND_TOPIC "raven_command"
#define RAVEN_COMMAND_POSE_TOPIC(side) (std::string(RAVEN_COMMAND_TOPIC) + std::string("_pose/")+std::string(side))

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

void cmd_pose_callback(const geometry_msgs::PoseStampedConstPtr& pose,int armId) {
	raven_2_msgs::RavenCommand cmd;
	cmd.header = pose->header;
	cmd.pedal_down = true;
	raven_2_msgs::ArmCommand arm_cmd;
	arm_cmd.active = true;
	arm_cmd.tool_command.absolute = true;
	arm_cmd.tool_command.tool_pose = pose->pose;
	cmd.arms[armId] = arm_cmd;
	//ravenCmdCallback(cmd);
}

void ravenCmdCallback(const raven_2_msgs::RavenCommand& cmd) {
    static ros::Time last_call(-1);

    ros::Time now = ros::Time::now();
    if (last_call.toSec() < 0) { last_call = now; return; }
    ros::Duration since_last_call = (now-last_call);
    last_call = now;

	_localio_counter++;

	//printf("cmd callback!\n");

	btVector3 p;
	btQuaternion rot;
	btMatrix3x3 rot_mx_temp;

	param_pass data1;
	getRcvdParams(&data1);

	if (!cmd.pedal_down) {
		if (_localio_counter % PRINT_EVERY == 0) { printf("inactive!\n"); }
		data1.surgeon_mode =  SURGEON_DISENGAGED;
	} else {
		data1.surgeon_mode = SURGEON_ENGAGED;
	}
	if (data1.surgeon_mode) {
		for (int mech_ind=0;mech_ind<NUM_MECH;mech_ind++) {
			int arm_serial = USBBoards.boards[mech_ind];
			int arm_id = armIdFromSerial(arm_serial);
			if (!cmd.arms[arm_id].active) { continue; }
			if (_localio_counter % PRINT_EVERY == 0 || true) {
				if (arm_id == 0) {
					//printf("*********active %d!-----------------------------------\n",i);
				} else {
					//printf("*********active %d!XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n",i);
				}
			}

			tf::Transform tool_pose_raw;
			tf::poseMsgToTF(cmd.arms[arm_id].tool_command.tool_pose,tool_pose_raw);

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

			p = tool_pose.getOrigin();
			/*
			p[0] = cmd.arms[arm_id].tool_command.tool_pose.position.x;
			p[1] = cmd.arms[arm_id].tool_command.tool_pose.position.y;
			p[2] = cmd.arms[arm_id].tool_command.tool_pose.position.z;
			*/

			if (cmd.arms[arm_id].tool_command.absolute) {
				master_raw_position[arm_id] = p;
			} else {
				master_raw_position[arm_id] += p;
			}

			rot = tool_pose.getRotation();
			/*
			rot.setX(cmd.arms[arm_id].tool_command.tool_pose.orientation.x);
			rot.setY(cmd.arms[arm_id].tool_command.tool_pose.orientation.y);
			rot.setZ(cmd.arms[arm_id].tool_command.tool_pose.orientation.z);
			rot.setW(cmd.arms[arm_id].tool_command.tool_pose.orientation.w);
			*/

			master_raw_orientation[arm_id].setRotation(rot);

			btMatrix3x3 rot_mx_temp(rot);

			//p = btMatrix3x3(0,1,0,  -1,0,0,  0,0,1) * p;

			if (cmd.arms[arm_id].tool_command.absolute) {
				data1.xd[mech_ind].x = p.x() * MICRON_PER_M;
				data1.xd[mech_ind].y = p.y() * MICRON_PER_M;
				data1.xd[mech_ind].z = p.z() * MICRON_PER_M;

				master_position[arm_id] = p;
			} else {
				data1.xd[mech_ind].x += p.x() * MICRON_PER_M;
				data1.xd[mech_ind].y += p.y() * MICRON_PER_M;
				data1.xd[mech_ind].z += p.z() * MICRON_PER_M;

				master_position[arm_id] += p;
			}
			//rot_mx_temp = btMatrix3x3(0,1,0,  -1,0,0,  0,0,1) * rot_mx_temp * btMatrix3x3(1,0,0,  0,-1,0,  0,0,-1);
			rot_mx_temp = rot_mx_temp * TOOL_POSE_AXES_TRANSFORM.getBasis().inverse();
			master_orientation[arm_id] = rot_mx_temp;

			float grasp_scale_factor = 500 * since_last_call.toSec();
			if (grasp_scale_factor < 1) {
				grasp_scale_factor = 1;
			}
			const int graspmax = (M_PI/2 * 1000);
			const int graspmin = (-20.0 * 1000.0 DEG2RAD);
			if (arm_serial == GOLD_ARM_SERIAL) {
				data1.rd[mech_ind].grasp = saturate(data1.rd[mech_ind].grasp + grasp_scale_factor*cmd.arms[arm_id].tool_command.grasp,graspmin,graspmax);
			} else {
				data1.rd[mech_ind].grasp = saturate(data1.rd[mech_ind].grasp - grasp_scale_factor*cmd.arms[arm_id].tool_command.grasp,-graspmax,-graspmin);
			}

			float insertion_scale = (Z_INS_MAX_LIMIT - Z_INS_MIN_LIMIT) / 10.;
			int mechId = -1;
			for (int m=0;m<NUM_MECH;m++) {
				if (armIdFromMechType(device0ptr->mech[m].type) == armIdFromSerial(arm_serial)) {
					mechId = m;
					break;
				}
			}

			for (size_t i=0;i<cmd.arms[arm_id].joint_types.size();i++) {
				if (cmd.arms[arm_id].joint_types[i] == raven_2_msgs::Constants::JOINT_TYPE_INSERTION
						&& cmd.arms[arm_id].joint_commands[i].command_type == raven_2_msgs::JointCommand::COMMAND_TYPE_VELOCITY) {
					float ins_amt = since_last_call.toSec() * insertion_scale * cmd.arms[arm_id].joint_commands[i].value;
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

					data1.xd[mech_ind].x += z_ins_in_world.x() * MICRON_PER_M;
					data1.xd[mech_ind].y += z_ins_in_world.y() * MICRON_PER_M;
					data1.xd[mech_ind].z += z_ins_in_world.z() * MICRON_PER_M;
				}
			}



			for (int j=0;j<3;j++) {
				for (int k=0;k<3;k++) {
					data1.rd[mech_ind].R[j][k] = rot_mx_temp[j][k];
				}
			}
		}
	}
	//isUpdated = TRUE;
	writeUpdate(&data1);
}

void publish_ravenstate_old(struct robot_device *device0,u_08 runlevel,u_08 sublevel,ros::Duration since_last_pub);

void publish_ros(struct robot_device *device0,u_08 runlevel,u_08 sublevel) {
	static bool hasHomed = false;
	static raven_2_msgs::RavenState raven_state;
	raven_state.arms.clear();

	if (runlevel == RL_PEDAL_DN || runlevel == RL_PEDAL_UP) {
		hasHomed = true;
	}

	static ros::Time last_pub;
	static ros::Duration interval(0.01);
	static ros::Duration since_last_pub;
	if (!checkRate(last_pub,interval,since_last_pub)) { return; }

	publish_master_pose(device0);

	//Only publish these if we've homed already
	if (hasHomed) {
		publish_joints(device0);
		//publish_marker(device0);
		publish_command_pose(device0);
		publish_tool_pose(device0);
	}

	publish_ravenstate_old(device0,runlevel,sublevel,since_last_pub);

	//raven_state

	raven_state.header.stamp = ros::Time::now();
	raven_state.header.frame_id = "/0_link";
	raven_state.runlevel = runlevel;
	raven_state.sublevel = sublevel;
	raven_state.surgeon_mode = device0->surgeon_mode;
	for (int m = 0; m < NUM_MECH; m++) {
		raven_2_msgs::ArmState arm_state;
		uint8_t arm_type;
		btMatrix3x3 transform;
		switch (device0->mech[m].type) {
		case GOLD_ARM:
			arm_type = raven_2_msgs::Constants::ARM_TYPE_GOLD;
			transform = btMatrix3x3(1,0,0,  0,-1,0,  0,0,-1);
			break;
		case GREEN_ARM:
			arm_type = raven_2_msgs::Constants::ARM_TYPE_GREEN;
			transform = btMatrix3x3(1,0,0,  0,-1,0,  0,0,-1);
			break;
		default:
			ROS_ERROR_STREAM("Unknown mech type"<<device0->mech[m].type);
			continue;
		}
		arm_state.type = arm_type;

		arm_state.base_pose = toRos(device0->mech[m].base_pos,device0->mech[m].base_ori,transform);

		switch (device0->mech[m].type) {
		case TOOL_NONE: arm_state.tool_type = raven_2_msgs::Constants::TOOL_TYPE_NONE; break;
		case TOOL_GRASPER_10MM: arm_state.tool_type = raven_2_msgs::Constants::TOOL_TYPE_GRASPER_10MM; break;
		case TOOL_GRASPER_8MM: arm_state.tool_type = raven_2_msgs::Constants::TOOL_TYPE_GRASPER_8MM; break;
		}

		arm_state.tool_pose = toRos(device0->mech[m].pos,device0->mech[m].ori,transform);
		arm_state.tool_pose_desired = toRos(device0->mech[m].pos_d,device0->mech[m].ori_d,transform);

		arm_state.grasp = rosGraspFromMech(armIdFromMechType(device0->mech[m].type),device0->mech[m].ori.grasp);
		arm_state.grasp_desired = rosGraspFromMech(armIdFromMechType(device0->mech[m].type),device0->mech[m].ori_d.grasp);

		int grasp1_index=-1;
		int grasp2_index=-1;
		for (int j=0;j < MAX_DOF_PER_MECH; j++) {
			raven_2_msgs::JointState joint_state;
			int joint_type = jointTypeFromCombinedType(device0->mech[m].joint[j].type);
			switch (joint_type) {
			case SHOULDER: joint_state.type = raven_2_msgs::Constants::JOINT_TYPE_SHOULDER; break;
			case ELBOW: joint_state.type = raven_2_msgs::Constants::JOINT_TYPE_ELBOW; break;
			case Z_INS: joint_state.type = raven_2_msgs::Constants::JOINT_TYPE_INSERTION; break;
			case TOOL_ROT: joint_state.type = raven_2_msgs::Constants::JOINT_TYPE_ROTATION; break;
			case WRIST: joint_state.type = raven_2_msgs::Constants::JOINT_TYPE_PITCH; break;
			case GRASP1: joint_state.type = raven_2_msgs::Constants::JOINT_TYPE_GRASP_FINGER1; grasp1_index = j; break;
			case GRASP2: joint_state.type = raven_2_msgs::Constants::JOINT_TYPE_GRASP_FINGER2; grasp2_index = j; break;
			case NO_CONNECTION: continue;
			}

			switch (device0->mech[m].joint[j].state) {
			case jstate_not_ready: joint_state.state = raven_2_msgs::JointState::STATE_NOT_READY; break;
			case jstate_pos_unknown: joint_state.state = raven_2_msgs::JointState::STATE_POS_UNKNOWN; break;
			case jstate_homing1: joint_state.state = raven_2_msgs::JointState::STATE_HOMING1; break;
			case jstate_homing2: joint_state.state = raven_2_msgs::JointState::STATE_HOMING2; break;
			case jstate_ready: joint_state.state = raven_2_msgs::JointState::STATE_READY; break;
			case jstate_wait: joint_state.state = raven_2_msgs::JointState::STATE_WAIT; break;
			case jstate_hard_stop: joint_state.state = raven_2_msgs::JointState::STATE_HARD_STOP; break;
			default: joint_state.state = raven_2_msgs::JointState::STATE_LAST_TYPE; break;
			}

			joint_state.encoder_value = device0->mech[m].joint[j].enc_val;
			joint_state.encoder_offset = device0->mech[m].joint[j].enc_offset;
			joint_state.dac_command = device0->mech[m].joint[j].current_cmd;

			joint_state.position = device0->mech[m].joint[j].jpos;
			joint_state.velocity = device0->mech[m].joint[j].jvel;

			joint_state.motor_position = device0->mech[m].joint[j].mpos;
			joint_state.motor_velocity = device0->mech[m].joint[j].mvel;

			joint_state.torque = device0->mech[m].joint[j].tau;
			joint_state.gravitation_torque_estimate = device0->mech[m].joint[j].tau_g;

			joint_state.integrated_position_error = device0->mech[m].joint[j].perror_int;

			raven_2_msgs::JointCommand joint_cmd;
			joint_cmd.command_type = raven_2_msgs::JointCommand::COMMAND_TYPE_POSITION;
			joint_cmd.value = device0->mech[m].joint[j].jpos_d;
			/*
	    		joint_cmd.position = device0->mech[m].joint[j].jpos;
	    		joint_cmd.velocity = device0->mech[m].joint[j].jvel;

	    		joint_cmd.motor_position = device0->mech[m].joint[j].mpos;
	    		joint_cmd.motor_velocity = device0->mech[m].joint[j].mvel;

	    		joint_cmd.torque = device0->mech[m].joint[j].tau;
			 */

			joint_state.command = joint_cmd;

			arm_state.joints.push_back(joint_state);
		}

		raven_state.arms.push_back(arm_state);
	}

	pub_ravenstate.publish(raven_state);
}

void publish_command_pose(struct robot_device* device0) {
	static ros::Time last_pub;
	static ros::Duration interval(0.1);
	static ros::Duration since_last_pub;
	if (!checkRate(last_pub,interval,since_last_pub)) { return; }

	geometry_msgs::PoseStamped command_pose;
	command_pose.header.stamp = ros::Time::now();
	command_pose.header.frame_id = "/0_link";

	mechanism* _mech = NULL;
	int mechnum = 0;
	while (loop_over_mechs(device0,_mech,mechnum)) {
		int armId = armIdFromMechType(_mech->type);
		command_pose.pose.position.x = ((float)_mech->pos_d.x) / MICRON_PER_M;
		command_pose.pose.position.y = ((float)_mech->pos_d.y) / MICRON_PER_M;
		command_pose.pose.position.z = ((float)_mech->pos_d.z) / MICRON_PER_M;

		btQuaternion rot;
		(toBt(_mech->ori_d.R) * TOOL_POSE_AXES_TRANSFORM.getBasis()).getRotation(rot);

		command_pose.pose.orientation.x = rot.x();
		command_pose.pose.orientation.y = rot.y();
		command_pose.pose.orientation.z = rot.z();
		command_pose.pose.orientation.w = rot.w();

		pub_command_pose[armId].publish(command_pose);
	}
}

void publish_tool_pose(struct robot_device* device0) {
	static ros::Time last_pub;
	static ros::Duration interval(0.1);
	static ros::Duration since_last_pub;
	if (!checkRate(last_pub,interval,since_last_pub)) { return; }

	geometry_msgs::PoseStamped tool_pose;
	tool_pose.header.stamp = ros::Time::now();
	tool_pose.header.frame_id = "/0_link";

	mechanism* _mech = NULL;
	int mechnum = 0;
	while (loop_over_mechs(device0,_mech,mechnum)) {
		int armId = armIdFromMechType(_mech->type);

		btVector3 pos = btVector3((float)_mech->pos.x,(float)_mech->pos.y,(float)_mech->pos.z)/MICRON_PER_M;
		btQuaternion rot;
		toBt(_mech->ori.R).getRotation(rot);
		btTransform pose = btTransform(rot,pos) * TOOL_POSE_AXES_TRANSFORM;

		tf::poseTFToMsg(pose,tool_pose.pose);

		pub_tool_pose[armId].publish(tool_pose);
	}
}

void publish_master_pose(struct robot_device* device0) {
	static ros::Time last_pub;
	static ros::Duration interval(0.1);
	static ros::Duration since_last_pub;
	if (!checkRate(last_pub,interval,since_last_pub)) { return; }

	btQuaternion q;

	geometry_msgs::PoseStamped master_pose;
	master_pose.header.stamp = ros::Time::now();
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

		pub_master_pose_raw[armId].publish(master_pose_raw);
		pub_master_pose[armId].publish(master_pose);
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

    sensor_msgs::JointState joint_state;
    //update joint_state
    joint_state.header.stamp = ros::Time::now();
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

    while (loop_over_mechs(device0,_mech,mechnum)) {
    	_joint = NULL;
    	std::string armName = rosArmName(armIdFromMechType(_mech->type));
    	while (loop_over_joints(_mech,_joint,jnum)) {
    		joint_state.name.push_back(rosJointName(jointTypeFromCombinedType(_joint->type)) + "_" + armName);
    		joint_state.position.push_back(_joint->jpos);
    	}
    	joint_state.name.push_back("grasper_yaw_" + armName);
    	joint_state.position.push_back(THY_MECH_FROM_FINGERS(armIdFromMechType(_mech->type),_mech->joint[GRASP1].jpos,_mech->joint[GRASP2].jpos));
    	joint_state.name.push_back("grasper_" + armName);
    	joint_state.position.push_back(rosGraspFromMech(armIdFromMechType(_mech->type),_mech->ori.grasp));
    }

    _mech=NULL;
    _joint=NULL;

    while (loop_over_mechs(device0,_mech,mechnum)) {
    	_joint = NULL;
    	std::string armName = rosArmName(armIdFromMechType(_mech->type));
    	while (loop_over_joints(_mech,_joint,jnum)) {
    		joint_state.name.push_back(rosJointName(jointTypeFromCombinedType(_joint->type)) + "_" + armName + "2");
    		joint_state.position.push_back(_joint->jpos_d);
    	}
    	joint_state.name.push_back("grasper_yaw_" + armName + "2");
    	joint_state.position.push_back(THY_MECH_FROM_FINGERS(armIdFromMechType(_mech->type),_mech->joint[GRASP1].jpos_d,_mech->joint[GRASP2].jpos_d));
    	joint_state.name.push_back("grasper_" + armName + "2");
    	joint_state.position.push_back(rosGraspFromMech(armIdFromMechType(_mech->type),_mech->ori_d.grasp));
    }

    /*
    //======================LEFT ARM===========================
    joint_state.name.push_back("shoulder_L");
    joint_state.position.push_back(device0->mech[left].joint[SHOULDER].jpos);
    joint_state.name.push_back("elbow_L");
    joint_state.position.push_back(device0->mech[left].joint[ELBOW].jpos);
    joint_state.name.push_back("insertion_L");
    joint_state.position.push_back(device0->mech[left].joint[Z_INS].jpos);
    joint_state.name.push_back("tool_roll_L");
    joint_state.position.push_back(device0->mech[left].joint[TOOL_ROT].jpos);
    joint_state.name.push_back("wrist_joint_L");
    joint_state.position.push_back(device0->mech[left].joint[WRIST].jpos);
    joint_state.name.push_back("grasper_joint_1_L");
    joint_state.position.push_back(device0->mech[left].joint[GRASP1].jpos);
    joint_state.name.push_back("grasper_joint_2_L");
    joint_state.position.push_back(device0->mech[left].joint[GRASP2].jpos);
    joint_state.name.push_back("grasper_yaw_L");
    joint_state.position.push_back(THY_MECH_FROM_FINGERS(armIdFromMechType(device0->mech[left].type),device0->mech[left].joint[GRASP1].jpos,device0->mech[left].joint[GRASP2].jpos));
    joint_state.name.push_back("grasper_L");
    joint_state.position.push_back(device0->mech[left].ori.grasp / 1000.);
//======================RIGHT ARM===========================
    joint_state.name.push_back("shoulder_R");
    joint_state.position.push_back(device0->mech[right].joint[SHOULDER].jpos);
    joint_state.name.push_back("elbow_R");
    joint_state.position.push_back(device0->mech[right].joint[ELBOW].jpos);
    joint_state.name.push_back("insertion_R");
    joint_state.position.push_back(device0->mech[right].joint[Z_INS].jpos);
    joint_state.name.push_back("tool_roll_R");
    joint_state.position.push_back(device0->mech[right].joint[TOOL_ROT].jpos);
    joint_state.name.push_back("wrist_joint_R");
    joint_state.position.push_back(device0->mech[right].joint[WRIST].jpos);
    joint_state.name.push_back("grasper_joint_1_R");
    joint_state.position.push_back(device0->mech[right].joint[GRASP1].jpos);
    joint_state.name.push_back("grasper_joint_2_R");
    joint_state.position.push_back(device0->mech[right].joint[GRASP2].jpos);
    joint_state.name.push_back("grasper_yaw_R");
    joint_state.position.push_back(THY_MECH_FROM_FINGERS(armIdFromMechType(device0->mech[right].type),device0->mech[right].joint[GRASP1].jpos,device0->mech[right].joint[GRASP2].jpos));
    joint_state.name.push_back("grasper_R");
    joint_state.position.push_back(-device0->mech[right].ori.grasp / 1000.);
//======================LEFT ARM===========================
    joint_state.name.push_back("shoulder_L2");
    joint_state.position.push_back(device0->mech[left].joint[SHOULDER].jpos_d);
    joint_state.name.push_back("elbow_L2");
    joint_state.position.push_back(device0->mech[left].joint[ELBOW].jpos_d);
    joint_state.name.push_back("insertion_L2");
    joint_state.position.push_back(device0->mech[left].joint[Z_INS].jpos_d);
    joint_state.name.push_back("tool_roll_L2");
    joint_state.position.push_back(device0->mech[left].joint[TOOL_ROT].jpos_d);
    joint_state.name.push_back("wrist_joint_L2");
    joint_state.position.push_back(device0->mech[left].joint[WRIST].jpos_d);
    joint_state.name.push_back("grasper_joint_1_L2");
    joint_state.position.push_back(device0->mech[left].joint[GRASP1].jpos_d);
    joint_state.name.push_back("grasper_joint_2_L2");
    joint_state.position.push_back(device0->mech[left].joint[GRASP2].jpos_d);
    joint_state.name.push_back("grasper_yaw_L2");
    joint_state.position.push_back(THY_MECH_FROM_FINGERS(armIdFromMechType(device0->mech[left].type),device0->mech[left].joint[GRASP1].jpos_d,device0->mech[left].joint[GRASP2].jpos_d));
    joint_state.name.push_back("grasper_L2");
    joint_state.position.push_back(device0->mech[left].ori_d.grasp / 1000.);
//======================RIGHT ARM===========================
    joint_state.name.push_back("shoulder_R2");
    joint_state.position.push_back(device0->mech[right].joint[SHOULDER].jpos_d);
    joint_state.name.push_back("elbow_R2");
    joint_state.position.push_back(device0->mech[right].joint[ELBOW].jpos_d);
    joint_state.name.push_back("insertion_R2");
    joint_state.position.push_back(device0->mech[right].joint[Z_INS].jpos_d);
    joint_state.name.push_back("tool_roll_R2");
    joint_state.position.push_back(device0->mech[right].joint[TOOL_ROT].jpos_d);
    joint_state.name.push_back("wrist_joint_R2");
    joint_state.position.push_back(device0->mech[right].joint[WRIST].jpos_d);
    joint_state.name.push_back("grasper_joint_1_R2");
    joint_state.position.push_back(device0->mech[right].joint[GRASP1].jpos_d);
    joint_state.name.push_back("grasper_joint_2_R2");
    joint_state.position.push_back(device0->mech[right].joint[GRASP2].jpos_d);
    joint_state.name.push_back("grasper_yaw_R2");
    joint_state.position.push_back(THY_MECH_FROM_FINGERS(armIdFromMechType(device0->mech[right].type),device0->mech[right].joint[GRASP1].jpos_d,device0->mech[right].joint[GRASP2].jpos_d));
    joint_state.name.push_back("grasper_R2");
    joint_state.position.push_back(-device0->mech[right].ori_d.grasp / 1000.);
    */

    //Publish the joint states
    joint_publisher.publish(joint_state);

}

int init_pubs(ros::NodeHandle &n,struct robot_device *device0) {
	pub_ravenstate = n.advertise<raven_2_msgs::RavenState>(RAVEN_STATE_TOPIC, 1000);
	joint_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1);

	mechanism* _mech = NULL;
	int mechnum = 0;
	while (loop_over_mechs(device0,_mech,mechnum)) {
		int armId = armIdFromMechType(_mech->type);
		std::string armName = rosArmName(armId);
		pub_command_pose[armId] = n.advertise<geometry_msgs::PoseStamped>( COMMAND_POSE_TOPIC(armName), 1);
		pub_tool_pose[armId] = n.advertise<geometry_msgs::PoseStamped>(TOOL_POSE_TOPIC(armName), 1);
		pub_master_pose_raw[armId] = n.advertise<geometry_msgs::PoseStamped>( MASTER_POSE_TOPIC(armName), 1);
		pub_master_pose[armId] = n.advertise<geometry_msgs::PoseStamped>( MASTER_POSE_RAW_TOPIC(armName), 1);
	}

	vis_pub1 = n.advertise<visualization_msgs::Marker>( "visualization_marker1", 0 );
	vis_pub2 = n.advertise<visualization_msgs::Marker>( "visualization_marker2", 0 );

	pub_ravenstate_old = n.advertise<raven_state>("ravenstate_old", 1000);

	return 0;
}

void init_subs(ros::NodeHandle &n,struct robot_device *device0) {
	std::cout << "Initializing ros subscribers" << std::endl;
	cmd_sub = n.subscribe(RAVEN_COMMAND_TOPIC, 1, ravenCmdCallback);
	mechanism* _mech = NULL;
	int mechnum = 0;
	while (loop_over_mechs(device0,_mech,mechnum)) {
		int armId = armIdFromMechType(_mech->type);
		std::string armName = rosArmName(armId);
		cmd_pose_sub[armId] = n.subscribe<geometry_msgs::PoseStamped>(RAVEN_COMMAND_POSE_TOPIC(armName), 1, boost::bind(cmd_pose_callback,_1,armId));
	}
	torque_sub1 = n.subscribe("torque_cmd1", 1, torqueCallback1);
	//torque_sub2 = n.subscribe("torque_cmd2", 2, torqueCallback2);
	joint_sub = n.subscribe("joint_cmd", 1, jointCallback);

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
