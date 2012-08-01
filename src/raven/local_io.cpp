/***************************************
 *
 * File: local_io.c
 *

Local_io keeps its own copy of DS1 for incorporating new
network-layer and toolkit updates.

The local DS1 copy is protected by a mutex.

This transient copy should then be read to another copy for
active use.

***************************************/

///TODO: Modify the guts of local comm and network layer
#ifndef _GNU_SOURCE
#define _GNU_SOURCE //For realtime posix support. see http://www.gnu.org/s/libc/manual/html_node/Feature-Test-Macros.html
#endif

#include <string.h>
#include <pthread.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <iostream>

#include "log.h"
#include "local_io.h"
#include "utils.h"
#include "mapping.h"
#include "itp_teleoperation.h"
#include "kinematics_defines.h"

extern int NUM_MECH;
extern USBStruct USBBoards;
extern unsigned long int gTime;

static struct param_pass data1;
pthread_mutexattr_t data1MutexAttr;
pthread_mutex_t data1Mutex;

static int _localio_counter;
static btVector3 master_raw_position[2];
static btVector3 master_position[2];
static btMatrix3x3 master_raw_orientation[2];
static btMatrix3x3 master_orientation[2];
static const int PRINT_EVERY_PEDAL_UP   = 1000000000;
static const int PRINT_EVERY_PEDAL_DOWN = 1000;
static int PRINT_EVERY = PRINT_EVERY_PEDAL_DOWN;

volatile int isUpdated; //volatile int instead of atomic_t ///Should we use atomic builtins? http://gcc.gnu.org/onlinedocs/gcc-4.1.2/gcc/Atomic-Builtins.html

btQuaternion Q_ori[2];

// initialize data arrays to zero
// create mutex
int initLocalioData(void)
{
    int i;
    pthread_mutexattr_init(&data1MutexAttr);
    pthread_mutexattr_setprotocol(&data1MutexAttr,PTHREAD_PRIO_INHERIT);
    pthread_mutex_init(&data1Mutex,&data1MutexAttr);

    pthread_mutex_lock(&data1Mutex);
    for (i=0;i<NUM_MECH;i++) {
        data1.xd[i].x = 0;
        data1.xd[i].y = 0;
        data1.xd[i].z = 0;
        data1.rd[i].yaw = 0;
        data1.rd[i].pitch = 0;
        data1.rd[i].roll = 0;
        data1.rd[i].grasp = 0;
        Q_ori[i] = Q_ori[i].getIdentity();
    }
    data1.surgeon_mode=0;
    {
        _localio_counter = 0;
        for (i=0;i<NUM_MECH;i++) {
        	master_raw_position[i] = btVector3(0,0,0);
            master_position[i] = btVector3(0,0,0);
            //_teleop_rot_tracker0[i] = btMatrix3x3::getIdentity();
            //_teleop_rot_tracker[i] = btMatrix3x3::getIdentity();
        }
    }
    pthread_mutex_unlock(&data1Mutex);
    return 0;
}

// --------------------------- //
// - Recieve userspace data  - //
//---------------------------- //
//int recieveUserspace(unsigned int fifo)
int recieveUserspace(void *u,int size)
{
    if (size==sizeof(struct u_struct))
    {
        isUpdated = TRUE;
        teleopIntoDS1((struct u_struct*)u);
    }
    return 0;
}

// teleopIntoDS1()
//
//   Input from the master is put into DS1 as pos_d.
//
void teleopIntoDS1(struct u_struct *t)
{
    _localio_counter++;

    if (_localio_counter % PRINT_EVERY == 0) {
        //print_u_struct_pos(t,0);
        //print_u_struct_pos(t,1);
    	//std::cout << "grasp " << t->grasp[0] << " " << data1.rd[0].grasp << " | " << t->grasp[1] << " " << data1.rd[1].grasp << " | " << t->surgeon_mode << std::endl;
    }

    btVector3 p;
    int i, armidx, armserial;
    pthread_mutex_lock(&data1Mutex);
    btQuaternion rot;
    btMatrix3x3 rot_mx_temp;

    for (i=0;i<NUM_MECH;i++)
    {
        armserial = USBBoards.boards[i]==GREEN_ARM_SERIAL ? GREEN_ARM_SERIAL : GOLD_ARM_SERIAL;
        armidx    = USBBoards.boards[i]==GREEN_ARM_SERIAL ? 1 : 0;

        // apply mapping to teleop data
        p[0] = t->delx[armidx];
        p[1] = t->dely[armidx];
        p[2] = t->delz[armidx];

        //set local quaternion from teleop quaternion data
        rot.setX( t->Qx[armidx] );
        rot.setY( t->Qy[armidx] );
        rot.setZ( t->Qz[armidx] );
        rot.setW( t->Qw[armidx] );

        master_raw_position[armidx] += p/MICRON_PER_M;
        if (armserial == GOLD_ARM_SERIAL) {
        	master_raw_orientation[armidx].setRotation(rot);
        } else {
        	master_raw_orientation[armidx] = master_orientation[armidx] * btMatrix3x3(rot);
        }

        //print teleop pose
        if (_localio_counter % PRINT_EVERY == 0 && armserial == GOLD_ARM_SERIAL) {
        	tb_angles angles = get_tb_angles(rot);
        	tb_angles angles1 = get_tb_angles(master_orientation[armidx]);
        	log_msg("teleop %d (%0.04f %0.04f,%0.04f)  ypr (%0.4f,%0.4f,%0.4f) (%0.4f,%0.4f,%0.4f)",armidx,
        			master_position[armidx].x(),master_position[armidx].y(),master_position[armidx].z(),
        			angles.yaw_deg,angles.pitch_deg,angles.roll_deg,
        			angles1.yaw_deg,angles1.pitch_deg,angles1.roll_deg);
        }


        if (_localio_counter % PRINT_EVERY == 0 && armserial == GOLD_ARM_SERIAL) {
        	printf("(%f %f,%f) ",p.x(),p.y(),p.z());
        	printf("(%f %f,%f)\n",master_raw_position[armidx].x(),master_raw_position[armidx].y(),master_raw_position[armidx].z());
        }
        btQuaternion rot0 = rot;
        fromITP(p, rot, armserial);
        if (!t->surgeon_mode) {
        	master_orientation[armidx].getRotation(rot);
        }
        if (_localio_counter % PRINT_EVERY == 0 && armserial == GOLD_ARM_SERIAL) {
        	printf("(%f %f,%f) ",p.x(),p.y(),p.z());
        	printf("(%f %f,%f)\n",master_position[armidx].x(),master_position[armidx].y(),master_position[armidx].z());
        }


        master_orientation[armidx].setRotation(rot);

        data1.xd[i].x += p.x();
        data1.xd[i].y += p.y();
        data1.xd[i].z += p.z();

        master_position[armidx][0] = data1.xd[i].x/MICRON_PER_M;
        master_position[armidx][1] = data1.xd[i].y/MICRON_PER_M;
        master_position[armidx][2] = data1.xd[i].z/MICRON_PER_M;

        //data1.rd[i].grasp = t->buttonstate[armidx];
        const int graspmax = (M_PI/2 * 1000);
        const int graspmin = (-20.0 * 1000.0 DEG2RAD);
        if (armserial == GOLD_ARM_SERIAL)
        {
            data1.rd[i].grasp -= t->grasp[armidx];
            if (data1.rd[i].grasp>graspmax) data1.rd[i].grasp=graspmax;
            else if(data1.rd[i].grasp<graspmin) data1.rd[i].grasp=graspmin;
        }
        else
        {
            data1.rd[i].grasp += t->grasp[armidx];
            if (data1.rd[i].grasp < -graspmax) data1.rd[i].grasp = -graspmax;
            else if(data1.rd[i].grasp > -graspmin) data1.rd[i].grasp = -graspmin;
        }


        //Add quaternion increment
        if (armserial == GOLD_ARM_SERIAL) {
        	rot_mx_temp.setRotation(rot);
        } else {
        	Q_ori[armidx]= rot*Q_ori[armidx];
        	rot_mx_temp.setRotation(Q_ori[armidx]);
        }

        // Set rotation command
        for (int j=0;j<3;j++)
            for (int k=0;k<3;k++)
                data1.rd[i].R[j][k] = rot_mx_temp[j][k];
    }

//    log_msg("updated d1.xd to: (%d,%d,%d)/(%d,%d,%d)",
//           data1.xd[0].x, data1.xd[0].y, data1.xd[0].z,
//           data1.xd[1].x, data1.xd[1].y, data1.xd[1].z);

    data1.surgeon_mode = t->surgeon_mode;
    pthread_mutex_unlock(&data1Mutex);
}

// checkLocalUpdates()
//
//  returns true if updates have been recieved from master or toolkit since last module update
//  returns false otherwise
//  Also, checks the last time updates were made from master.  If it has been to long
//  surgeon_mode state is set to pedal-up.
int checkLocalUpdates()
{
    static unsigned long int lastUpdated;

    if (isUpdated || lastUpdated == 0)
    {
        lastUpdated = gTime;
    }
    else if (((gTime-lastUpdated) > MASTER_CONN_TIMEOUT) && ( data1.surgeon_mode ))
    {
        // if timeout period is expired, set surgeon_mode "DISENGAGED" if currently "ENGAGED"
        log_msg("Master connection timeout.  surgeon_mode -> up.\n");
        data1.surgeon_mode = SURGEON_DISENGAGED;
        lastUpdated = gTime;
        isUpdated = TRUE;
    }

    return isUpdated;
}

// Give the latest updated DS1 to the caller.
// Precondition: d1 is a pointer to allocated memory
// Postcondition: memory location of d1 contains latest DS1 Data from network/toolkit.
struct param_pass * getRcvdParams(struct param_pass* d1)
{
	///TODO: Check performance of trylock / default priority inversion scheme
    if (pthread_mutex_trylock(&data1Mutex)!=0)   //Use trylock since this function is called form rt-thread. return immediately with old values if unable to lock
        return d1;
    //pthread_mutex_lock(&data1Mutex); //Priority inversion enabled. Should force completion of other parts and enter into this section.
    memcpy(d1, &data1, sizeof(struct param_pass));

    isUpdated = 0;
    pthread_mutex_unlock(&data1Mutex);
    return d1;
}

// Reset writable copy of DS1
void updateMasterRelativeOrigin(struct device *device0)
{
    int armidx;
    struct orientation *_ori;
    btMatrix3x3 tmpmx;

    // update data1 (network position desired) to device0.position_desired (device position desired)
    //   This eliminates accumulation of deltas from network while robot is idle.
    pthread_mutex_lock(&data1Mutex);
    for (int i=0;i<NUM_MECH;i++)
    {
        data1.xd[i].x = device0->mech[i].pos_d.x;
        data1.xd[i].y = device0->mech[i].pos_d.y;
        data1.xd[i].z = device0->mech[i].pos_d.z;
        _ori = &(device0->mech[i].ori_d);
        data1.rd[i].grasp = _ori->grasp;
        for (int j=0;j<3;j++)
            for (int k=0;k<3;k++)
                data1.rd[i].R[j][k] = _ori->R[j][k];

        // Set the local quaternion orientation rep.
        armidx = USBBoards.boards[i]==GREEN_ARM_SERIAL ? 1 : 0;
        tmpmx.setValue(_ori->R[0][0], _ori->R[0][1], _ori->R[0][2],
                        _ori->R[1][0], _ori->R[1][1], _ori->R[1][2],
                        _ori->R[2][0], _ori->R[2][1], _ori->R[2][2]);
        tmpmx.getRotation(Q_ori[armidx]);

    }
    pthread_mutex_unlock(&data1Mutex);
    isUpdated = TRUE;

    return;
}

///
/// PUBLISH ROS DATA
///
#include <tf/transform_datatypes.h>
#include <raven_2/raven_state.h>
#include <raven_2/torque_command.h>
#include <raven_2/joint_command.h>
#include <raven_2_msgs/RavenCommand.h>
#include <raven_2_msgs/RavenToolCommand.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>


void publish_joints(struct robot_device*);
void publish_marker(struct robot_device*);

using namespace raven_2;
// Global publisher for raven data
ros::Publisher pub_ravenstate;
ros::Publisher joint_publisher;
ros::Publisher vis_pub1;
ros::Publisher vis_pub2;
ros::Publisher pub_command_pose;
ros::Publisher pub_omni_pose0;
ros::Publisher pub_omni_pose;


ros::Subscriber torque_sub1;
ros::Subscriber torque_sub2;
ros::Subscriber joint_sub;
ros::Subscriber cmd_sub;

ros::Publisher pub_ravenstate2;

/**
*    init_ravenstate_publishing()  - initialize publishing data to ros.
*/
int init_ravenstate_publishing(ros::NodeHandle &n){
    pub_ravenstate = n.advertise<raven_state>("ravenstate", 1000);
    joint_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    vis_pub1 = n.advertise<visualization_msgs::Marker>( "visualization_marker1", 0 );
    vis_pub2 = n.advertise<visualization_msgs::Marker>( "visualization_marker2", 0 );
    pub_command_pose = n.advertise<geometry_msgs::PoseStamped>( "command_pose", 1);
    pub_omni_pose0 = n.advertise<geometry_msgs::PoseStamped>( "master_pose_raw", 1);
    pub_omni_pose = n.advertise<geometry_msgs::PoseStamped>( "master_pose", 1);
    return 0;
}

extern device* device0ptr;
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
struct robot_device* device0ptr2;
void ravenCmdCallback(const raven_2_msgs::RavenCommand::ConstPtr& cmd) {
    static ros::Time last_pub(-1);

    ros::Time now = ros::Time::now();
    if (last_pub.toSec() < 0) { last_pub = now; return; }
    ros::Duration since_last_pub = (now-last_pub);
    last_pub = now;

	_localio_counter++;

	//printf("cmd callback!\n");
	pthread_mutex_lock(&data1Mutex);

	btVector3 p;
	btQuaternion rot;
	btMatrix3x3 rot_mx_temp;

	if (!cmd->pedal_down) {
		if (_localio_counter % PRINT_EVERY == 0) { printf("inactive!\n"); }
		data1.surgeon_mode =  SURGEON_DISENGAGED;
	} else {
		data1.surgeon_mode = SURGEON_ENGAGED;
		for (int mech_ind=0;mech_ind<NUM_MECH;mech_ind++) {
			int arm_serial = USBBoards.boards[mech_ind];
			int arm_id = arm_serial==GREEN_ARM_SERIAL ? GREEN_ARM_ID : GOLD_ARM_ID;
			if (!cmd->tool_command[arm_id].active) { continue; }
			if (_localio_counter % PRINT_EVERY == 0 || true) {
				if (arm_id == 0) {
					//printf("*********active %d!-----------------------------------\n",i);
				} else {
					//printf("*********active %d!XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX\n",i);
				}
			}

			p[0] = cmd->tool_command[arm_id].tool_pose.position.x;
			p[1] = cmd->tool_command[arm_id].tool_pose.position.y;
			p[2] = cmd->tool_command[arm_id].tool_pose.position.z;

			master_raw_position[arm_id] += p;

			rot.setX(cmd->tool_command[arm_id].tool_pose.orientation.x);
			rot.setY(cmd->tool_command[arm_id].tool_pose.orientation.y);
			rot.setZ(cmd->tool_command[arm_id].tool_pose.orientation.z);
			rot.setW(cmd->tool_command[arm_id].tool_pose.orientation.w);

			master_raw_orientation[arm_id].setRotation(rot);

			btMatrix3x3 rot_mx_temp(rot);

			p = btMatrix3x3(0,1,0,  -1,0,0,  0,0,1) * p;

			data1.xd[mech_ind].x += p.x() * MICRON_PER_M;
			data1.xd[mech_ind].y += p.y() * MICRON_PER_M;
			data1.xd[mech_ind].z += p.z() * MICRON_PER_M;

			master_position[arm_id] += p;
			rot_mx_temp = btMatrix3x3(0,1,0,  -1,0,0,  0,0,1) * rot_mx_temp * btMatrix3x3(1,0,0,  0,-1,0,  0,0,-1);
			master_orientation[arm_id] = rot_mx_temp;

			const int grasp_scale_factor = 10;
			const int graspmax = (M_PI/2 * 1000);
			const int graspmin = (-20.0 * 1000.0 DEG2RAD);
			if (arm_serial == GOLD_ARM_SERIAL) {
				data1.rd[mech_ind].grasp = saturate(data1.rd[mech_ind].grasp + grasp_scale_factor*cmd->tool_command[arm_id].grasp,graspmin,graspmax);
			} else {
				data1.rd[mech_ind].grasp = saturate(data1.rd[mech_ind].grasp + grasp_scale_factor*cmd->tool_command[arm_id].grasp,-graspmax,-graspmin);
			}

			float insertion_scale = (Z_INS_MAX_LIMIT - Z_INS_MIN_LIMIT) / 10.;
			int mechId = -1;
			for (int m=0;m<NUM_MECH;m++) {
				if (armIdFromMechType(device0ptr2->mech[m].type) == armIdFromSerial(arm_serial)) {
					mechId = m;
					break;
				}
			}
			if (arm_serial == GOLD_ARM_SERIAL) {
				float ths_offset = SHOULDER_OFFSET_GOLD; //from original URDF
				float thr_offset = TOOL_ROT_OFFSET_GOLD;
				const float th12 = THETA_12;
				const float th23 = THETA_23;

				const float ks12 = sin(th12);
				const float kc12 = cos(th12);
				const float ks23 = sin(th23);
				const float kc23 = cos(th23);

				const float dw = 0.012;

				btTransform Tw2b(btMatrix3x3(0,-1,0, 0,0,-1, 1,0,0));
				btTransform Zs = Z(device0ptr2->mech[mechId].joint[SHOULDER].jpos + ths_offset,0);
				btTransform Xu = X(th12,0);
				btTransform Ze = Z(device0ptr2->mech[mechId].joint[ELBOW].jpos,0);
				btTransform Xf = X(th23,0);
				btTransform Zr = Z(fix_angle(-device0ptr2->mech[mechId].joint[TOOL_ROT].jpos + thr_offset),0);
				btTransform Zi = Z(0,-device0ptr2->mech[mechId].joint[Z_INS].jpos);

				float ins_amt = since_last_pub.toSec() * insertion_scale * cmd->joint_velocities[arm_id].insertion;
				btVector3 z_ins_in_world = (Tw2b * Zs * Xu * Ze * Xf * Zr * Zi).getBasis() * btVector3(0,0,ins_amt);

				data1.xd[mech_ind].x += z_ins_in_world.x() * MICRON_PER_M;
				data1.xd[mech_ind].y += z_ins_in_world.y() * MICRON_PER_M;
				data1.xd[mech_ind].z += z_ins_in_world.z() * MICRON_PER_M;
			} else {

			}

			for (int j=0;j<3;j++) {
				for (int k=0;k<3;k++) {
					data1.rd[mech_ind].R[j][k] = rot_mx_temp[j][k];
				}
			}
		}
	}
	isUpdated = TRUE;

	pthread_mutex_unlock(&data1Mutex);
}


void init_subs(ros::NodeHandle &n,struct robot_device *device0) {
	device0ptr2 = device0;
	std::cout << "Initializing ros subscribers" << std::endl;
    torque_sub1 = n.subscribe("torque_cmd1", 1, torqueCallback1);
    //torque_sub2 = n.subscribe("torque_cmd2", 2, torqueCallback2);
    joint_sub = n.subscribe("joint_cmd", 1, jointCallback);
    cmd_sub = n.subscribe("raven_command", 1, ravenCmdCallback);
}


/*
* publish_ravenstate_ros()
*
*   Copy robot data over to ros message type and publish.
*/
void publish_ravenstate_ros(struct robot_device *device0,u_08 runlevel,u_08 sublevel){
    static raven_state msg_ravenstate;  // satic variables to minimize memory allocation calls
    static ros::Time last_pub(-1);
    static ros::Duration interval(0.01);

    ros::Time now = ros::Time::now();
    ros::Duration since_last_pub = (now-last_pub);
    if (last_pub.toSec() > 0 && since_last_pub < interval) { return; }
    last_pub = now;

    msg_ravenstate.dt = since_last_pub;

    publish_joints(device0);

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
    pub_ravenstate.publish(msg_ravenstate);

    geometry_msgs::PoseStamped command_pose;
    command_pose.header.stamp = ros::Time::now();
    command_pose.header.frame_id = "/0_link";

    for (int ind=0;ind<2;ind++) {
    	if (device0->mech[ind].type != GOLD_ARM) { continue; }
		command_pose.pose.position.x = ((float)device0->mech[ind].pos_d.x) / MICRON_PER_M;
		command_pose.pose.position.y = ((float)device0->mech[ind].pos_d.y) / MICRON_PER_M;
		command_pose.pose.position.z = ((float)device0->mech[ind].pos_d.z) / MICRON_PER_M;

		btQuaternion rot;
		(toBt(device0->mech[ind].ori_d.R) * btMatrix3x3(1,0,0,  0,-1,0,  0,0,-1)).getRotation(rot);

		command_pose.pose.orientation.x = rot.x();
		command_pose.pose.orientation.y = rot.y();
		command_pose.pose.orientation.z = rot.z();
		command_pose.pose.orientation.w = rot.w();
    }

    pub_command_pose.publish(command_pose);



    {
    	geometry_msgs::PoseStamped omni_pose0;
    	omni_pose0.header.stamp = ros::Time::now();
    	omni_pose0.header.frame_id = "/0_link";

    	omni_pose0.pose.position.x = master_raw_position[0].x();
    	omni_pose0.pose.position.y = master_raw_position[0].y();
    	omni_pose0.pose.position.z = master_raw_position[0].z();

    	btQuaternion q;
    	master_raw_orientation[0].getRotation(q);

    	//    omni_pose.pose.orientation.x = _teleop_rot_tracker0[0].x();
    	//    omni_pose.pose.orientation.y = _teleop_rot_tracker0[0].y();
    	//    omni_pose.pose.orientation.z = _teleop_rot_tracker0[0].z();
    	//    omni_pose.pose.orientation.w = _teleop_rot_tracker0[0].w();
    	omni_pose0.pose.orientation.x = q.x();
    	omni_pose0.pose.orientation.y = q.y();
    	omni_pose0.pose.orientation.z = q.z();
    	omni_pose0.pose.orientation.w = q.w();

    	pub_omni_pose0.publish(omni_pose0);
    }

    geometry_msgs::PoseStamped omni_pose;
    omni_pose.header.stamp = ros::Time::now();
    omni_pose.header.frame_id = "/0_link";

    omni_pose.pose.position.x = master_position[0].x();
    omni_pose.pose.position.y = master_position[0].y();
    omni_pose.pose.position.z = master_position[0].z();

    btQuaternion q;
    master_orientation[0].getRotation(q);

//    omni_pose.pose.orientation.x = _teleop_rot_tracker[0].x();
//    omni_pose.pose.orientation.y = _teleop_rot_tracker[0].y();
//    omni_pose.pose.orientation.z = _teleop_rot_tracker[0].z();
//    omni_pose.pose.orientation.w = _teleop_rot_tracker[0].w();
    omni_pose.pose.orientation.x = q.x();
    omni_pose.pose.orientation.y = q.y();
    omni_pose.pose.orientation.z = q.z();
    omni_pose.pose.orientation.w = q.w();

    pub_omni_pose.publish(omni_pose);

    //raven_state 2

}

/**
*   publish_joints() - publish joint angles to visualization
*/
void publish_joints(struct robot_device* device0){

    static int count=0;
    static ros::Time last_pub(-1);
	static ros::Duration interval(0.01);

	ros::Time now = ros::Time::now();
	ros::Duration since_last_pub = (now-last_pub);
	if (last_pub.toSec() > 0 && since_last_pub < interval) { return; }
	last_pub = now;

    //publish_marker(device0);

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
    joint_state.position.push_back(fix_angle(device0->mech[left].joint[GRASP1].jpos - device0->mech[left].joint[GRASP2].jpos,0) / 2);
    joint_state.name.push_back("grasper_L");
    joint_state.position.push_back(device0->mech[left].joint[GRASP2].jpos + device0->mech[left].joint[GRASP1].jpos);
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
    joint_state.position.push_back(fix_angle(device0->mech[right].joint[GRASP2].jpos - device0->mech[right].joint[GRASP1].jpos,0) / 2);
    joint_state.name.push_back("grasper_R");
    joint_state.position.push_back(device0->mech[right].joint[GRASP2].jpos + device0->mech[right].joint[GRASP1].jpos);
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
    joint_state.position.push_back(fix_angle(device0->mech[left].joint[GRASP1].jpos_d - device0->mech[left].joint[GRASP2].jpos_d,0) / 2);
    joint_state.name.push_back("grasper_L2");
    joint_state.position.push_back(device0->mech[left].joint[GRASP2].jpos_d + device0->mech[left].joint[GRASP1].jpos_d);
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
    joint_state.position.push_back(fix_angle(device0->mech[right].joint[GRASP2].jpos_d - device0->mech[right].joint[GRASP1].jpos_d,0) / 2);
    joint_state.name.push_back("grasper_R2");
    joint_state.position.push_back(device0->mech[right].joint[GRASP2].jpos_d + device0->mech[right].joint[GRASP1].jpos_d);

    //Publish the joint states
    joint_publisher.publish(joint_state);

}


void publish_marker(struct robot_device* device0)
{
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



//
//    marker1.type = visualization_msgs::Marker::LINE_LIST;
//    marker1.action = visualization_msgs::Marker::ADD;   // Set the marker action.  Options are ADD and DELETE
//    marker1.header.stamp = ros::Time::now();
//    marker1.ns = "RCM_marker";
//    marker1.lifetime = ros::Duration();
//    // Set the scale of the marker -- 1x1x1 here means 1m on a side
//    marker1.scale.x = 0.020;
//    marker1.scale.y = 0.020;
//    marker1.scale.z = 0.020;
//
//    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
//    marker1.header.frame_id = "/base_link_L";
//    marker1.id = 0;
//
//    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
//
//    p.x = device0->mech[left].pos.x/1e6;
//    p.y = device0->mech[left].pos.y/1e6;
//    p.z = device0->mech[left].pos.z/1e6;
//    marker1.points.push_back(p);
//    px=p;
//    px.x+=0.3;
//    marker1.points.push_back(px);
//    marker1.points.push_back(p);
//    py=p;
//    py.y+=0.3;
//    marker1.points.push_back(py);
//    marker1.points.push_back(p);
//    pz=p;
//    pz.z+=0.3;
//    marker1.points.push_back(pz);
//
////    marker1.pose.position.x = device0->mech[left].pos.x/1e6;
////    marker1.pose.position.y = device0->mech[left].pos.y/1e6;
////    marker1.pose.position.z = device0->mech[left].pos.z/1e6;
//
//    // Get quaternion representation of rotation
//    struct orientation* _ori = &(device0->mech[left].ori);
//    btMatrix3x3 xform (_ori->R[0][0], _ori->R[0][1], _ori->R[0][2],
//                        _ori->R[1][0], _ori->R[1][1], _ori->R[1][2],
//                        _ori->R[2][0], _ori->R[2][1], _ori->R[2][2]);
//    btQuaternion bq;
//    xform.getRotation(bq);
//    marker1.pose.orientation.x = bq.getX();
//    marker1.pose.orientation.y = bq.getY();
//    marker1.pose.orientation.z = bq.getZ();
//    marker1.pose.orientation.w = bq.getW();
//
//    // Set the color -- be sure to set alpha to something non-zero!
//    marker1.color.r = 1.0f;
//    marker1.color.g = 0.0f;
//    marker1.color.b = 0.0f;
//    marker1.color.a = 1.0;
//    // Publish the marker
//    vis_pub1.publish(marker1);




// ------------------------- //
// - Recieve toolkit data  - //
////-------------------------- //
//int recieveToolkit(int fifo) //was unsigned int fifo
//{
//    struct param_pass ds1Recv;
//    int ret;
//
//    ret=read(fifo, &ds1Recv, sizeof(struct param_pass));
//    if ( ret != sizeof( struct param_pass) )
//    {
//        log_msg("Bogus toolkit data on FIFO Rcv.  Resetting fifo.\n");
//        return -1;
//    }
//
//    updateDS1(&ds1Recv);
//
//    return 0;
//}


// updateDS1()
//
//   update DS1 from toolkit update.
//
//void updateDS1(struct param_pass *d1)
//{
//    int i;
//
//    pthread_mutex_lock(&data1Mutex);
//
//    // on entrance into RL=0, reset pos_d.
//    for (i=0;i<NUM_MECH;i++)
//    {
//        if (d1->runlevel==0)
//        {
//            data1.xd[i].x = 0;
//            data1.xd[i].y = 0;
//            data1.xd[i].z = 0;
//            data1.rd[i].yaw   =0;
//            data1.rd[i].pitch =0;
//            data1.rd[i].roll  =0;
//            data1.rd[i].grasp =0;
//        }
//        // copy position data to new DS1 (rec'd from toolkit)
//        //  The objective is to keep current position/orientation,
//        //  while receiving commands (gains, trajectories, etc) from toolkit
//        d1->xd[i].x = data1.xd[i].x;
//        d1->xd[i].y = data1.xd[i].y;
//        d1->xd[i].z = data1.xd[i].z;
//        d1->rd[i].yaw   = data1.rd[i].yaw;
//        d1->rd[i].pitch = data1.rd[i].pitch;
//        d1->rd[i].roll  = data1.rd[i].roll;
//        d1->rd[i].grasp  = data1.rd[i].grasp;
//    }
//    d1->surgeon_mode = data1.surgeon_mode;
//
//    // copy new DS to local store
//    memcpy(&data1, d1, sizeof(struct param_pass));
//    isUpdated = TRUE;
//
//    pthread_mutex_unlock(&data1Mutex);
//}
