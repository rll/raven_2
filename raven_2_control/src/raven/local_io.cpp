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
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <iostream>

#include "log.h"
#include "local_io.h"
#include "utils.h"
#include "mapping.h"
#include "itp_teleoperation.h"
#include "kinematics_defines.h"

extern bool disable_arm_id[2];
extern int NUM_MECH;
extern USBStruct USBBoards;
extern unsigned long int gTime;

#define GOLD_ARM_TELEOP_ID GOLD_ARM_ID
#define GREEN_ARM_TELEOP_ID GREEN_ARM_ID

static param_pass data1;
pthread_mutexattr_t data1MutexAttr;
pthread_mutex_t data1Mutex;

btVector3 master_raw_position[2];
btVector3 master_position[2];
btMatrix3x3 master_raw_orientation[2];
btMatrix3x3 master_orientation[2];

static int _localio_counter;

#define DISABLE_ALL_PRINTING true
static const int PRINT_EVERY_PEDAL_UP   = 1000000000;
static const int PRINT_EVERY_PEDAL_DOWN = 1000;
static int PRINT_EVERY = PRINT_EVERY_PEDAL_DOWN;
#define PRINT (_localio_counter % PRINT_EVERY == 0 && !(DISABLE_ALL_PRINTING))

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
	static long int last_call = -1;

	unsigned long int now = gTime;
	if (last_call < 0) { last_call = now; return; }
	float since_last_call = ((float)(now-last_call)) / 1000.;
	last_call = now;

	pthread_mutex_lock(&data1Mutex);
	_localio_counter++;

	if (PRINT) {
		//print_u_struct_pos(t,0);
		//print_u_struct_pos(t,1);
		//std::cout << "grasp " << t->grasp[0] << " " << data1.rd[0].grasp << " | " << t->grasp[1] << " " << data1.rd[1].grasp << " | " << t->surgeon_mode << std::endl;
	}

	btVector3 p_raw, p;
	int mechnum, teleopArmId, armidx, armserial;
	btQuaternion rot_raw, rot;
    btMatrix3x3 rot_mx_temp;

    for (mechnum=0;mechnum<NUM_MECH;mechnum++)
    {
        armserial = USBBoards.boards[mechnum]==GREEN_ARM_SERIAL ? GREEN_ARM_SERIAL : GOLD_ARM_SERIAL;
        teleopArmId    = USBBoards.boards[mechnum]==GREEN_ARM_SERIAL ? GREEN_ARM_TELEOP_ID : GOLD_ARM_TELEOP_ID;
        armidx    = USBBoards.boards[mechnum]==GREEN_ARM_SERIAL ? GREEN_ARM_ID : GOLD_ARM_ID;

        if (PRINT && !disable_arm_id[armidx]) {
        	printf("teleop %d -------------------------------\n",armidx);
        	print_u_struct(t,teleopArmId);
        }

        // apply mapping to teleop data
        p_raw[0] = t->delx[teleopArmId];
        p_raw[1] = t->dely[teleopArmId];
        p_raw[2] = t->delz[teleopArmId];

        //set local quaternion from teleop quaternion data
        rot_raw.setX( t->Qx[teleopArmId] );
        rot_raw.setY( t->Qy[teleopArmId] );
        rot_raw.setZ( t->Qz[teleopArmId] );
        rot_raw.setW( t->Qw[teleopArmId] );

        master_raw_position[armidx] += p_raw / MICRON_PER_M;
        master_raw_orientation[armidx].setRotation(rot_raw);

        //print teleop pose
        if (PRINT && !disable_arm_id[armidx]) {
        	tb_angles2 angles = tb_angles2(rot_raw);
        	tb_angles2 angles1 = tb_angles2(master_raw_orientation[armidx]);
        	printf("teleop %d raw (% 1.3f,% 1.3f,% 1.3f)  ypr (% 3.1f,% 3.1f,% 3.1f) (% 3.1f,% 3.1f,% 3.1f)\n",armidx,
        			master_raw_position[armidx].x(),master_raw_position[armidx].y(),master_raw_position[armidx].z(),
        			angles.yaw_deg,angles.pitch_deg,angles.roll_deg,
        			angles1.yaw_deg,angles1.pitch_deg,angles1.roll_deg);
        }


        p = p_raw;
        rot = rot_raw;
        fromITP(p, rot, armserial);

        if (!t->surgeon_mode) {
        	//master_orientation[armidx].getRotation(rot);
        }

        master_orientation[armidx].setRotation(rot);

        data1.xd[mechnum].x += p.x();
        data1.xd[mechnum].y += p.y();
        data1.xd[mechnum].z += p.z();

        master_position[armidx] += p / MICRON_PER_M;

        if (PRINT && !disable_arm_id[armidx]) {
        	tb_angles2 angles = tb_angles2(master_orientation[armidx]);
        	printf("teleop %d     (% 1.3f,% 1.3f,% 1.3f)  ypr (% 3.1f,% 3.1f,% 3.1f)\n",armidx,
        	        			master_position[armidx].x(),master_position[armidx].y(),master_position[armidx].z(),
        	        			angles.yaw_deg,angles.pitch_deg,angles.roll_deg);
        }

        float grasp_scale_factor = 500 * since_last_call;
        if (grasp_scale_factor < 1) {
        	grasp_scale_factor = 1;
        }
        const int graspmax = (M_PI/2 * 1000);
        const int graspmin = (-20.0 * 1000.0 DEG2RAD);
        if (armserial == GOLD_ARM_SERIAL) {
        	data1.rd[mechnum].grasp = saturate(data1.rd[mechnum].grasp + grasp_scale_factor*t->grasp[teleopArmId],graspmin,graspmax);
        } else {
        	data1.rd[mechnum].grasp = saturate(data1.rd[mechnum].grasp - grasp_scale_factor*t->grasp[teleopArmId],-graspmax,-graspmin);
        }

        rot_mx_temp.setRotation(rot);

        // Set rotation command
        for (int j=0;j<3;j++)
            for (int k=0;k<3;k++)
                data1.rd[mechnum].R[j][k] = rot_mx_temp[j][k];
    }

//    log_msg("updated d1.xd to: (%d,%d,%d)/(%d,%d,%d)",
//           data1.xd[0].x, data1.xd[0].y, data1.xd[0].z,
//           data1.xd[1].x, data1.xd[1].y, data1.xd[1].z);

    data1.surgeon_mode = t->surgeon_mode;
    pthread_mutex_unlock(&data1Mutex);
}

void writeUpdate(struct param_pass* data_in) {
	pthread_mutex_lock(&data1Mutex);
	memcpy(&data1, data_in, sizeof(struct param_pass));

	isUpdated = TRUE;
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

