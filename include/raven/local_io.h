/*********
 *
 * File: local_io.h
 *
 *  Functions for interfacing with FIFOs and local system.
 */

#ifndef LOCAL_IO_H
#define LOCAL_IO_H

#include <ros/ros.h>

#include "struct.h"
#include "defines.h"
#include "fifo.h"
#include "USB_init.h"

int initLocalioData(void);

// update controller state w/ toolkit input
void teleopIntoDS1(struct u_struct*);

// fifo handler to recv command data
int recieveUserspace(void *u,int size);

// Check: have any command updates happened?
int checkLocalUpdates(void);

// Return current parameter-update set
struct param_pass * getRcvdParams(struct param_pass*);

void updateMasterRelativeOrigin(struct device *device0);

int init_ravenstate_publishing(ros::NodeHandle &n);
void publish_ravenstate_ros(struct robot_device *dev,u_08 runlevel,u_08 sublevel);

void init_subs(ros::NodeHandle &n);
void updateJoints(mechanism* mech);

#endif
