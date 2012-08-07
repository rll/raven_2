/*
 * pulley_board_init.h
 *
 */

//Include files
#include <ros/ros.h>

#include "struct.h" /*Includes DS0, DS1, DOF_type*/
#include "fwd_kinematics.h"
#include "fwd_cable_coupling.h"
#include "motor.h"
#include "USB_init.h"

#define JOINT_ENABLED    1

//
#define ENC_MAX_NOT_SET  -10000
#define ENC_MIN_NOT_SET   10000

void initRobotData(struct device *device0, int runlevel, struct param_pass *currParams);

/// Structure initialization
void initDOFs(struct device *device0);

/// Get ravengains from ROS parameter server.
int init_ravengains(ros::NodeHandle n, struct device *device0);

/// set the starting xyz coordinate (pos_d = pos)
void setStartXYZ(struct device *device0);

