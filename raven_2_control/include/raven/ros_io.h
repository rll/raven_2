/*
 * ros_io.h
 *
 *  Created on: Aug 6, 2012
 *      Author: benk
 */

#ifndef ROS_IO_H_
#define ROS_IO_H_

#include <ros/ros.h>

#include "struct.h"
#include "defines.h"
#include "USB_init.h"
#include "local_io.h"

void init_ros_topics(ros::NodeHandle &n,struct robot_device* device0);
void publish_ros(struct robot_device* dev,param_pass currParams);

#include <raven_2_msgs/RavenCommand.h>

void processRavenCmd(const raven_2_msgs::RavenCommand& cmd);
void cmd_callback(const raven_2_msgs::RavenCommand& cmd); //FIXME: remove


#endif /* ROS_IO_H_ */
