/*
 * shared_modes.h
 *
 *  Created on: Sep 26, 2012
 *      Author: benk
 */

#ifndef SHARED_MODES_H_
#define SHARED_MODES_H_

#include <string>
#include <set>

#include "struct.h"
#include <raven/util/enum.h>

/*********************** MASTER MODE *********************************/

BOOST_ENUM(MasterMode,(NONE)(NETWORK)(ROS_RAVEN_CMD)(ROS_POSE)(ROS_TRAJECTORY)(ROS_JOINT_POSITION)(ROS_JOINT_VELOCITY)(ROS_JOINT_TORQUE)(ROS_JOINT_TRAJECTORY))

MasterMode getMasterMode();

bool checkMasterMode(MasterMode mode);

bool getMasterModeConflicts(MasterMode& mode, std::set<MasterMode>& conflicts);

bool resetMasterMode();

/*********************** CONTROL MODE *********************************/

t_controlmode getControlMode();

std::string controlModeToString(t_controlmode mode);
std::string getControlModeString();

bool checkControlMode(t_controlmode mode);

bool getControlModeConflicts(t_controlmode& mode, std::set<t_controlmode>& conflicts);

bool setControlMode(t_controlmode new_mode);

inline bool resetControlMode() { return setControlMode(no_control); }

#endif /* MASTER_MODE_H_ */
