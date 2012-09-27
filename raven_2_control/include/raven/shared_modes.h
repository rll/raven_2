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

/*********************** MASTER MODE *********************************/

namespace MasterMode {
	enum Enum { NONE, NETWORK, ROS_RAVEN_CMD, ROS_POSE, ROS_JOINT_POSITION, ROS_JOINT_VELOCITY, ROS_JOINT_TORQUE, ROS_JOINT_TRAJECTORY };
}

MasterMode::Enum getMasterMode();

std::string masterModeToString(MasterMode::Enum mode);
std::string getMasterModeString();

bool checkMasterMode(MasterMode::Enum mode);

bool getMasterModeConflicts(MasterMode::Enum& mode, std::set<MasterMode::Enum>& conflicts);

bool resetMasterMode();

/*********************** CONTROL MODE *********************************/

t_controlmode getControlMode();

std::string controlModeToString(t_controlmode mode);
std::string getControlModeString();

bool checkControlMode(t_controlmode mode);

bool getControlModeConflicts(t_controlmode& mode, std::set<t_controlmode>& conflicts);

bool resetControlMode();

#endif /* MASTER_MODE_H_ */
