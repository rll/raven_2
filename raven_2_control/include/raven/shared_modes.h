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

#define MASTER_MODE_STRING

#ifdef MASTER_MODE_STRING
typedef std::string MasterMode;
#else
BOOST_ENUM(MasterMode,(NONE)(NETWORK)(ROS_RAVEN_CMD)(ROS_POSE)(ROS_TRAJECTORY)(ROS_JOINT_POSITION)(ROS_JOINT_VELOCITY)(ROS_JOINT_TORQUE)(ROS_JOINT_TRAJECTORY))
#endif

MasterMode getMasterMode();

std::string masterModeToString(const MasterMode& mode);
std::string getMasterModeString();

bool masterModeIsNone(const MasterMode& mode);

bool checkMasterMode(const MasterMode& mode);

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
