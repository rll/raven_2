/*
 * master_mode.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: benk
 */

#include "shared_modes.h"
#include "DS0.h"

#include <iostream>

extern struct robot_device device0;

#define STRINGIFY(s) STRINGIFY_HELPER(s)
#define STRINGIFY_HELPER(s) #s

/*********************** MASTER MODE *********************************/

MasterMode::Enum masterMode = MasterMode::NONE;
std::set<MasterMode::Enum> masterModeConflictSet;
pthread_mutexattr_t masterModeMutexAttr;
pthread_mutex_t masterModeMutex;

MasterMode::Enum getMasterMode() {
	return masterMode;
}

#define CASE_MASTER_MODE(m) case m: return std::string(STRINGIFY(m)).substr(12)
std::string masterModeToString(MasterMode::Enum mode) {
	switch (mode) {
	CASE_MASTER_MODE(MasterMode::NONE);
	CASE_MASTER_MODE(MasterMode::NETWORK);
	CASE_MASTER_MODE(MasterMode::ROS_RAVEN_CMD);
	CASE_MASTER_MODE(MasterMode::ROS_POSE);
	CASE_MASTER_MODE(MasterMode::ROS_JOINT_POSITION);
	CASE_MASTER_MODE(MasterMode::ROS_JOINT_VELOCITY);
	CASE_MASTER_MODE(MasterMode::ROS_JOINT_TORQUE);
	CASE_MASTER_MODE(MasterMode::ROS_JOINT_TRAJECTORY);
	}
	std::cerr << "Unknown master mode " << mode << std::endl;
	return "MasterMode::NONE";
}

std::string getMasterModeString() {
	MasterMode::Enum mode = getMasterMode();
	return masterModeToString(mode);
}

bool checkMasterMode(MasterMode::Enum mode) {
	if (mode == MasterMode::NONE) {
		return false;
	} else if (device0.runlevel == RL_E_STOP || device0.runlevel == RL_INIT) {
		return false;
	}
	bool succeeded = false;
	pthread_mutex_lock(&masterModeMutex);
	if (masterMode == mode || masterMode == MasterMode::NONE) {
		masterMode = mode;
		succeeded = true;
	} else {
		masterModeConflictSet.insert(mode);
	}
	pthread_mutex_unlock(&masterModeMutex);
	return succeeded;
}

bool getMasterModeConflicts(MasterMode::Enum& mode, std::set<MasterMode::Enum>& conflicts) {
	bool conflicted;
	pthread_mutex_lock(&masterModeMutex);
	mode = masterMode;
	conflicted = !masterModeConflictSet.empty();
	conflicts.insert(masterModeConflictSet.begin(),masterModeConflictSet.end());
	masterModeConflictSet.clear();
	pthread_mutex_unlock(&masterModeMutex);
	return conflicted;
}

bool resetMasterMode() {
	bool succeeded = false;
	pthread_mutex_lock(&masterModeMutex);
	masterMode = MasterMode::NONE;
	masterModeConflictSet.clear();
	succeeded = true;
	pthread_mutex_unlock(&masterModeMutex);
	return succeeded;
}

/*********************** CONTROLLER MODE *********************************/

t_controlmode controlMode = no_control;
std::set<t_controlmode> controlModeConflictSet;
pthread_mutexattr_t controlModeMutexAttr;
pthread_mutex_t controlModeMutex;

t_controlmode getControlMode() {
	return controlMode;
}

#define CASE_CONTROL_MODE(m) case m: return std::string(STRINGIFY(m))
std::string controlModeToString(t_controlmode mode) {
	switch (mode) {
	CASE_CONTROL_MODE(no_control);
	CASE_CONTROL_MODE(end_effector_control);
	CASE_CONTROL_MODE(joint_velocity_control);
	CASE_CONTROL_MODE(apply_arbitrary_torque);
	CASE_CONTROL_MODE(homing_mode);
	CASE_CONTROL_MODE(motor_pd_control);
	CASE_CONTROL_MODE(cartesian_space_control);
	CASE_CONTROL_MODE(multi_dof_sinusoid);
	CASE_CONTROL_MODE(joint_torque_control);
	}
	std::cerr << "Unknown control mode " << mode << std::endl;
	return "no_control";
}

std::string getControlModeString() {
	t_controlmode mode = getControlMode();
	return controlModeToString(mode);
}

bool checkControlMode(t_controlmode mode) {
	if (mode == no_control) {
		return false;
	}
	bool succeeded = false;
	pthread_mutex_lock(&controlModeMutex);
	if (controlMode == mode || controlMode == no_control) {
		controlMode = mode;
		succeeded = true;
	} else {
		controlModeConflictSet.insert(mode);
	}
	pthread_mutex_unlock(&controlModeMutex);
	return succeeded;
}

bool getControlModeConflicts(t_controlmode& mode, std::set<t_controlmode>& conflicts) {
	bool conflicted;
	pthread_mutex_lock(&controlModeMutex);
	mode = controlMode;
	conflicted = !controlModeConflictSet.empty();
	conflicts.insert(controlModeConflictSet.begin(),controlModeConflictSet.end());
	controlModeConflictSet.clear();
	pthread_mutex_unlock(&controlModeMutex);
	return conflicted;
}

bool resetControlMode() {
	bool succeeded = false;
	pthread_mutex_lock(&controlModeMutex);
	controlMode = no_control;
	controlModeConflictSet.clear();
	succeeded = true;
	pthread_mutex_unlock(&controlModeMutex);
	return succeeded;
}
