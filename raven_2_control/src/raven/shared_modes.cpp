/*
 * master_mode.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: benk
 */

#include "shared_modes.h"
#include "DS0.h"
#include "log.h"

#include <iostream>

#include <raven/state/runlevel.h>

#include <boost/thread/mutex.hpp>

extern struct robot_device device0;

#define STRINGIFY(s) STRINGIFY_HELPER(s)
#define STRINGIFY_HELPER(s) #s

/*********************** MASTER MODE *********************************/

MasterMode masterMode = "";
std::set<MasterMode> masterModeConflictSet;
boost::mutex masterModeMutex;

std::string masterModeToString(const MasterMode& mode) {
	if (masterMode.empty()) {
		return "NONE";
	}
	return mode;
}

std::string getMasterModeString() {
	return masterModeToString(getMasterMode());
}

MasterMode getMasterMode() {
	boost::mutex::scoped_lock _lock(masterModeMutex);
	return masterMode;
}

bool masterModeIsNone(const MasterMode& mode) {
	return mode.empty();
}

bool checkMasterMode(const MasterMode& mode) {
	if (mode.empty())
	{
		return false;
#ifdef USE_NEW_RUNLEVEL
	} else if (!RunLevel::hasHomed()) {
#else
	} else if (device0.runlevel == RL_E_STOP || device0.runlevel == RL_INIT) {
#endif
		return false;
	}
	bool succeeded = false;
	boost::mutex::scoped_lock _lock(masterModeMutex);
	if (masterMode == mode) {
		succeeded = true;
	} else if (masterMode.empty()) {
		masterMode = mode;
		succeeded = true;
		log_msg("Entering master mode: %s\n",masterMode.c_str());
	} else {
		masterModeConflictSet.insert(mode);
	}
	return succeeded;
}

bool getMasterModeConflicts(MasterMode& mode, std::set<MasterMode>& conflicts) {
	bool conflicted;
	boost::mutex::scoped_lock _lock(masterModeMutex);
	mode = masterMode;
	conflicted = !masterModeConflictSet.empty();
	conflicts.insert(masterModeConflictSet.begin(),masterModeConflictSet.end());
	masterModeConflictSet.clear();
	return conflicted;
}

bool resetMasterMode() {
	bool succeeded = false;
	boost::mutex::scoped_lock _lock(masterModeMutex);
	masterMode = "";
	masterModeConflictSet.clear();
	succeeded = true;
	return succeeded;
}

/*********************** CONTROLLER MODE *********************************/

t_controlmode controlMode = no_control;
std::set<t_controlmode> controlModeConflictSet;
boost::mutex controlModeMutex;

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
	CASE_CONTROL_MODE(trajectory_control);
	default:
		std::cerr << "Unknown control mode " << mode << std::endl;
		return "no_control";
	}
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
	boost::mutex::scoped_lock _lock(controlModeMutex);
	//change controllers ok if runlevel is pedal up
	if (controlMode == mode) {
		succeeded = true;
	} else if (controlMode == no_control
			|| (!RunLevel::get().isPedalDown() && !RunLevel::get().isInit())) {
		controlMode = mode;
		succeeded = true;
		log_msg("Entering control mode: %s\n",controlModeToString(mode).c_str());
	} else {
		controlModeConflictSet.insert(mode);
	}
	return succeeded;
}

bool getControlModeConflicts(t_controlmode& mode, std::set<t_controlmode>& conflicts) {
	bool conflicted;
	boost::mutex::scoped_lock _lock(controlModeMutex);
	mode = controlMode;
	conflicted = !controlModeConflictSet.empty();
	conflicts.insert(controlModeConflictSet.begin(),controlModeConflictSet.end());
	controlModeConflictSet.clear();
	return conflicted;
}

bool setControlMode(t_controlmode new_mode) {
	t_controlmode old_mode;
	boost::mutex::scoped_lock _lock(controlModeMutex);
	old_mode = controlMode;
	controlMode = new_mode;
	if (old_mode != new_mode) {
		controlModeConflictSet.clear();
	}
	return new_mode != old_mode;
}
