/*
 * motor_pid.cpp
 *
 *  Created on: Oct 9, 2012
 *      Author: benk
 */


#include <raven/control/controllers/motor_position_pid.h>

#include <algorithm>
#include <string>

#include <iostream>
#include "log.h"

ControllerStatePtr
MotorPositionPID::internalApplyControl(DevicePtr device) {
	static MotorList motorsForUpdate;
	TRACER_ENTER("MotorPositionPID::internalApplyControl");

	MotorPositionPIDStatePtr lastState = getLastState<MotorPositionPIDState>();
	MotorPositionPIDStatePtr state = cloneLastState<MotorPositionPIDState>(device);
	if (!state) {
		state.reset(new MotorPositionPIDState(device));
	}

	Eigen::VectorXf pos = device->motorPositionVector();
	Eigen::VectorXf vel = device->motorVelocityVector();

	Eigen::VectorXf pos_d;
	Eigen::VectorXf vel_d;

	MotorPositionInputPtr posInput;
	MotorVelocityInputPtr velInput;
	MultipleControlInputPtr multiInput;

	if (getInput(posInput)) {
		pos_d = posInput->values();
	} else if (getInput(velInput)){
		vel_d = velInput->values();
	} else if (getInput(multiInput)) {
		if (multiInput->getInput("position",posInput)) {
			pos_d = posInput->values();
		}
		if (multiInput->getInput("velocity",velInput)) {
			vel_d = velInput->values();
		}
	} else {
		OldControlInputPtr oldControlInput = ControlInput::getOldControlInput();
		pos_d = oldControlInput->motorPositionVector();
		vel_d = oldControlInput->motorVelocityVector();
	}

	if (pos_d.rows() == 0) {
		if (vel_d.rows() == 0) {
			//TODO: nothing
		} else {
			pos_d.setZero(vel_d.rows());
		}
	} else if (vel_d.rows() == 0) {
		vel_d.setZero(pos_d.rows());
	}

	Eigen::VectorXf pos_err = pos_d - pos;
	Eigen::VectorXf vel_err = vel_d - vel;

	Eigen::VectorXf int_err(pos_err.rows());
	if (!lastState || reset_) {
		int_err.setZero();
	} else {
		int_err = lastState->positionErrorIntegral + pos_err * (lastState->device->timestamp() - device->timestamp()).toSec();
	}
	state->positionErrorIntegral = int_err;

	Eigen::VectorXf p_term = KP_.cwiseProduct(pos_err);
	Eigen::VectorXf i_term = KI_.cwiseProduct(int_err);
	Eigen::VectorXf d_term = KD_.cwiseProduct(vel_err);
	log_msg_throttle(0.25,"pos err %f",pos_err(0));

	Eigen::VectorXf values1 = p_term + i_term;
	Eigen::VectorXf values2 = i_term + d_term;
	Eigen::VectorXf values3 = p_term + d_term;

	Eigen::VectorXf values = p_term + i_term + d_term;

	float tmp = -1;
	size_t begin_ind = 0;
	for (size_t i=0;i<device->arms().size();i++) {
		device->arm(i)->motorFilter()->getMotorsForUpdate(motorsForUpdate);
		Eigen::VectorXf armVals = values.segment(begin_ind,motorsForUpdate.size());

		for (size_t j=0;j<motorsForUpdate.size();j++) {
			float val = armVals(j);
			if (val!=0) { tmp = val; }
			motorsForUpdate[j]->setTorque(val);
		}

		begin_ind += motorsForUpdate.size();
	}
	log_msg_throttle(0.25,"tmp: %f",tmp);

	TRACER_LEAVE();
	return state;
}

MotorPositionPID::MotorPositionPID() : Controller(1), reset_(false) {
	static DevicePtr dev;
	size_t totalSize = 0;
	FOREACH_ARM_IN_CURRENT_DEVICE(arm,dev) {
		ArmGains armGains;
		armGains.id = arm->id();

		std::string armName = arm->name();
		std::transform(armName.begin(), armName.end(), armName.begin(), ::tolower);

		std::vector<double> kp_gains = getParameterVector<double>("gains_kp_" + armName);
		if (kp_gains.empty()) {
			kp_gains = getParameterVector<double>("gains_kp");
		}
		if (kp_gains.empty()) {
			kp_gains = getParameterVector<double>("gains_" + armName + "_kp");
			kp_gains.erase(kp_gains.begin()+3);
		}
		std::vector<double> ki_gains = getParameterVector<double>("gains_ki_" + armName);
		if (ki_gains.empty()) {
			ki_gains = getParameterVector<double>("gains_ki");
		}
		if (ki_gains.empty()) {
			ki_gains = getParameterVector<double>("gains_" + armName + "_ki");
			ki_gains.erase(ki_gains.begin()+3);
		}
		std::vector<double> kd_gains = getParameterVector<double>("gains_kd_" + armName);
		if (kd_gains.empty()) {
			kd_gains = getParameterVector<double>("gains_kd");
		}
		if (kd_gains.empty()) {
			kd_gains = getParameterVector<double>("gains_" + armName + "_kd");
			kd_gains.erase(kd_gains.begin()+3);
		}
		if (kp_gains.empty() || ki_gains.empty() || kd_gains.empty()) {
			log_err("Gains not found!");
			return;
		}

		for (size_t i=0;i<kp_gains.size();i++) {
			Gains g;
			g.KP = kp_gains[i];
			g.KI = ki_gains[i];
			g.KD = kd_gains[i];
			armGains.gains.push_back(g);
		}
		totalSize += kp_gains.size();
		gains_.push_back(armGains);
	}

	KP_.resize(totalSize);
	KI_.resize(totalSize);
	KD_.resize(totalSize);

	size_t startInd = 0;
	for (size_t i=0;i<gains_.size();i++) {
		for (size_t j=0;j<gains_[i].gains.size();j++) {
			KP_[startInd + j] = gains_[i].gains[j].KP;
			KI_[startInd + j] = gains_[i].gains[j].KI;
			KD_[startInd + j] = gains_[i].gains[j].KD;
		}
		startInd += gains_[i].gains.size();
	}
}

