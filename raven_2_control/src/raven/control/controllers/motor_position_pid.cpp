/*
 * motor_pid.cpp
 *
 *  Created on: Oct 9, 2012
 *      Author: benk
 */


#include <raven/control/controllers/motor_position_pid.h>

#include <boost/algorithm/string.hpp>
#include <string>

#include <raven/util/stringify.h>

#include <iostream>
#include "log.h"

ControllerStatePtr
MotorPositionPID::internalApplyControl(DevicePtr device) {
	static MotorList motorsForUpdate;
	TRACER_ENTER("MotorPositionPID::internalApplyControl()");

	MotorPositionPIDStatePtr lastState = getLastState<MotorPositionPIDState>();
	MotorPositionPIDStatePtr state = cloneLastState<MotorPositionPIDState>(device);
	if (!state) {
		state.reset(new MotorPositionPIDState(device));
	}

	Eigen::VectorXf pos = device->motorPositionVector();
	Eigen::VectorXf vel = device->motorVelocityVector();

	Eigen::VectorXf pos_d;
	Eigen::VectorXi pos_d_armIds;
	Eigen::VectorXi pos_d_motorIds;
	Eigen::VectorXf vel_d;
	Eigen::VectorXi vel_d_armIds;
	Eigen::VectorXi vel_d_motorIds;

	MotorPositionInputPtr posInput;
	MotorVelocityInputPtr velInput;
	DualControlInput<MotorPositionInput,MotorVelocityInput>::Ptr dualInput;
	MultipleControlInputPtr multiInput;

	Arm::IdSet ids;

	bool using_old = false;

	if (getInput(posInput)) {
		posInput->fullVector(pos_d,pos_d_armIds,pos_d_motorIds);
		Arm::idSetAdd(ids,posInput->ids());
	} else if (getInput(velInput)){
		velInput->fullVector(vel_d,vel_d_armIds,vel_d_motorIds);
		Arm::idSetAdd(ids,velInput->ids());
	} else if (getInput(dualInput)) {
		dualInput->first()->fullVector(pos_d,pos_d_armIds,pos_d_motorIds);
		dualInput->second()->fullVector(vel_d,vel_d_armIds,vel_d_motorIds);
		Arm::idSetAdd(ids,dualInput->armIds());
	} else if (getInput(multiInput)) {
		if (multiInput->getInput("position",posInput)) {
			posInput->fullVector(pos_d,pos_d_armIds,pos_d_motorIds);
			Arm::idSetAdd(ids,posInput->ids());
		}
		if (multiInput->getInput("velocity",velInput)) {
			velInput->fullVector(vel_d,vel_d_armIds,vel_d_motorIds);
			Arm::idSetAdd(ids,velInput->ids());
		}
	} else {
		using_old = true;
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
	if (!lastState || getResetState()) {
		int_err.setZero();
	} else {
		int_err = lastState->positionErrorIntegral + pos_err * (lastState->device->timestamp() - device->timestamp()).toSec();
		if (!using_old) {
			for (int i=0;i<int_err.rows();i++) {
				if (pos_d_armIds(i) < 0) {
					int_err(i) = lastState->positionErrorIntegral(i);
				}
			}
		}
	}
	state->positionErrorIntegral = int_err;

	Eigen::VectorXf p_term = KP_.cwiseProduct(pos_err);
	Eigen::VectorXf i_term = KI_.cwiseProduct(int_err);
	Eigen::VectorXf d_term = KD_.cwiseProduct(vel_err);
	//log_msg_throttle(0.25,"pos err %f",pos_err(0));

	Eigen::VectorXf values1 = p_term + i_term;
	Eigen::VectorXf values2 = i_term + d_term;
	Eigen::VectorXf values3 = p_term + d_term;

	Eigen::VectorXf values = p_term + i_term + d_term;

	float tmp = -1;
	size_t begin_ind = 0;
	for (size_t i=0;i<device->arms().size();i++) {
		device->arm(i)->controlMotorFilter()->getMotorsForUpdate(motorsForUpdate);
		Eigen::VectorXf armVals = values.segment(begin_ind,motorsForUpdate.size());

		for (size_t j=0;j<motorsForUpdate.size();j++) {
			float val = armVals(j);
			if (val!=0) { tmp = val; }
			motorsForUpdate[j]->setTorque(val);
		}

		begin_ind += motorsForUpdate.size();
	}
	//log_msg_throttle(0.25,"tmp: %f",tmp);

	TRACER_LEAVE();
	return state;
}

#define GET_GAIN(gainType) \
		std::vector<double> k##gainType##_gains = getParameterVector<double>(param_base + "/k" + STRINGIFY(gainType) + "/" + armName); \
		if (k##gainType##_gains.empty()) { \
			k##gainType##_gains = getParameterVector<double>(param_base + "/k" + STRINGIFY(gainType) + "/" + armType); \
		} \
		if (k##gainType##_gains.empty()) { \
			k##gainType##_gains = getParameterVector<double>(param_base + "/k" + STRINGIFY(gainType)); \
		} \
		if (k##gainType##_gains.empty()) { \
			k##gainType##_gains = getParameterVector<double>("gains_" + armType + "_kp"); \
			if (!k##gainType##_gains.empty()) { \
				k##gainType##_gains.erase(k##gainType##_gains.begin()+3); \
			} \
		} \
		if (k##gainType##_gains.empty()) { \
			log_err("Gains not found!"); \
		}

MotorPositionPID::MotorPositionPID() : Controller(1) {
	TRACER_ENTER_SCOPE("MotorPositionPID::MotorPositionPID()");
	size_t totalSize = 0;
	FOREACH_ARM_ID(armId) {
		ArmGains armGains;
		armGains.id = armId;

		std::string armName = Device::getArmNameFromId(armId);
		//boost::to_lower(armName);
		std::string armType(Device::getArmTypeFromId(armId).str());
		boost::to_lower(armType);

		std::string param_base = "gains";

		GET_GAIN(p)
		GET_GAIN(i)
		GET_GAIN(d)

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

