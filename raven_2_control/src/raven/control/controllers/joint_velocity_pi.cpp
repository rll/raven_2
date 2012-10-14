/*
 * joint_velocity_pi.cpp
 *
 *  Created on: Oct 10, 2012
 *      Author: benk
 */

#include <raven/control/controllers/joint_velocity_pi.h>

#include <algorithm>
#include <string>

std::vector<std::string>* JointVelocityPI::INPUT_TYPES = 0;

ControllerStatePtr
JointVelocityPI::internalApplyControl(DevicePtr device) {
	static MotorList motorsForUpdate;

	JointVelocityPIStatePtr lastState = getLastState<JointVelocityPIState>();
	JointVelocityPIStatePtr state = cloneLastState<JointVelocityPIState>(device);
	if (!state) {
		state.reset(new JointVelocityPIState(device));
	}

	Eigen::VectorXf vel = device->motorVelocityVector();

	Eigen::VectorXf vel_d;
	if (velocityInput_) {
		vel_d = velocityInput_->values();
	} else {
		vel_d = oldControlInput_->motorVelocityVector();
	}

	Eigen::VectorXf vel_err = vel_d - vel;

	Eigen::VectorXf int_err;
	if (!lastState || reset_) {
		int_err.setZero(vel_err.rows());
	} else {
		int_err = lastState->velocityErrorIntegral + vel_err * (lastState->device->timestamp() - device->timestamp()).toSec();
	}
	state->velocityErrorIntegral = int_err;

	Eigen::VectorXf p_term = KP_.cwiseProduct(vel_err);
	Eigen::VectorXf i_term = KI_.cwiseProduct(int_err);

	Eigen::VectorXf values = p_term + i_term;

	size_t begin_ind = 0;
	for (size_t i=0;i<device->arms().size();i++) {
		device->arm(i)->motorFilter()->getMotorsForUpdate(motorsForUpdate);
		Eigen::VectorXf armVals = values.segment(motorsForUpdate.size(),begin_ind);

		for (size_t j=0;i<motorsForUpdate.size();j++) {
			float val = armVals(j);
			motorsForUpdate[j]->setTorque(val);
		}

		begin_ind += motorsForUpdate.size();
	}

	return state;
}

JointVelocityPI::JointVelocityPI(size_t history_size) : reset_(false) {
	static DevicePtr dev;
	FOREACH_ARM_IN_CURRENT_DEVICE(arm,dev) {
		ArmGains armGains;
		armGains.id = arm->id();

		std::string armName = arm->name();
		std::transform(armName.begin(), armName.end(), armName.begin(), ::tolower);

		std::vector<double> kp_gains = getParameterVector<double>("gains_" + armName + "_kp");
		std::vector<double> ki_gains = getParameterVector<double>("gains_" + armName + "_ki");
		//std::vector<double> kd_gains = getParameterVector<double>("gains_" + armName + "_kd");

		for (size_t i=0;i<kp_gains.size();i++) {
			Gains g;
			g.KP = kp_gains[i];
			g.KI = ki_gains[i];
			//g.KD = kd_gains[i];
			armGains.gains.push_back(g);
			KP_ << g.KP;
			KI_ << g.KI;
			//KD_ << g.KD;
		}
		gains_.push_back(armGains);
	}
}

const std::vector<std::string>&
JointVelocityPI::getInputTypes() const {
	if (!INPUT_TYPES) {
		INPUT_TYPES = new std::vector<std::string>();
		INPUT_TYPES->push_back("old");
		INPUT_TYPES->push_back("joint.velocity");
	}
	return *INPUT_TYPES;
}

void
JointVelocityPI::clearInput() {
	oldControlInput_.reset();
	velocityInput_.reset();
}

void
JointVelocityPI::setInput(std::string type, ControlInputPtr input) {
	if (type == "old") {
		oldControlInput_ = boost::dynamic_pointer_cast<OldControlInput>(input);
	} else if (type == "joint.velocity") {
		velocityInput_ = boost::dynamic_pointer_cast<JointVelocityInput>(input);
	}
}
