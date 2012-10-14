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
#include <raven/log.h>

std::vector<std::string>* MotorPositionPID::INPUT_TYPES = 0;

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
	if (positionInput_ && velocityInput_) {
		pos_d = positionInput_->values();
		vel_d = velocityInput_->values();
	} else {
		pos_d = oldControlInput_->motorPositionVector();
		vel_d = oldControlInput_->motorVelocityVector();
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

//	std::cout << "p sz " << pos_err.rows() << " " << KP_.rows() << std::endl;
//	std::cout << "i sz " << int_err.rows() << " " << KI_.rows() << std::endl;
//	std::cout << "d sz " << vel_err.rows() << " " << KD_.rows() << std::endl;
//
//	std::cerr << pos_err << std::endl << std::endl << KP_ << std::endl << std::endl;
//	std::cerr << int_err << std::endl << std::endl << KI_ << std::endl << std::endl;
//	std::cerr << vel_err << std::endl << std::endl << KD_ << std::endl << std::endl;

	Eigen::VectorXf p_term = KP_.cwiseProduct(pos_err);
	Eigen::VectorXf i_term = KI_.cwiseProduct(int_err);
	Eigen::VectorXf d_term = KD_.cwiseProduct(vel_err);

//	if (p_term.rows() != i_term.rows() || i_term.rows() != d_term.rows()) {
//		log_err("sizes different! %i %i %i",p_term.rows(), i_term.rows(),d_term.rows());
//		std::cerr << pos_err << " " << KP_ << p_term << std::endl;
//		std::cerr << int_err << " " << KI_ << i_term << std::endl;
//		std::cerr << vel_err << " " << KD_ << d_term << std::endl;
//	}
//	if (p_term.rows() != i_term.rows() || i_term.rows() != d_term.rows() || p_term.rows() != d_term.rows()|| p_term.cols() != i_term.cols() || i_term.cols() != d_term.cols() || p_term.cols() != d_term.cols()) {
//		log_err("sizes different! (%i,%i) (%i,%i) (%i,%i)",p_term.rows(), p_term.cols(), i_term.rows(), i_term.cols(), d_term.rows(), d_term.cols());
//		std::cerr << pos_err << " " << KP_ << p_term << std::endl;
//		std::cerr << int_err << " " << KI_ << i_term << std::endl;
//		std::cerr << vel_err << " " << KD_ << d_term << std::endl;
//	}

	Eigen::VectorXf values1 = p_term + i_term;
	Eigen::VectorXf values2 = i_term + d_term;
	Eigen::VectorXf values3 = p_term + d_term;

	Eigen::VectorXf values = p_term + i_term + d_term;

	size_t begin_ind = 0;
	for (size_t i=0;i<device->arms().size();i++) {
		device->arm(i)->motorFilter()->getMotorsForUpdate(motorsForUpdate);
		Eigen::VectorXf armVals = values.segment(begin_ind,motorsForUpdate.size());

		for (size_t j=0;j<motorsForUpdate.size();j++) {
			float val = armVals(j);
			motorsForUpdate[j]->setTorque(val);
		}

		begin_ind += motorsForUpdate.size();
	}

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

const std::vector<std::string>&
MotorPositionPID::getInputTypes() const {
	if (!INPUT_TYPES) {
		INPUT_TYPES = new std::vector<std::string>();
		INPUT_TYPES->push_back("old");
		INPUT_TYPES->push_back("motor/position");
		INPUT_TYPES->push_back("motor/velocity");
	}
	return *INPUT_TYPES;
}

void
MotorPositionPID::clearInput() {
	oldControlInput_.reset();
	positionInput_.reset();
	velocityInput_.reset();
}

void
MotorPositionPID::setInput(std::string type, ControlInputPtr input) {
	if (type == "old") {
		oldControlInput_ = boost::dynamic_pointer_cast<OldControlInput>(input);
	} else if (type == "motor/position") {
		positionInput_ = boost::dynamic_pointer_cast<MotorPositionInput>(input);
	} else if (type == "motor/velocity") {
		velocityInput_ = boost::dynamic_pointer_cast<MotorVelocityInput>(input);
	}
}

