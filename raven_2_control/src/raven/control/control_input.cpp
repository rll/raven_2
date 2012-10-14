/*
 * control_input.cpp
 *
 *  Created on: Oct 9, 2012
 *      Author: benk
 */

#include <raven/control/control_input.h>
#include <raven/control/controller.h>

#include "defines.h"

#include <boost/thread/mutex.hpp>

boost::mutex inputMutex;

std::map<std::string,ControlInputPtr> ControlInput::CONTROL_INPUT;
OldControlInputPtr ControlInput::OLD_CONTROL_INPUT;

void
ControlInput::setControlInput(const std::string& type,ControlInputPtr input) {
	inputMutex.lock();
	ControlInput::CONTROL_INPUT[type] = input;
	inputMutex.unlock();
}

/*
ControlInputPtr
ControlInput::getControlInput() {
	return getControlInput(Controller::getController()->inputType());
}
*/

ControlInputPtr
ControlInput::getControlInput(const std::string& type) {
	ControlInputPtr input;
	inputMutex.lock();
	if (type == "old") {
		input = getOldControlInput();
	} else {
		input = CONTROL_INPUT[type];
	}
	inputMutex.unlock();
	return input;
}

OldControlInputPtr
ControlInput::getOldControlInput() {
	if (!OLD_CONTROL_INPUT) {
		OLD_CONTROL_INPUT.reset(new OldControlInput());
	}
	return OLD_CONTROL_INPUT;
}

OldArmInputData::OldArmInputData(const OldArmInputData& other) : //id_(other.id_),
			motorPositions_(new std::vector<float>(other.motorPositions_->begin(),other.motorPositions_->end())),
			motorVelocities_(new std::vector<float>(other.motorVelocities_->begin(),other.motorVelocities_->end())),
			motorTorques_(new std::vector<float>(other.motorTorques_->begin(),other.motorTorques_->end())),
			jointPositions_(new std::vector<float>(other.jointPositions_->begin(),other.jointPositions_->end())),
			jointVelocities_(new std::vector<float>(other.jointVelocities_->begin(),other.jointVelocities_->end())),
			pose_(new btTransform(*(other.pose_))), grasp_(new float(*(other.grasp_))) {

}

//OldArmInputData::OldArmInputData(int id) : id_(id),
//OldArmInputData::OldArmInputData() :
OldArmInputData::OldArmInputData(int id,size_t numMotors,size_t numJoints) :
		motorPositions_(new std::vector<float>(numMotors,0)),
		motorVelocities_(new std::vector<float>(numMotors,0)),
		motorTorques_(new std::vector<float>(numMotors,0)),
		jointPositions_(new std::vector<float>(numJoints,0)),
		jointVelocities_(new std::vector<float>(numJoints,0)),
		pose_(new btTransform(btTransform::getIdentity())), grasp_(new float(0)) {

}


OldControlInput::OldControlInput() {
	/*static DevicePtr device;
	FOREACH_ARM_IN_CURRENT_DEVICE(arm,device) {
		arms_.push_back(OldArmInputData(arm->id()));
	}*/
}

void
OldControlInput::setFrom(DevicePtr dev) {
	FOREACH_ARM_IN_DEVICE(arm_in,dev) {
		OldArmInputData& arm_curr = armById(arm_in->id());
		for (size_t i=0;i<arm_curr.motorPositions().size();i++) {
			arm_curr.motorPositions()[i]= arm_in->motor(i)->position();
		}
		for (size_t i=0;i<arm_curr.motorVelocities().size();i++) {
			arm_curr.motorVelocities()[i]= arm_in->motor(i)->velocity();
		}
		for (size_t i=0;i<arm_curr.motorTorques().size();i++) {
			arm_curr.motorTorques()[i]= arm_in->motor(i)->torque();
		}
		for (size_t i=0;i<arm_curr.jointPositions().size();i++) {
			arm_curr.jointPositions()[i]= arm_in->joint(i)->position();
		}
		for (size_t i=0;i<arm_curr.jointVelocities().size();i++) {
			arm_curr.jointVelocities()[i]= arm_in->joint(i)->velocity();
		}
		arm_curr.pose() = arm_in->pose();
		arm_curr.grasp() = arm_in->joint(Joint::Type::GRASP_)->position();
	}
}

Eigen::VectorXf OldControlInput::motorPositionVector() const {
	size_t numEl = 0;
	for (size_t i=0;i<arms_.size();i++) {
		numEl += arms_[i].motorPositions().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = arms_[i].motorPositions().size();
		v.segment(ind,numElInArm) = arms_[i].motorPositionVector();
		ind += numElInArm;
	}
	return v;
}
Eigen::VectorXf OldControlInput::motorVelocityVector() const {
	size_t numEl = 0;
	for (size_t i=0;i<arms_.size();i++) {
		numEl += arms_[i].motorVelocities().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = arms_[i].motorVelocities().size();
		v.segment(ind,numElInArm) = arms_[i].motorVelocityVector();
		ind += numElInArm;
	}
	return v;
}
Eigen::VectorXf OldControlInput::motorTorqueVector() const {
	size_t numEl = 0;
	for (size_t i=0;i<arms_.size();i++) {
		numEl += arms_[i].motorTorques().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = arms_[i].motorTorques().size();
		v.segment(ind,numElInArm) = arms_[i].motorTorqueVector();
		ind += numElInArm;
	}
	return v;
}
Eigen::VectorXf OldControlInput::jointPositionVector() const {
	size_t numEl = 0;
	for (size_t i=0;i<arms_.size();i++) {
		numEl += arms_[i].jointPositions().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = arms_[i].jointPositions().size();
		v.segment(ind,numElInArm) = arms_[i].jointPositionVector();
		ind += numElInArm;
	}
	return v;
}
Eigen::VectorXf OldControlInput::jointVelocityVector() const {
	size_t numEl = 0;
	for (size_t i=0;i<arms_.size();i++) {
		numEl += arms_[i].jointVelocities().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = arms_[i].jointVelocities().size();
		v.segment(ind,numElInArm) = arms_[i].jointVelocityVector();
		ind += numElInArm;
	}
	return v;
}

float&
OldControlInput::motorPositionByOldType(int type) {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].motorPosition(joint_ind);
}

const float&
OldControlInput::motorPositionByOldType(int type) const {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].motorPosition(joint_ind);
}

float&
OldControlInput::motorVelocityByOldType(int type) {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].motorVelocity(joint_ind);
}

const float&
OldControlInput::motorVelocityByOldType(int type) const {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].motorVelocity(joint_ind);
}

float&
OldControlInput::motorTorqueByOldType(int type) {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].motorTorque(joint_ind);
}

const float&
OldControlInput::motorTorqueByOldType(int type) const {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].motorTorque(joint_ind);
}

float&
OldControlInput::jointPositionByOldType(int type) {
	int arm_id;
		int joint_ind;
		getArmAndJointIndices(type,arm_id,joint_ind);

		if (joint_ind == 3) {
			//return 0;
		}
		if (joint_ind > 3) {
			joint_ind -= 1;
		}
		return arms_[arm_id].jointPosition(joint_ind);
}

const float&
OldControlInput::jointPositionByOldType(int type) const {
	int arm_id;
		int joint_ind;
		getArmAndJointIndices(type,arm_id,joint_ind);

		if (joint_ind == 3) {
			//return 0;
		}
		if (joint_ind > 3) {
			joint_ind -= 1;
		}
		return arms_[arm_id].jointPosition(joint_ind);
}

float&
OldControlInput::jointVelocityByOldType(int type) {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].jointVelocity(joint_ind);
}

const float&
OldControlInput::jointVelocityByOldType(int type) const {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].jointVelocity(joint_ind);
}

//OldArmInputData&
//OldControlInput::armById(int id) {
//	for (size_t i=0;i<arms_.size();i++) {
//		if (arms_.at(i).id() == id) {
//			return arms_.at(i);
//		}
//	}
//}
//
//const OldArmInputData&
//OldControlInput::armById(int id) const {
//	for (size_t i=0;i<arms_.size();i++) {
//		if (arms_.at(i).id() == id) {
//			return arms_.at(i);
//		}
//	}
//}
