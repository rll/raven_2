/*
 * motor_input.cpp
 *
 *  Created on: Oct 10, 2012
 *      Author: benk
 */

#include <raven/control/input/motor_input.h>
#include <algorithm>

#include "defines.h"

float&
MotorValuesInput::valueByOldType(int type) {
	int arm_ind;
	int joint_ind;
	getArmAndJointIndices(type,arm_ind,joint_ind);
	int arm_id = armSerialFromID(arm_ind);

	if (joint_ind == 3) {
		throw std::out_of_range("MVI::vBOT bad ind!");
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return armById(arm_id).value(joint_ind);
}

const float&
MotorValuesInput::valueByOldType(int type) const {
	int arm_ind;
	int joint_ind;
	getArmAndJointIndices(type,arm_ind,joint_ind);
	int arm_id = armSerialFromID(arm_ind);

	if (joint_ind == 3) {
		throw std::out_of_range("MVI::vBOT bad ind!");
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return armById(arm_id).value(joint_ind);
}

Eigen::VectorXf
MotorValuesInput::values() const {
	size_t numEl = 0;
	for (size_t i=0;i<arms_.size();i++) {
		numEl += arms_[i].values().rows();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = arms_[i].values().rows();
		v.segment(ind,numElInArm) = arms_[i].values();
		ind += numElInArm;
	}
	return v;
}

void
MotorValuesInput::values(Eigen::VectorXf& v, Eigen::VectorXi& armIds, Eigen::VectorXi& motorInds) const {
	size_t numEl = 0;
	for (size_t i=0;i<arms_.size();i++) {
		numEl += arms_[i].values().rows();
	}
	v.resize(numEl);
	armIds.resize(numEl);
	motorInds.resize(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = arms_[i].values().rows();
		v.segment(ind,numElInArm) = arms_[i].values();
		armIds.segment(ind,numElInArm).setConstant(armIds_[i]);
		for (size_t j=0;j<numElInArm;j++) {
			motorInds(ind + j) = j;
		}
		ind += numElInArm;
	}
}

Eigen::VectorXf
MotorValuesInput::fullVector() const {
	Arm::IdList ids = Device::armIds();
	size_t numEl = 0;
	for (size_t i=0;i<ids.size();i++) {
		numEl += Device::numMotorsOnArmById(ids[i]);
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	for (size_t i=0;i<ids.size();i++) {
		size_t numElInArm = Device::numMotorsOnArmById(ids[i]);
		Arm::IdList::const_iterator itr = std::find(armIds_.begin(),armIds_.end(),ids[i]);
		if (itr != armIds_.end()) {
			v.segment(ind,numElInArm) = armById(*itr).values();
		}
		ind += numElInArm;
	}
	return v;
}

void
MotorValuesInput::fullVector(Eigen::VectorXf& v, Eigen::VectorXi& armIds, Eigen::VectorXi& motorInds) const {
	Arm::IdList ids = Device::armIds();
	size_t numEl = 0;
	for (size_t i=0;i<ids.size();i++) {
		numEl += Device::numMotorsOnArmById(ids[i]);
	}
	v.resize(numEl);
	armIds.resize(numEl);
	motorInds.resize(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = Device::numMotorsOnArmById(ids[i]);
		Arm::IdList::const_iterator itr = std::find(armIds_.begin(),armIds_.end(),ids[i]);
		if (itr != armIds_.end()) {
			v.segment(ind,numElInArm) = armById(*itr).values();
			armIds.segment(ind,numElInArm).setConstant(*itr);
			for (size_t j=0;j<numElInArm;j++) {
				motorInds(ind + j) = j;
			}
		} else {
			armIds.segment(ind,numElInArm).setConstant(-(*itr) - 1);
			for (size_t j=0;j<numElInArm;j++) {
				motorInds(ind + j) = -j - 1;
			}
		}
		ind += numElInArm;
	}
}

void
MotorPositionInput::setFrom(DeviceConstPtr dev) {
	TRACER_ENTER_SCOPE("MotorPositionInput::setFrom()");
	FOREACH_ARM_IN_CONST_DEVICE(arm_in,dev) {
		MotorArmData& arm_curr = armById(arm_in->id());
		for (size_t i=0;i<arm_curr.size();i++) {
			arm_curr.values()[i]= arm_in->motor(i)->position();
		}
	}
}

void
MotorVelocityInput::setFrom(DeviceConstPtr dev) {
	TRACER_ENTER_SCOPE("MotorVelocityInput::setFrom()");
	FOREACH_ARM_IN_CONST_DEVICE(arm_in,dev) {
		MotorArmData& arm_curr = armById(arm_in->id());
		for (size_t i=0;i<arm_curr.size();i++) {
			arm_curr.values()[i]= arm_in->motor(i)->velocity();
		}
	}
}

void
MotorTorqueInput::setFrom(DeviceConstPtr dev) {
	TRACER_ENTER_SCOPE("MotorTorqueInput::setFrom()");
	FOREACH_ARM_IN_CONST_DEVICE(arm_in,dev) {
		MotorArmData& arm_curr = armById(arm_in->id());
		for (size_t i=0;i<arm_curr.size();i++) {
			arm_curr.values()[i]= arm_in->motor(i)->torque();
		}
	}
}

/**************** single arm *******************/

void
SingleArmMotorPositionInput::setFrom(DeviceConstPtr dev) {
	ArmConstPtr arm = dev->getArmById(id());
	size_t numMotors = Device::numMotorsOnArmById(id());
	data().values().resize(numMotors);
	for (size_t i=0;i<numMotors;i++) {
		data().values()[i]= arm->motor(i)->position();
	}
}

void
SingleArmMotorVelocityInput::setFrom(DeviceConstPtr dev) {
	ArmConstPtr arm = dev->getArmById(id());
	size_t numMotors = Device::numMotorsOnArmById(id());
	data().values().resize(numMotors);
	for (size_t i=0;i<numMotors;i++) {
		data().values()[i]= arm->motor(i)->velocity();
	}
}

void
SingleArmMotorTorqueInput::setFrom(DeviceConstPtr dev) {
	ArmConstPtr arm = dev->getArmById(id());
	size_t numMotors = Device::numMotorsOnArmById(id());
	data().values().resize(numMotors);
	for (size_t i=0;i<numMotors;i++) {
		data().values()[i]= arm->motor(i)->torque();
	}
}
