/*
 * joint_input.cpp
 *
 *  Created on: Oct 10, 2012
 *      Author: benk
 */

#include <raven/control/input/joint_input.h>
#include <stdexcept>

#include "defines.h"

float&
JointValuesInput::valueByOldType(int type) {
	int arm_ind;
	int joint_ind;
	getArmAndJointIndices(type,arm_ind,joint_ind);
	int arm_id = armSerialFromID(arm_ind);

	if (joint_ind == 3) {
		throw std::out_of_range("JVI::vBOT bad ind!");
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].value(joint_ind);
}

const float&
JointValuesInput::valueByOldType(int type) const {
	int arm_ind;
	int joint_ind;
	getArmAndJointIndices(type,arm_ind,joint_ind);
	int arm_id = armSerialFromID(arm_ind);

	if (joint_ind == 3) {
		throw std::out_of_range("JVI::vBOT bad ind!");
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].value(joint_ind);
}

Eigen::VectorXf
JointValuesInput::values() const {
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
JointValuesInput::values(Eigen::VectorXf& v, Eigen::VectorXi& armIds, Eigen::VectorXi& jointInds) const {
	size_t numEl = 0;
	for (size_t i=0;i<arms_.size();i++) {
		numEl += arms_[i].values().rows();
	}
	v.resize(numEl);
	armIds.resize(numEl);
	jointInds.resize(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = arms_[i].values().rows();
		v.segment(ind,numElInArm) = arms_[i].values();
		armIds.segment(ind,numElInArm).setConstant(armIds_[i]);
		for (size_t j=0;j<numElInArm;j++) {
			jointInds(ind + j) = j;
		}
		ind += numElInArm;
	}
}

Eigen::VectorXf
JointValuesInput::fullVector() const {
	Arm::IdList ids = Device::armIds();
	size_t numEl = 0;
	for (size_t i=0;i<ids.size();i++) {
		numEl += Device::numJointsOnArmById(ids[i]);
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	for (size_t i=0;i<ids.size();i++) {
		size_t numElInArm = Device::numJointsOnArmById(ids[i]);
		Arm::IdList::const_iterator itr = std::find(armIds_.begin(),armIds_.end(),ids[i]);
		if (itr != armIds_.end()) {
			v.segment(ind,numElInArm) = armById(*itr).values();
		}
		ind += numElInArm;
	}
	return v;
}

void
JointValuesInput::fullVector(Eigen::VectorXf& v, Eigen::VectorXi& armIds, Eigen::VectorXi& jointInds) const {
	Arm::IdList ids = Device::armIds();
	size_t numEl = 0;
	for (size_t i=0;i<ids.size();i++) {
		numEl += Device::numJointsOnArmById(ids[i]);
	}
	v.resize(numEl);
	armIds.resize(numEl);
	jointInds.resize(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = Device::numJointsOnArmById(ids[i]);
		Arm::IdList::const_iterator itr = std::find(armIds_.begin(),armIds_.end(),ids[i]);
		if (itr != armIds_.end()) {
			v.segment(ind,numElInArm) = armById(*itr).values();
			armIds.segment(ind,numElInArm).setConstant(*itr);
			for (size_t j=0;j<numElInArm;j++) {
				jointInds(ind + j) = j;
			}
		} else {
			armIds.segment(ind,numElInArm).setConstant(-(*itr) - 1);
			for (size_t j=0;j<numElInArm;j++) {
				jointInds(ind + j) = -j - 1;
			}
		}
		ind += numElInArm;
	}
}

void
JointPositionInput::setFrom(DeviceConstPtr dev) {
	FOREACH_ARM_IN_CONST_DEVICE(arm_in,dev) {
		JointArmData& arm_curr = armById(arm_in->id());
		for (size_t i=0;i<arm_curr.size();i++) {
			arm_curr.values()[i]= arm_in->joint(i)->position();
		}
	}
}

void
JointVelocityInput::setFrom(DeviceConstPtr dev) {
	FOREACH_ARM_IN_CONST_DEVICE(arm_in,dev) {
		JointArmData& arm_curr = armById(arm_in->id());
		for (size_t i=0;i<arm_curr.size();i++) {
			arm_curr.values()[i]= arm_in->joint(i)->velocity();
		}
	}
}

/************ single arm ***********************/

void
SingleArmJointPositionInput::setFrom(DeviceConstPtr dev) {
	ArmConstPtr arm = dev->getArmById(id());
	size_t numJoints = Device::numJointsOnArmById(id());
	data().values().resize(numJoints);
	for (size_t i=0;i<numJoints;i++) {
		data().values()[i]= arm->joint(i)->position();
	}
}

void
SingleArmJointVelocityInput::setFrom(DeviceConstPtr dev) {
	ArmConstPtr arm = dev->getArmById(id());
	size_t numJoints = Device::numJointsOnArmById(id());
	data().values().resize(numJoints);
	for (size_t i=0;i<numJoints;i++) {
		data().values()[i]= arm->joint(i)->velocity();
	}
}
