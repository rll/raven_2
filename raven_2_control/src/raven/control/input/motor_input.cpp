/*
 * motor_input.cpp
 *
 *  Created on: Oct 10, 2012
 *      Author: benk
 */

#include <raven/control/input/motor_input.h>

#include "defines.h"

float&
MotorValuesInput::valueByOldType(int type) {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].value(joint_ind);
}

const float&
MotorValuesInput::valueByOldType(int type) const {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].value(joint_ind);
}

Eigen::VectorXf
MotorValuesInput::values() const {
	Eigen::VectorXf v;
	for (size_t i=0;i<arms_.size();i++) {
		v << arms_[i].values();
	}
	return v;
}

void
MotorPositionInput::setFrom(DevicePtr dev) {
	FOREACH_ARM_IN_DEVICE(arm_in,dev) {
		MotorArmData& arm_curr = armById(arm_in->id());
		for (size_t i=0;i<arm_curr.size();i++) {
			arm_curr.values()[i]= arm_in->motor(i)->position();
		}
	}
}

void
MotorVelocityInput::setFrom(DevicePtr dev) {
	FOREACH_ARM_IN_DEVICE(arm_in,dev) {
			MotorArmData& arm_curr = armById(arm_in->id());
			for (size_t i=0;i<arm_curr.size();i++) {
				arm_curr.values()[i]= arm_in->motor(i)->velocity();
			}
		}
}

void
MotorTorqueInput::setFrom(DevicePtr dev) {
	FOREACH_ARM_IN_DEVICE(arm_in,dev) {
			MotorArmData& arm_curr = armById(arm_in->id());
			for (size_t i=0;i<arm_curr.size();i++) {
				arm_curr.values()[i]= arm_in->motor(i)->torque();
			}
		}
}
