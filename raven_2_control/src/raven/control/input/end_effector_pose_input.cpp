/*
 * end_effector_pose.cpp
 *
 *  Created on: Oct 10, 2012
 *      Author: benk
 */

#include <raven/control/input/end_effector_pose_input.h>

#include <sstream>

EndEffectorPoseInput::EndEffectorPoseInput(const Arm::IdList& ids) : SeparateArmControlInput<EndEffectorPoseData>(ids), relative_(false) {

}
EndEffectorPoseInput::EndEffectorPoseInput(const Arm::IdList& ids,bool relative) : SeparateArmControlInput<EndEffectorPoseData>(ids), relative_(relative) {

}

std::vector<btTransform>
EndEffectorPoseInput::values() const {
	std::vector<btTransform> values;
	for (size_t i=0;i<arms_.size();i++) {
		values.push_back(arms_[i].value());
	}
	return values;
}

void
EndEffectorPoseInput::setFrom(DeviceConstPtr dev) {
	relative_ = false;
	FOREACH_ARM_IN_CONST_DEVICE(arm,dev) {
		armById(arm->id()).value() = arm->kinematics().forwardPose();
	}
}


EndEffectorPoseAndInsertionInput::EndEffectorPoseAndInsertionInput(const Arm::IdList& ids) : EndEffectorPoseInput(ids,true) {

}

bool
EndEffectorPoseAndInsertionInput::hasInsertion(Arm::IdType id) const {
	return insertions_.find(id) != insertions_.end();
}

float
EndEffectorPoseAndInsertionInput::getInsertion(Arm::IdType id) const {
	if (!hasInsertion(id)) {
		std::stringstream ss;
		ss << "Insertion for arm " << id << " not found!";
		throw std::out_of_range(ss.str());
	}
	return insertions_.at(id);
}

void
EndEffectorPoseAndInsertionInput::setInsertion(Arm::IdType id,float velocity) {
	insertions_[id] = velocity;
}

void
EndEffectorPoseAndInsertionInput::removeInsertion(Arm::IdType id) {
	insertions_.erase(id);
}

void
EndEffectorPoseAndInsertionInput::clearInsertions() {
	insertions_.clear();
}

SingleArmEndEffectorPoseInput::SingleArmEndEffectorPoseInput(Arm::IdType id) : EndEffectorPoseInput(Arm::IdList(1,id)) {

}
SingleArmEndEffectorPoseInput::SingleArmEndEffectorPoseInput(Arm::IdType id, bool relative) : EndEffectorPoseInput(Arm::IdList(1,id),relative) {

}

void
SingleArmEndEffectorPoseInput::setFrom(DeviceConstPtr dev) {
	relative_ = false;
	data().value() = dev->getArmById(id())->kinematics().forwardPose();
}

SingleArmEndEffectorPoseAndInsertionInput::SingleArmEndEffectorPoseAndInsertionInput(Arm::IdType id) : EndEffectorPoseAndInsertionInput(Arm::IdList(1,id)), SingleArmEndEffectorPoseInput(id) {

}

bool
SingleArmEndEffectorPoseAndInsertionInput::hasInsertion() const {
	return EndEffectorPoseAndInsertionInput::hasInsertion(id());
}

float
SingleArmEndEffectorPoseAndInsertionInput::getInsertion() const {
	return EndEffectorPoseAndInsertionInput::getInsertion(id());
}

void
SingleArmEndEffectorPoseAndInsertionInput::setInsertion(float velocity) {
	setInsertion(id(),velocity);
}
void
SingleArmEndEffectorPoseAndInsertionInput::clearInsertion() {
	clearInsertions();
}

void
SingleArmEndEffectorPoseAndInsertionInput::setInsertion(Arm::IdType armId,float velocity) {
	if (armId != id()) {
		throw std::out_of_range("Can't set insertion for any other arm");
	}
	insertions_[armId] = velocity;
}
