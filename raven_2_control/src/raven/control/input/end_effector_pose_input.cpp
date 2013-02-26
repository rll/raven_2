/*
 * end_effector_pose.cpp
 *
 *  Created on: Oct 10, 2012
 *      Author: benk
 */

#include <raven/control/input/end_effector_pose_input.h>

#include <sstream>

EndEffectorPoseInput::EndEffectorPoseInput() : SeparateArmControlInput<EndEffectorPoseData>(), relative_(false) {

}
EndEffectorPoseInput::EndEffectorPoseInput(bool relative) : SeparateArmControlInput<EndEffectorPoseData>(), relative_(relative) {

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
EndEffectorPoseInput::setFrom(DevicePtr dev) {
	relative_ = false;
	FOREACH_ARM_IN_DEVICE(arm,dev) {
		armById(arm->id()).value() = arm->kinematics().forwardPose();
	}
}


EndEffectorPoseAndInsertionInput::EndEffectorPoseAndInsertionInput() : EndEffectorPoseInput(true) {

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
