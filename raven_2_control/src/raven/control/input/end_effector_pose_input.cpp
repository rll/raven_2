/*
 * end_effector_pose.cpp
 *
 *  Created on: Oct 10, 2012
 *      Author: benk
 */

#include <raven/control/input/end_effector_pose_input.h>

std::vector<btTransform>
EndEffectorPoseInput::values() const {
	std::vector<btTransform> values;
	for (size_t i=0;i<arms_.size();i++) {
		values.push_back(arms_[i].value());
	}
	return values;
}


