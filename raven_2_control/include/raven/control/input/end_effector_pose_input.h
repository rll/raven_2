/*
 * end_effector_pose_input.h
 *
 *  Created on: Oct 10, 2012
 *      Author: benk
 */

#ifndef END_EFFECTOR_POSE_INPUT_H_
#define END_EFFECTOR_POSE_INPUT_H_

#include <raven/control/control_input.h>

#include <LinearMath/btTransform.h>

class EndEffectorPoseData {
private:
	std::auto_ptr<btTransform> value_;
public:
	EndEffectorPoseData(const EndEffectorPoseData& other) : value_(new btTransform(*(other.value_))) {}
	EndEffectorPoseData(int id,size_t numMotors,size_t numJoints) : value_(new btTransform(btTransform::getIdentity())) {}

	btTransform& value() { return *value_; }
	const btTransform& value() const { return *value_; }
};

class EndEffectorPoseInput : public SeparateArmControlInput<EndEffectorPoseData> {
public:
	std::vector<btTransform> values() const;
};
POINTER_TYPES(EndEffectorPoseInput)


#endif /* END_EFFECTOR_POSE_INPUT_H_ */
