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

#include <map>
#include <stdexcept>

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
private:
	bool relative_;
public:
	EndEffectorPoseInput();
	EndEffectorPoseInput(bool relative);

	bool relative() const { return relative_; }
	bool absolute() const { return !relative_; }
	virtual void setRelative(bool value = true) { relative_ = value; }
	virtual void setAbsolute(bool value = true) { relative_ = !value; }

	std::vector<btTransform> values() const;

	virtual void setFrom(DevicePtr dev);
};
POINTER_TYPES(EndEffectorPoseInput)

class EndEffectorPoseAndInsertionInput : public EndEffectorPoseInput {
private:
	std::map<Arm::IdType,float> insertions_;
public:
	EndEffectorPoseAndInsertionInput();

	virtual void setRelative(bool value = true) { if (!value) { throw std::runtime_error("EndEffectorPoseAndInsertionInput cannot be set to absolute!"); } }
	virtual void setAbsolute(bool value = true) { if (value) { throw std::runtime_error("EndEffectorPoseAndInsertionInput cannot be set to absolute!"); } }

	bool hasInsertion(Arm::IdType id) const;
	float getInsertion(Arm::IdType id) const;
	void setInsertion(Arm::IdType id,float velocity);
	void removeInsertion(Arm::IdType id);
	void clearInsertions();

	virtual void setFrom(DevicePtr dev) { throw std::runtime_error("EndEffectorPoseAndInsertionInput cannot be set from device!"); }
};
POINTER_TYPES(EndEffectorPoseAndInsertionInput)

#endif /* END_EFFECTOR_POSE_INPUT_H_ */
