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
protected:
	bool relative_;
public:
	EndEffectorPoseInput(const Arm::IdList& ids);
	EndEffectorPoseInput(const Arm::IdList& ids,bool relative);

	bool relative() const { return relative_; }
	bool absolute() const { return !relative_; }
	virtual void setRelative(bool value = true) { relative_ = value; }
	virtual void setAbsolute(bool value = true) { relative_ = !value; }

	std::vector<btTransform> values() const;

	virtual void setFrom(DeviceConstPtr dev);
};
POINTER_TYPES(EndEffectorPoseInput)

class EndEffectorPoseAndInsertionInput : public EndEffectorPoseInput {
protected:
	std::map<Arm::IdType,float> insertions_;
public:
	EndEffectorPoseAndInsertionInput(const Arm::IdList& ids);

	virtual void setRelative(bool value = true) { if (!value) { throw std::runtime_error("EndEffectorPoseAndInsertionInput cannot be set to absolute!"); } }
	virtual void setAbsolute(bool value = true) { if (value) { throw std::runtime_error("EndEffectorPoseAndInsertionInput cannot be set to absolute!"); } }

	virtual bool hasInsertion(Arm::IdType id) const;
	virtual float getInsertion(Arm::IdType id) const;
	virtual void setInsertion(Arm::IdType id,float velocity);
	virtual void removeInsertion(Arm::IdType id);
	virtual void clearInsertions();

	virtual void setFrom(DeviceConstPtr dev) { throw std::runtime_error("EndEffectorPoseAndInsertionInput cannot be set from device!"); }
};
POINTER_TYPES(EndEffectorPoseAndInsertionInput)


/************ single arm ***********************/

class SingleArmEndEffectorPoseInput : public EndEffectorPoseInput, public SingleArmControlInput<EndEffectorPoseData> {
public:
	SingleArmEndEffectorPoseInput(Arm::IdType id);
	SingleArmEndEffectorPoseInput(Arm::IdType id, bool relative);
	SINGLE_ARM_CONTROL_INPUT_METHODS(EndEffectorPoseData)

	btTransform& value() { return data().value(); }
	const btTransform& value() const { return data().value(); }

	virtual void setFrom(DeviceConstPtr dev);
};
POINTER_TYPES(SingleArmEndEffectorPoseInput)

class SingleArmEndEffectorPoseAndInsertionInput : public EndEffectorPoseAndInsertionInput, public SingleArmEndEffectorPoseInput {
public:
	SingleArmEndEffectorPoseAndInsertionInput(Arm::IdType id);

	virtual void setRelative(bool value = true) { if (!value) { throw std::runtime_error("SingleArmEndEffectorPoseAndInsertionInput cannot be set to absolute!"); } }
	virtual void setAbsolute(bool value = true) { if (value) { throw std::runtime_error("SingleArmEndEffectorPoseAndInsertionInput cannot be set to absolute!"); } }

	virtual bool hasInsertion() const;
	virtual float getInsertion() const;
	void setInsertion(float velocity);
	void clearInsertion();

	virtual void setInsertion(Arm::IdType id,float velocity);

	virtual void setFrom(DeviceConstPtr dev) { throw std::runtime_error("SingleArmEndEffectorPoseAndInsertionInput cannot be set from device!"); }
};
POINTER_TYPES(SingleArmEndEffectorPoseAndInsertionInput)


#endif /* END_EFFECTOR_POSE_INPUT_H_ */
