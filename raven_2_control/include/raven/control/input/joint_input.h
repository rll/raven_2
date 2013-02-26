/*
 * joint_input.h
 *
 *  Created on: Oct 10, 2012
 *      Author: benk
 */

#ifndef JOINT_INPUT_H_
#define JOINT_INPUT_H_

#include <raven/control/control_input.h>
#include <Eigen/Core>

class JointArmData {
private:
	std::auto_ptr<Eigen::VectorXf > values_;
public:
	JointArmData(const JointArmData& other) : values_(new Eigen::VectorXf(*(other.values_))) {}
	JointArmData(int id,size_t numMotors,size_t numJoints) : values_(new Eigen::VectorXf(numJoints)) {}

	size_t size() const  { return values_->rows(); }

	float& value(size_t i) { return (*values_)(i); }
	const float& value(size_t i) const { return (*values_)(i); }

	Eigen::VectorXf& values() { return *values_; }
	const Eigen::VectorXf& values() const { return *values_; }

};

class JointValuesInput : public SeparateArmControlInput<JointArmData> {
protected:
	JointValuesInput(const Arm::IdList& ids) : SeparateArmControlInput<JointArmData>(ids) {}
public:
	float& valueByOldType(int type);
	const float& valueByOldType(int type) const;

	Eigen::VectorXf values() const;
	void values(Eigen::VectorXf& v, Eigen::VectorXi& armIds, Eigen::VectorXi& jointInds) const;

	Eigen::VectorXf fullVector() const;
	void fullVector(Eigen::VectorXf& v, Eigen::VectorXi& armIds, Eigen::VectorXi& jointInds) const;
};

class JointPositionInput : public JointValuesInput {
public:
	JointPositionInput(const Arm::IdList& ids) : JointValuesInput(ids) {}
	virtual void setFrom(DeviceConstPtr dev);
};
POINTER_TYPES(JointPositionInput)

class JointVelocityInput : public JointValuesInput {
public:
	JointVelocityInput(const Arm::IdList& ids) : JointValuesInput(ids) {}
	virtual void setFrom(DeviceConstPtr dev);
};
POINTER_TYPES(JointVelocityInput)

/************ single arm ***********************/

class SingleArmJointValuesInput : public SingleArmControlInput<JointArmData> {
protected:
	SingleArmJointValuesInput() {}
public:
	float& valueByOldType(int type);
	const float& valueByOldType(int type) const;

	Eigen::VectorXf& values() { return data().values(); }
	const Eigen::VectorXf& values() const { return data().values(); }
};

class SingleArmJointPositionInput : public JointPositionInput, public SingleArmJointValuesInput {
public:
	SingleArmJointPositionInput(Arm::IdType armId) : JointPositionInput(Arm::IdList(1,armId)) {}
	SINGLE_ARM_CONTROL_INPUT_METHODS(JointArmData)
	virtual void setFrom(DeviceConstPtr dev);
};
POINTER_TYPES(SingleArmJointPositionInput)

class SingleArmJointVelocityInput : public JointVelocityInput, public SingleArmJointValuesInput {
public:
	SingleArmJointVelocityInput(Arm::IdType armId) : JointVelocityInput(Arm::IdList(1,armId)) {}
	SINGLE_ARM_CONTROL_INPUT_METHODS(JointArmData)
	virtual void setFrom(DeviceConstPtr dev);
};
POINTER_TYPES(SingleArmJointVelocityInput)


#endif /* JOINT_INPUT_H_ */
