/*
 * motor_input.h
 *
 *  Created on: Oct 10, 2012
 *      Author: benk
 */

#ifndef MOTOR_INPUT_H_
#define MOTOR_INPUT_H_

#include <raven/control/control_input.h>
#include <Eigen/Core>

class MotorArmData {
private:
	std::auto_ptr<Eigen::VectorXf > values_;
public:
	MotorArmData(const MotorArmData& other) : values_(new Eigen::VectorXf(*(other.values_))) {}
	MotorArmData(int id,size_t numMotors,size_t numJoints) : values_(new Eigen::VectorXf(numMotors)) {}

	size_t size() const  { return values_->rows(); }

	float& value(size_t i) { return (*values_)(i); }
	const float& value(size_t i) const { return (*values_)(i); }

	Eigen::VectorXf& values() { return *values_; }
	const Eigen::VectorXf& values() const { return *values_; }

};

class MotorValuesInput : public SeparateArmControlInput<MotorArmData> {
protected:
	MotorValuesInput(const Arm::IdList& ids) : SeparateArmControlInput<MotorArmData>(ids) {}

public:
	float& valueByOldType(int type);
	const float& valueByOldType(int type) const;

	Eigen::VectorXf values() const;
};

class MotorPositionInput : public MotorValuesInput {
public:
	MotorPositionInput(const Arm::IdList& ids) : MotorValuesInput(ids) {}
	virtual void setFrom(DevicePtr dev);
};
POINTER_TYPES(MotorPositionInput)

class MotorVelocityInput : public MotorValuesInput {
public:
	MotorVelocityInput(const Arm::IdList& ids) : MotorValuesInput(ids) {}
	virtual void setFrom(DevicePtr dev);
};
POINTER_TYPES(MotorVelocityInput)

class MotorTorqueInput : public MotorValuesInput {
public:
	MotorTorqueInput(const Arm::IdList& ids) : MotorValuesInput(ids) {}
	virtual void setFrom(DevicePtr dev);
};
POINTER_TYPES(MotorTorqueInput);


/************ single arm ***********************/

class SingleArmMotorValuesInput : public SingleArmControlInput<MotorArmData> {
protected:
	SingleArmMotorValuesInput() {}
public:
	size_t size() const { return data().size(); }

	float& valueByOldType(int type);
	const float& valueByOldType(int type) const;

	Eigen::VectorXf& values() { return data().values(); }
	const Eigen::VectorXf& values() const { return data().values(); }
};

class SingleArmMotorPositionInput : public MotorPositionInput, public SingleArmMotorValuesInput {
public:
	SingleArmMotorPositionInput(Arm::IdType armId) : MotorPositionInput(Arm::IdList(1,armId)) {}
	SINGLE_ARM_CONTROL_INPUT_METHODS(MotorArmData)
	virtual void setFrom(DevicePtr dev);
};
POINTER_TYPES(SingleArmMotorPositionInput)

class SingleArmMotorVelocityInput : public MotorVelocityInput, public SingleArmMotorValuesInput {
public:
	SingleArmMotorVelocityInput(Arm::IdType armId) : MotorVelocityInput(Arm::IdList(1,armId)) {}
	SINGLE_ARM_CONTROL_INPUT_METHODS(MotorArmData)
	virtual void setFrom(DevicePtr dev);
};
POINTER_TYPES(SingleArmMotorVelocityInput)

class SingleArmMotorTorqueInput : public MotorTorqueInput, public SingleArmMotorValuesInput {
public:
	SingleArmMotorTorqueInput(Arm::IdType armId) : MotorTorqueInput(Arm::IdList(1,armId)) {}
	SINGLE_ARM_CONTROL_INPUT_METHODS(MotorArmData)
	virtual void setFrom(DevicePtr dev);
};
POINTER_TYPES(SingleArmMotorTorqueInput);

#endif /* MOTOR_INPUT_H_ */
