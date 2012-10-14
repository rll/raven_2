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
public:
	float& valueByOldType(int type);
	const float& valueByOldType(int type) const;

	Eigen::VectorXf values() const;
};

class MotorPositionInput : public MotorValuesInput {
public:
	virtual void setFrom(DevicePtr dev);
};
POINTER_TYPES(MotorPositionInput)

class MotorVelocityInput : public MotorValuesInput {
public:
	virtual void setFrom(DevicePtr dev);
};
POINTER_TYPES(MotorVelocityInput)

class MotorTorqueInput : public MotorValuesInput {
public:
	virtual void setFrom(DevicePtr dev);
};
POINTER_TYPES(MotorTorqueInput);

#endif /* MOTOR_INPUT_H_ */
