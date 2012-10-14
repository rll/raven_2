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
public:
	float& valueByOldType(int type);
	const float& valueByOldType(int type) const;

	Eigen::VectorXf values() const;
};

class JointPositionInput : public JointValuesInput {
public:
	virtual void setFrom(DevicePtr dev);
};
typedef boost::shared_ptr<JointPositionInput> JointPositionInputPtr;

class JointVelocityInput : public JointValuesInput {
public:
	virtual void setFrom(DevicePtr dev);
};
POINTER_TYPES(JointVelocityInput)


#endif /* JOINT_INPUT_H_ */
