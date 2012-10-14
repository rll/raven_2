/*
 * end_effector_grasp.h
 *
 *  Created on: Oct 10, 2012
 *      Author: benk
 */

#ifndef END_EFFECTOR_GRASP_H_
#define END_EFFECTOR_GRASP_H_

#include <raven/control/control_input.h>

class EndEffectorGraspData {
private:
	std::auto_ptr<float> value_;
public:
	EndEffectorGraspData(const EndEffectorGraspData& other) : value_(new float(*(other.value_))) {}
	EndEffectorGraspData(int id,size_t numMotors,size_t numJoints) : value_(new float(0)) {}

	float& value() { return *value_; }
	const float& value() const { return *value_; }
};

class EndEffectorGraspInput : public SeparateArmControlInput<EndEffectorGraspData> {
public:
	std::vector<float> values() const;
};
POINTER_TYPES(EndEffectorGraspInput)


#endif /* END_EFFECTOR_GRASP_H_ */
