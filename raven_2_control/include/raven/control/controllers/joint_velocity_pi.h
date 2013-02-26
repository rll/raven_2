/*
 * joint_velocity_pi.h
 *
 *  Created on: Oct 10, 2012
 *      Author: benk
 */

#ifndef JOINT_VELOCITY_PI_H_
#define JOINT_VELOCITY_PI_H_

#include <raven/control/controller.h>
#include <raven/control/input/joint_input.h>

struct JointVelocityPIState : public ControllerState {
	Eigen::VectorXf velocityErrorIntegral;
	JointVelocityPIState(DevicePtr dev) : ControllerState(dev), velocityErrorIntegral() {}
	virtual ControllerStatePtr clone(DevicePtr dev) const {
		JointVelocityPIState* newState = new JointVelocityPIState(dev);
		newState->velocityErrorIntegral = velocityErrorIntegral;
		return ControllerStatePtr(newState);
	}
};
POINTER_TYPES(JointVelocityPIState)

class JointVelocityPI : public Controller {
public:
	struct Gains {
		float KP;
		float KI;
		//float KD;
	};
	struct ArmGains {
		int id;
		std::vector<Gains> gains;
	};
private:
	std::vector<ArmGains> gains_;
	Eigen::VectorXf KP_;
	Eigen::VectorXf KI_;
	//Eigen::VectorXf KD_;
	bool reset_;

	virtual ControllerStatePtr internalApplyControl(DevicePtr device);
public:
	JointVelocityPI(size_t history_size=0);
	virtual ~JointVelocityPI() {}

	virtual std::string name() const { return "joint_velocity_pi"; }
	virtual std::string type() const { return "joint_velocity"; }

	void resetIntegrator() { reset_ = true; }
};


#endif /* JOINT_VELOCITY_PI_H_ */
