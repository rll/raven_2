/*
 * motor_pid.h
 *
 *  Created on: Oct 9, 2012
 *      Author: benk
 */

#ifndef MOTOR_PID_H_
#define MOTOR_PID_H_

#include <raven/control/controller.h>
#include <raven/control/input/motor_input.h>

#include <Eigen/Core>

struct MotorPositionPIDState : public ControllerState {
	Eigen::VectorXf positionErrorIntegral;
	MotorPositionPIDState(DevicePtr dev) : ControllerState(dev), positionErrorIntegral() {}
	virtual ControllerStatePtr clone(DevicePtr dev) const {
		MotorPositionPIDState* newState = new MotorPositionPIDState(dev);
		newState->positionErrorIntegral = positionErrorIntegral;
		return ControllerStatePtr(newState);
	}
};
POINTER_TYPES(MotorPositionPIDState)

class MotorPositionPID : public Controller {
public:
	struct Gains {
		float KP;
		float KI;
		float KD;
	};
	struct ArmGains {
		int id;
		std::vector<Gains> gains;
	};
private:
	std::vector<ArmGains> gains_;
	Eigen::VectorXf KP_;
	Eigen::VectorXf KI_;
	Eigen::VectorXf KD_;

	virtual ControllerStatePtr internalApplyControl(DevicePtr device);
public:
	MotorPositionPID();
	virtual ~MotorPositionPID() {}

	virtual std::string name() const { return "motor/position/pid"; }
	virtual std::string type() const { return "motor/position"; }
};


#endif /* MOTOR_PID_H_ */
