/*
 * end_effector_control.h
 *
 *  Created on: Oct 15, 2012
 *      Author: benk
 */

#ifndef END_EFFECTOR_CONTROL_H_
#define END_EFFECTOR_CONTROL_H_

#include <raven/control/controller.h>
#include <raven/control/input/end_effector_pose_input.h>
#include <raven/control/input/end_effector_grasp_input.h>

#include <raven/control/controllers/motor_position_pid.h>

#include <Eigen/Core>


struct EndEffectorControlState : public ControllerState {
	ros::Time timestamp;
	MotorPositionPIDStatePtr motorControllerState;
	EndEffectorControlState(DevicePtr dev) : ControllerState(dev), timestamp(dev->timestamp()) {}
	virtual ControllerStatePtr clone(DevicePtr dev) const {
		EndEffectorControlState* newState = new EndEffectorControlState(dev);
		newState->timestamp = timestamp;
		newState->motorControllerState = boost::dynamic_pointer_cast<MotorPositionPIDState>(motorControllerState->clone(dev));
		return ControllerStatePtr(newState);
	}
};
POINTER_TYPES(EndEffectorControlState)

class EndEffectorController : public Controller {
private:
	MotorPositionPID motorController_;

	virtual ControllerStatePtr internalApplyControl(DevicePtr device);
public:
	EndEffectorController();
	virtual ~EndEffectorController() {}

	virtual std::string name() const { return "end_effector/pose/motor_pid"; }
	virtual std::string type() const { return "end_effector/grasp+pose"; }

};

#endif /* END_EFFECTOR_CONTROL_H_ */
