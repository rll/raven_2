/*
 * control_process.cpp
 *
 *  Created on: Oct 12, 2012
 *      Author: benk
 */

#include <raven/control/controller.h>
#include <raven/control/controllers/motor_position_pid.h>
#include <raven/util/timing.h>
#include "log.h"

void* control_process(void* ) {

	log_msg("Waiting for device");
	while (ros::ok()) {
		if (Device::currentNoClone()) {
			break;
		}
		ros::Duration(0.1).sleep();
	}

	log_msg("Creating controller");

	ControllerPtr pid(new MotorPositionPID());

	log_msg("Registering controller");

	Controller::registerController(Arm::ALL_ARMS,"motor/position",pid);

	log_msg("Setting controller");
	Controller::setController(Arm::ALL_ARMS,"motor/position");

	log_msg("Waiting for input");
	while (ros::ok()) {
		if (!ControlInput::getOldControlInput()->arm(0).motorVelocities().empty()) {
			break;
		}
		ros::Duration(0.1).sleep();
	}

	log_msg("Control ready!");
	//TRACER_ON();

	ros::Rate rate(10);
	while (ros::ok()) {
		TRACER_ENTER("control_process loop");
		ControlTiming timing;

		timing.mark_overall_start();
		int ret = Controller::executeInProcessControl();
		timing.mark_overall_end();

		if (ret < 0) {
			log_msg_throttle(0.5,"Return bad! %i",ret);
		}

		ControlTiming::mark_loop_end();

		TRACER_LEAVE();
		rate.sleep();
	}

	log_warn("Control over!");

	return 0;
}

