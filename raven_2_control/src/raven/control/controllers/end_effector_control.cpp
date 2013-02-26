/*
 * end_effector_control.cpp
 *
 *  Created on: Oct 15, 2012
 *      Author: benk
 */

#include <raven/control/controllers/end_effector_control.h>

#include "log.h"
#include <algorithm>

EndEffectorController::EndEffectorController() : Controller(1) {
	
}

ControllerStatePtr
EndEffectorController::internalApplyControl(DevicePtr device) {
	static MotorPositionInputPtr motorInput;
	static DevicePtr internalDevice;
	TRACER_ENTER("MotorPositionPID::internalApplyControl");

	EndEffectorControlStatePtr lastState = getLastState<EndEffectorControlState>();
	EndEffectorControlStatePtr state = cloneLastState<EndEffectorControlState>(device);
	if (!state) {
		state.reset(new EndEffectorControlState(device));
	}

	DevicePtr devTmp;

	EndEffectorPoseInputPtr poseInput;
	EndEffectorGraspInputPtr graspInput;
	OldControlInputPtr oldControlInput;

	MultipleControlInputPtr multiInput = getInput<MultipleControlInput>();
	if (multiInput) {
		poseInput = multiInput->getInput<EndEffectorPoseInput>("pose");
		graspInput = multiInput->getInput<EndEffectorGraspInput>("grasp");

		if (!poseInput && !graspInput) {
			//TODO: complain
		}
	} else {
		poseInput = getInput<EndEffectorPoseInput>();
		graspInput = getInput<EndEffectorGraspInput>();

		if (!poseInput && !graspInput) {
			oldControlInput = ControlInput::getOldControlInput();
		}
	}

	if (graspInput) {
		device->cloneInto(internalDevice);
		FOREACH_ARM_IN_DEVICE_AND_ID_LIST(arm,internalDevice,graspInput->ids()) {
			arm->getJointByType(Joint::Type::GRASP_)->setPosition(graspInput->armById(arm->id()).value());
		}
		devTmp = internalDevice;
	} else if (oldControlInput) {
		device->cloneInto(internalDevice);
		FOREACH_ARM_IN_DEVICE(arm,internalDevice) {
			arm->getJointByType(Joint::Type::GRASP_)->setPosition(oldControlInput->armById(arm->id()).grasp());
		}
		devTmp = internalDevice;
	} else {
		devTmp = device;
	}

	FOREACH_ARM_IN_DEVICE(arm,devTmp) {
		btTransform pose;
		if (poseInput and poseInput->hasId(arm->id())) {
			if (poseInput->absolute()) {
				pose = poseInput->armById(arm->id()).value();
			} else {
				pose.setOrigin(device->getArmById(arm->id())->pose().getOrigin() + poseInput->armById(arm->id()).value().getOrigin());
				pose.setRotation(poseInput->armById(arm->id()).value().getRotation());
				EndEffectorPoseAndInsertionInputPtr withInsertion = boost::dynamic_pointer_cast<EndEffectorPoseAndInsertionInput>(poseInput);
				if (withInsertion && withInsertion->hasInsertion(arm->id())) {
					//FIXME: insertion
				}
			}
		} else {
			pose = oldControlInput->armById(arm->id()).pose();
		}
		InverseKinematicsReportPtr report = arm->kinematics().inverse(pose);
		if (report->success()) {
			motorInput->armById(arm->id()).values() = arm->motorPositionVector();
		} else {
			log_err_throttle(0.25,"inv kinematics bad! %f",0.1);
			motorInput->armById(arm->id()).values() = device->getArmById(arm->id())->motorPositionVector();
		}
	}

	motorController_.setInput(motorInput);

	motorController_.applyControl(device);

	state->motorControllerState = motorController_.lastState<MotorPositionPIDState>();

	TRACER_LEAVE();
	return state;
}
