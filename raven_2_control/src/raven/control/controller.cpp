/*
 * controller.cpp
 *
 *  Created on: Oct 2, 2012
 *      Author: benk
 */

#include <raven/control/controller.h>
#include <raven/state/runlevel.h>

#include "log.h"

#include <boost/thread/mutex.hpp>

boost::mutex controllerMutex;
boost::mutex controlOutputMutex;

std::string Controller::CURRENT_CONTROLLER;
std::map<std::string,ControllerPtr> Controller::CONTROLLERS;
DevicePtr Controller::CONTROL_OUTPUT;

#define CONTROL_OUTPUT_HISTORY_SIZE 50
std::map<std::string,History<Device>::Type> Controller::CONTROL_OUTPUT_HISTORY;

ControllerPtr
Controller::getController() {
	boost::mutex::scoped_lock(controllerMutex);
	ControllerPtr c;
	c = CONTROLLERS[CURRENT_CONTROLLER];
	return c;
}

ControllerPtr
Controller::getControllerAndType(std::string& type) {
	boost::mutex::scoped_lock(controllerMutex);
	ControllerPtr c;
	type = CURRENT_CONTROLLER;
	c = CONTROLLERS[CURRENT_CONTROLLER];
	return c;
}

int
Controller::internalExecuteControl(ControllerPtr controller,const std::string& type) {
	static DevicePtr dev;
	controller->clearInput();
	ControlInputPtr input = ControlInput::getControlInput(controller->type());
	if (input) {
		controller->setInput(input);
	}

	Device::current(dev);
	dev->beginUpdate();
	int ret = controller->applyControl(dev);
	dev->finishUpdate();
	CONTROL_OUTPUT_HISTORY[type].push_front(CloningWrapper<Device>(dev));
	setControlOutput(dev);
	return ret;
}

int
Controller::executeInProcessControl() {
	std::string type;
	ControllerPtr controller = getControllerAndType(type);
	if (!controller) {
		return -1;
	}
	if (!controller->inProcess()) {
		return -2;
	}
	int ret = internalExecuteControl(controller,type);
	return ret;
}

int
Controller::executeOutOfProcessControl() {
	static std::string type;
	while (ros::ok()) {
		ControllerPtr controller = getControllerAndType(type);
		if (controller->inProcess()) {
			ros::Duration(0.01).sleep();
			continue;
		}
		int ret = internalExecuteControl(controller,type);
		controller->rate().sleep();
	}
	return 0;
}

DevicePtr
Controller::getControlOutput() {
	boost::mutex::scoped_lock(controlOutputMutex);
	DevicePtr dev;
	dev = CONTROL_OUTPUT;
	return dev;
}

void
Controller::setControlOutput(DevicePtr dev) {
	boost::mutex::scoped_lock(controlOutputMutex);
	CONTROL_OUTPUT = dev;
}

void
Controller::registerController(const std::string& type,ControllerPtr controller) {
	boost::mutex::scoped_lock(controllerMutex);
	CONTROLLERS[type] = controller;
	CONTROL_OUTPUT_HISTORY[type] = History<Device>::Type(CONTROL_OUTPUT_HISTORY_SIZE,0,CloningWrapper<Device>());
}

bool
Controller::setController(const std::string& type) {
	if (RunLevel::get().isPedalDown() && RunLevel::get().isInit()) {
		return false;
	}
	boost::mutex::scoped_lock(controllerMutex);
	Controller::CURRENT_CONTROLLER = type;
	return true;
}

void
Controller::saveState(ControllerStatePtr state) {
	if (history_.capacity() == 0) {
		return;
	} else {
		history_.push_front(state);
	}
}


Controller::Controller(size_t historySize) : history_(historySize,0,ControllerStatePtr()), rate_(new ros::Rate(0)) {

}

int
Controller::applyControl(DevicePtr device) {
	ControllerStatePtr result = internalApplyControl(device);
	result->input = getInput();

	saveState(result);
	return result->returnCode;
}

std::vector<ControllerStatePtr>
Controller::stateHistory(int numSteps) const {
	if (numSteps < 0 || numSteps > (int)history_.size()) {
		numSteps = history_.size();
	}
	std::vector<ControllerStatePtr> hist(numSteps);
	for (int i=0;i<numSteps;i++) {
		hist.push_back(history_[i]);
	}
	return hist;
}

ControllerStatePtr
Controller::lastState() const {
	if (history_.size() > 0) {
		return history_.at(0);
	} else {
		return ControllerStatePtr();
	}
}
