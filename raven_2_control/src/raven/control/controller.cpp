/*
 * controller.cpp
 *
 *  Created on: Oct 2, 2012
 *      Author: benk
 */

#include <raven/control/controller.h>
#include <raven/state/runlevel.h>

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
	ControllerPtr c;
	controllerMutex.lock();
	c = CONTROLLERS[CURRENT_CONTROLLER];
	controllerMutex.unlock();
	return c;
}

ControllerPtr
Controller::getControllerAndType(std::string& type) {
	ControllerPtr c;
	controllerMutex.lock();
	type = CURRENT_CONTROLLER;
	c = CONTROLLERS[CURRENT_CONTROLLER];
	controllerMutex.unlock();
	return c;
}

int
Controller::internalExecuteControl(ControllerPtr controller,const std::string& type) {
	static DevicePtr dev;
	controller->clearInput();
	std::vector<std::string> inputTypes = controller->getInputTypes();
	BOOST_FOREACH(std::string type,inputTypes) {
		ControlInputPtr input = ControlInput::getControlInput(type);
		if (input) {
			controller->setInput(type,input);
		}
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
	DevicePtr dev;
	controlOutputMutex.lock();
	dev = CONTROL_OUTPUT;
	controlOutputMutex.unlock();
	return dev;
}

void
Controller::setControlOutput(DevicePtr dev) {
	controlOutputMutex.lock();
	CONTROL_OUTPUT = dev;
	controlOutputMutex.unlock();
}

void
Controller::registerController(const std::string& type,ControllerPtr controller) {
	controllerMutex.lock();
	CONTROLLERS[type] = controller;
	CONTROL_OUTPUT_HISTORY[type] = History<Device>::Type(CONTROL_OUTPUT_HISTORY_SIZE);
	controllerMutex.unlock();
}

bool
Controller::setController(const std::string& type) {
	if (RunLevel::get().isPedalDown() && RunLevel::get().isInit()) {
		return false;
	}
	controllerMutex.lock();
	Controller::CURRENT_CONTROLLER = type;
	controllerMutex.unlock();
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


Controller::Controller(size_t historySize) : history_(historySize), rate_(new ros::Rate(0)) {

}

int
Controller::applyControl(DevicePtr device) {
	ControllerStatePtr state;
	ControllerStatePtr result = internalApplyControl(device);
	saveState(result);
	return result->returnCode;
}
