/*
 * controller.cpp
 *
 *  Created on: Oct 2, 2012
 *      Author: benk
 */

#include <raven/control/controller.h>
#include <raven/state/runlevel.h>
#include <raven/control/controllers/motor_position_pid.h>
#include <raven/control/input/motor_input.h>

#include <set>

#include "log.h"

#include <boost/thread/mutex.hpp>

boost::mutex controllerMutex;
boost::mutex controlOutputMutex;

std::map<Arm::IdType,std::string> Controller::CURRENT_CONTROLLERS;
std::map<std::pair<Arm::IdType,std::string>,ControllerPtr> Controller::CONTROLLERS;
std::map<Arm::IdType,DevicePtr> Controller::CONTROL_OUTPUT;

ControllerPtr Controller::HOLD_POSITION_CONTROLLER;
ControllerPtr Controller::getHoldPositionController() {
	if (!HOLD_POSITION_CONTROLLER) {
		HOLD_POSITION_CONTROLLER.reset(new MotorPositionPID());
	}
	return HOLD_POSITION_CONTROLLER;
}


#define CONTROL_OUTPUT_HISTORY_SIZE 50
std::map<std::pair<Arm::IdType,std::string>,History<Device>::Type> Controller::CONTROL_OUTPUT_HISTORY;

ControllerMap
Controller::getControllers() {
	boost::mutex::scoped_lock(controllerMutex);
	ControllerMap controllers;
	std::map<Arm::IdType,std::string>::iterator itr;
	for (itr=CURRENT_CONTROLLERS.begin();itr!=CURRENT_CONTROLLERS.end();itr++) {
		controllers[itr->first] = std::pair<std::string,ControllerPtr>(itr->second,CONTROLLERS[*itr]);
	}
	return controllers;
}

/*
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
*/

int
Controller::internalExecuteControl(std::pair<Arm::IdType,std::string> type, ControllerPtr controller) {
	static DevicePtr dev;
	controller->clearInput();
	ControlInputPtr input = ControlInput::getControlInput(type.first,controller->type());
	if (input) {
		controller->setInput(input);
	}

	Device::current(dev);
	dev->beginUpdate();
	int ret = controller->applyControl(dev);
	dev->finishUpdate();
	CONTROL_OUTPUT_HISTORY[type].push_front(CloningWrapper<Device>(dev));
	setControlOutput(type.first,dev);
	return ret;
}

int
Controller::executeInProcessControl() {
	//static MotorPositionInputPtr holdPosInput = createSeparateArmControlInput<MotorPositionInput>();
	static DevicePtr holdPosDev;
	Arm::IdList holdPosIds = Device::disabledArmIds();
	std::string type;
	ControllerMap controllers = getControllers();
	if (controllers.find(Arm::ALL_ARMS) != controllers.end()) {
		//FIXME: all-arm control
	} else {
		Arm::IdList armIds = Device::armIds();
		Arm::IdList::iterator armItr;
		for (armItr = armIds.begin();armItr != armIds.end(); armItr++) {
			ControllerMap::iterator ctrlItr = controllers.find(*armItr);
			if (ctrlItr == controllers.end()) {
				holdPosIds.push_back(*armItr);
			} else {
				std::pair<Arm::IdType,std::string> pair_type(*armItr,ctrlItr->second.first);
				int ctrl_ret = internalExecuteControl(pair_type,ctrlItr->second.second);
			}
		}
	}
	if (!holdPosIds.empty()) {
		MotorPositionInputPtr holdPosInput(new MotorPositionInput(holdPosIds));
		holdPosInput->setFrom(Device::currentNoClone());
		getHoldPositionController()->setInput(holdPosInput);
		Device::current(holdPosDev);
		holdPosDev->beginUpdate();
		int ret = getHoldPositionController()->applyControl(holdPosDev);
		holdPosDev->finishUpdate();
		Arm::IdList::iterator armItr;
		for (armItr = holdPosIds.begin();armItr != holdPosIds.end(); armItr++) {
			setControlOutput(*armItr,holdPosDev);
		}
	}
	return 0;
}

int
Controller::executeOutOfProcessControl() {
	static std::string type;
	while (ros::ok()) {
		//FIXME: how to do this?
		/*
		ControllerPtr controller = getControllerAndType(type);
		if (controller->inProcess()) {
			ros::Duration(0.01).sleep();
			continue;
		}
		int ret = internalExecuteControl(controller,type);
		controller->rate().sleep();
		*/
		ros::Duration(0.01).sleep();
	}
	return 0;
}

std::map<Arm::IdType,DevicePtr>
Controller::getControlOutput() {
	boost::mutex::scoped_lock(controlOutputMutex);
	return CONTROL_OUTPUT;
}

DevicePtr
Controller::getControlOutput(Arm::IdType armId) {
	boost::mutex::scoped_lock(controlOutputMutex);
	return CONTROL_OUTPUT[armId];
}

void
Controller::setControlOutput(Arm::IdType armId,DevicePtr dev) {
	boost::mutex::scoped_lock(controlOutputMutex);
	if (armId == Arm::ALL_ARMS) {
		FOREACH_ARM_ID_IN_LIST(armId,Device::armIds()) {
			CONTROL_OUTPUT[armId] = dev;
		}
	} else {
		CONTROL_OUTPUT[armId] = dev;
	}
}

void
Controller::registerController(Arm::IdType armId, const std::string& type,ControllerPtr controller) {
	boost::mutex::scoped_lock(controllerMutex);
	std::pair<Arm::IdType,std::string> pair_type(armId,type);
	CONTROLLERS[pair_type] = controller;
	CONTROL_OUTPUT_HISTORY[pair_type] = History<Device>::Type(CONTROL_OUTPUT_HISTORY_SIZE,0,CloningWrapper<Device>());
}

bool
Controller::setController(Arm::IdType armId, const std::string& type) {
	if (RunLevel::get().isPedalDown() && RunLevel::get().isInit()) {
		return false;
	}
	boost::mutex::scoped_lock(controllerMutex);
	if (armId == Arm::ALL_ARMS) {
		CURRENT_CONTROLLERS.clear();
		CURRENT_CONTROLLERS[armId] = type;
		if (!type.empty()) {
			HOLD_POSITION_CONTROLLER->resetState();
//			FOREACH_ARM_ID(armId) {
//				getHoldPositionController(armId)->resetState();
//			}
		}

		return true;
	} else {
		CURRENT_CONTROLLERS.erase(Arm::ALL_ARMS);
	}
	std::map<Arm::IdType,std::string>::iterator itr = CURRENT_CONTROLLERS.find(armId);
	if (type.empty() && itr!=CURRENT_CONTROLLERS.end()) {
		CURRENT_CONTROLLERS.erase(itr);
	} else if (itr->second != type){
		itr->second = type;
		HOLD_POSITION_CONTROLLER->resetState();
//		getHoldPositionController(armId)->resetState();
	}
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


Controller::Controller(size_t historySize) : history_(historySize,0,ControllerStatePtr()), reset_(false) {

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
