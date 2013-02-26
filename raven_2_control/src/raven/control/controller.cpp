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

#include <raven/util/timing.h>

#include <set>
#include <stdexcept>

#include "log.h"

#include <boost/thread/mutex.hpp>

using namespace std;

boost::mutex controllerMutex;
boost::mutex controlOutputMutex;

std::map<Arm::IdType,std::string> Controller::CURRENT_CONTROLLERS;
std::map<std::pair<Arm::IdType,std::string>,ControllerPtr> Controller::CONTROLLERS;
std::map<Arm::IdType,DevicePtr> Controller::CONTROL_OUTPUT;

ControllerPtr Controller::HOLD_POSITION_CONTROLLER;
ControllerPtr Controller::getHoldPositionController() {
	if (!HOLD_POSITION_CONTROLLER) {
		throw std::runtime_error("Hold position controller not initialized!");
	}
	return HOLD_POSITION_CONTROLLER;
}


std::map<std::pair<Arm::IdType,std::string>,History<Device>::Type> Controller::CONTROL_OUTPUT_HISTORY;

std::vector<ros::NodeHandle>
Controller::getParameterNodeHandles() {
	if (parameterNodeHandles_.empty()) {
		parameterNodeHandles_.push_back(ros::NodeHandle(name()));
		parameterNodeHandles_.push_back(ros::NodeHandle(type()));
		parameterNodeHandles_.push_back(ros::NodeHandle(""));
	}
	return parameterNodeHandles_;
}

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

int
Controller::internalExecuteControl(std::pair<Arm::IdType,std::string> type, ControllerPtr controller) {
	static DevicePtr dev;
	TRACER_ENTER_SCOPE("Controller::internalExecuteControl()");
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
	static DevicePtr holdPosDev;
	TRACER_ENTER_SCOPE("Controller::executeInProcessControl()");
	TimingInfo t_info;

	Arm::IdList holdPosIds; // = Device::disabledArmIds();
	std::string type;
	ControllerMap controllers = getControllers();
	if (controllers.find(Arm::ALL_ARMS) != controllers.end()) {
		log_err_throttle(0.5,"All-arm control not implemented!");
		//FIXME: all-arm control
	} else {
		Arm::IdList armIds = Device::armIds();
		Arm::IdList::iterator armItr;
		for (armItr = armIds.begin();armItr != armIds.end(); armItr++) {
			ControllerMap::iterator ctrlItr = controllers.find(*armItr);
			if (ctrlItr == controllers.end()) {
				holdPosIds.push_back(*armItr);
			} else {
				TRACER_VERBOSE_PRINT("Controlling arm %i",*armItr);
				std::pair<Arm::IdType,std::string> pair_type(*armItr,ctrlItr->second.first);
				int ctrl_ret = internalExecuteControl(pair_type,ctrlItr->second.second);
			}
		}
	}
	if (!holdPosIds.empty()) {
		t_info.mark_cn_overall_start();

		t_info.mark_cn_get_input_start();
		TRACER_VERBOSE_PRINT("Controlling %u held arms",holdPosIds.size());
		MotorPositionInputPtr holdPosInput(new MotorPositionInput(holdPosIds));

		TRACER_VERBOSE_PRINT("Copying input from device");
		holdPosInput->setFrom(Device::currentNoClone());
		t_info.mark_cn_get_input_end();

		t_info.mark_cn_set_input_start();
		TRACER_VERBOSE_PRINT("Setting input in controller");
		getHoldPositionController()->setInput(holdPosInput);
		t_info.mark_cn_set_input_end();

		t_info.mark_cn_copy_device_start();
		TRACER_VERBOSE_PRINT("Copying current device");
		Device::current(holdPosDev);
		t_info.mark_cn_copy_device_end();

		t_info.mark_cn_ctrl_overall_start();
		t_info.mark_cn_ctrl_begin_start();
		TRACER_VERBOSE_PRINT("Beginning control");
		holdPosDev->beginUpdate();
		t_info.mark_cn_ctrl_begin_end();

		t_info.mark_cn_apply_ctrl_start();
		TRACER_VERBOSE_PRINT("Applying control");
		int ret = getHoldPositionController()->applyControl(holdPosDev);
		t_info.mark_cn_apply_ctrl_end();

		t_info.mark_cn_ctrl_finish_start();
		TRACER_VERBOSE_PRINT("Finishing control");
		holdPosDev->finishUpdate();
		t_info.mark_cn_ctrl_finish_end();
		t_info.mark_cn_ctrl_overall_end();

		t_info.mark_cn_set_output_start();
		Arm::IdList::iterator armItr;
		for (armItr = holdPosIds.begin();armItr != holdPosIds.end(); armItr++) {
			TRACER_VERBOSE_PRINT("Setting control output for arm %s",Device::getArmNameFromId(*armItr).c_str());
			setControlOutput(*armItr,holdPosDev);
		}
		t_info.mark_cn_set_output_end();

		t_info.mark_cn_overall_end();

		if (t_info.cn_overall().toNSec() > 100000000) {
			printf("Ctrl HUGE! [%i]\n",LoopNumber::getMain());

			cout << TimingInfo::cn_get_input_str_padded() << ":\t" << t_info.cn_get_input().toNSec() << endl;
			cout << TimingInfo::cn_set_input_str_padded() << ":\t" << t_info.cn_set_input().toNSec() << endl;
			cout << TimingInfo::cn_copy_device_str_padded() << ":\t" << t_info.cn_copy_device().toNSec() << endl;

			cout << TimingInfo::cn_ctrl_begin_str_padded() << ":\t" << t_info.cn_ctrl_begin().toNSec() << endl;
			cout << TimingInfo::cn_apply_ctrl_str_padded() << ":\t" << t_info.cn_apply_ctrl().toNSec() << endl;
			cout << TimingInfo::cn_ctrl_finish_str_padded() << ":\t" << t_info.cn_ctrl_finish().toNSec() << endl;
			cout << TimingInfo::cn_ctrl_overall_str_padded() << ":\t" << t_info.cn_ctrl_overall().toNSec() << endl;

			cout << TimingInfo::cn_set_output_str_padded() << ":\t" << t_info.cn_set_output().toNSec() << endl;

			cout << TimingInfo::cn_overall_str_padded() << ":\t" << t_info.cn_overall().toNSec() << endl;

			cout << endl;
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
	TRACER_ENTER_SCOPE("Controller::registerController(%i,%s,%s)",armId,type.c_str(),typeid(*controller).name());
	boost::mutex::scoped_lock(controllerMutex);
	if (type.empty()) {
		if (controller) {
			HOLD_POSITION_CONTROLLER = controller;
		} else if (!HOLD_POSITION_CONTROLLER) {
			HOLD_POSITION_CONTROLLER.reset(new MotorPositionPID());
		}
		return;
	}
	if (controller) {
		log_msg("Registering %s controller for arm id %i: %s",type.c_str(),armId,typeid(*controller).name());
	} else {
		log_msg("Unregistering %s controller for arm id %i",type.c_str(),armId);
	}
	std::pair<Arm::IdType,std::string> pair_type(armId,type);
	CONTROLLERS[pair_type] = controller;
	CONTROL_OUTPUT_HISTORY[pair_type] = History<Device>::Type(CONTROL_OUTPUT_HISTORY_SIZE,0,CloningWrapper<Device>());
}

bool
Controller::setController(Arm::IdType armId, const std::string& type) {
	RunLevel rl = RunLevel::get();
	if (rl.isPedalDown() || rl.isInit()) {
		return false;
	}
	boost::mutex::scoped_lock(controllerMutex);
	if (armId == Arm::ALL_ARMS) {
		CURRENT_CONTROLLERS.clear();
		CURRENT_CONTROLLERS[armId] = type;
		if (!type.empty()) {
			HOLD_POSITION_CONTROLLER->resetState();
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
