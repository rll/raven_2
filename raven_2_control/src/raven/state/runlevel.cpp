/*
 * runlevel.cpp
 *
 *  Created on: Oct 14, 2012
 *      Author: benk
 */

#include <raven/state/runlevel.h>

#include "log.h"
#include <ros/ros.h>

#include <raven/state/device.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/tss.hpp>

#include "USB_init.h"
extern USBStruct USBBoards;

//std::atomic<unsigned int> RunLevel::LOOP_NUMBER(0);

#define USE_RUNLEVEL_MUTEX
boost::recursive_mutex runlevelMutex;
RunLevel* RunLevel::PREVIOUS_RUNLEVEL = new RunLevel(RunLevel::_E_STOP_HARDWARE_());
RunLevel* RunLevel::INSTANCE = new RunLevel(RunLevel::_E_STOP_HARDWARE_());
bool RunLevel::PEDAL = false;
bool RunLevel::SOFTWARE_ESTOP = false;
bool RunLevel::IS_INITED = false;
bool RunLevel::HAS_HOMED = false;
int RunLevel::FIRST_HOMED_LOOP = -1;
std::map<int,bool> RunLevel::ARMS_ACTIVE;

RunLevel RunLevel::_E_STOP_() { return RunLevel::_E_STOP_SOFTWARE_(); }
RunLevel RunLevel::_E_STOP_SOFTWARE_() { return RunLevel(0,1); }
RunLevel RunLevel::_E_STOP_HARDWARE_() { return RunLevel(0,0); }
RunLevel RunLevel::_INIT_(runlevel_t sublevel) { return RunLevel(1,sublevel); }
RunLevel RunLevel::_PEDAL_UP_() { return RunLevel(2); }
RunLevel RunLevel::_PEDAL_DOWN_() { return RunLevel(3); }

void RunLevel::updateRunlevel(runlevel_t level) {
	boost::recursive_mutex::scoped_lock _l(runlevelMutex);
	int loop = LoopNumber::getMain();
	*PREVIOUS_RUNLEVEL = *INSTANCE;
	if (SOFTWARE_ESTOP) {
		setInitialized(false);
		if (level != 0) {
			*INSTANCE = _E_STOP_SOFTWARE_();
		} else {
			err_msg("*** ENTERED SOFTWARE E-STOP STATE [%i] ***\n",loop);
			*INSTANCE = _E_STOP_HARDWARE_();
			SOFTWARE_ESTOP = false;
		}
	} else if (INSTANCE->value_ != level) {
		if (ROS_UNLIKELY(!HAS_HOMED) && level >= 2) {
			log_msg("Homing completed [%i]",loop);
			HAS_HOMED = true;
			FIRST_HOMED_LOOP = loop;
		}
		*INSTANCE = RunLevel(level,0);
		if (level == 0) {
			err_msg("*** ENTERED E-STOP STATE [%i] ***\n",loop);
		} else {
			log_msg("Entered runlevel %s [%i]", INSTANCE->str().c_str(),loop);
		}
	}
}

bool RunLevel::isEstop() const {
	return value_ == 0;
}

bool RunLevel::isHardwareEstop() const {
	return isEstop() && sublevel_ == 0;
}
bool RunLevel::isSoftwareEstop() const {
	return isEstop() && sublevel_ == 1;
}

bool RunLevel::isInit() const {
	return value_ == 1;
}

bool RunLevel::isInitSublevel(const runlevel_t& sublevel) const {
	return value_ == 1 && sublevel == sublevel_;
}

bool RunLevel::isPedalUp() const {
	return value_ == 2;
}

bool RunLevel::isPedalDown() const {
	return value_ == 3;
}

/*
bool RunLevel::isActive() const {
	return value_ >= 2;
}
*/

std::string
RunLevel::str() const {
	switch (value_) {
	case 0:
		return "E_STOP";
	case 1:
		switch (sublevel_) {
		case 0:
			return "INIT:0";
		case 1:
			return "INIT:1";
		case 2:
			return "INIT:2";
		case 3:
			return "INIT:3";
		default:
			std::stringstream ss;
			ss << "INIT:UNKNOWN[" << sublevel_ << "]";
			return ss.str();
		}
		break;
	case 2:
		return "PEDAL_UP";
	case 3:
		return "PEDAL_DOWN";
	default:
		std::stringstream ss;
		ss << "UNKNOWN[" << value_ << "]";
		return ss.str();
	}
}

RunLevel
RunLevel::get() {
	boost::recursive_mutex::scoped_lock _l(runlevelMutex);
	RunLevel rl(*INSTANCE);
	rl.armsActive_ = ARMS_ACTIVE;
	return rl;
}

void RunLevel::setSublevel(runlevel_t sublevel) {
	boost::recursive_mutex::scoped_lock _l(runlevelMutex);
	int loop = LoopNumber::getMain();
	if (INSTANCE->isInit() && INSTANCE->sublevel_ != sublevel) {
		INSTANCE->sublevel_ = sublevel;
		log_msg("Entered runlevel %s [%i]", INSTANCE->str().c_str(),loop);
	}
}

bool
RunLevel::changed(bool checkSublevel) {
	boost::recursive_mutex::scoped_lock _l(runlevelMutex);
	return PREVIOUS_RUNLEVEL->value_ != INSTANCE->value_ || (checkSublevel && PREVIOUS_RUNLEVEL->sublevel_ != INSTANCE->sublevel_);
}

bool
RunLevel::hasHomed() {
	boost::recursive_mutex::scoped_lock _l(runlevelMutex);
	return HAS_HOMED;
}

bool
RunLevel::newlyHomed() {
	boost::recursive_mutex::scoped_lock _l(runlevelMutex);
	if (FIRST_HOMED_LOOP == -1) {
		return false;
	}
	int loop = LoopNumber::getMain();
	return loop == FIRST_HOMED_LOOP;
}

bool
RunLevel::isInitialized() {
	boost::recursive_mutex::scoped_lock _l(runlevelMutex);
	return IS_INITED;
}

void
RunLevel::setInitialized(bool value) {
	boost::recursive_mutex::scoped_lock _l(runlevelMutex);
	IS_INITED = value;
}

void
RunLevel::eStop() {
	boost::recursive_mutex::scoped_lock _l(runlevelMutex);
	SOFTWARE_ESTOP = true;
}

void
RunLevel::setPedalUp() {
	boost::recursive_mutex::scoped_lock _l(runlevelMutex);
	ARMS_ACTIVE.clear();
}

bool
RunLevel::getPedal() {
	bool pedal = false;
	boost::recursive_mutex::scoped_lock _l(runlevelMutex);
	//pedal = RunLevel::PEDAL;
	std::map<int,bool>::const_iterator itr;
	for (itr = ARMS_ACTIVE.begin();itr!=ARMS_ACTIVE.end();itr++) {
		if (itr->second) {
			pedal = true;
			break;
		}
	}
	return pedal;
}
void
RunLevel::setArmActive(int armId,bool active) {
	boost::recursive_mutex::scoped_lock _l(runlevelMutex);
	if (armId == Arm::ALL_ARMS) {
#ifdef USE_NEW_DEVICE
        FOREACH_ARM_ID(armId) {
#else
		for (std::vector<int>::iterator itr=USBBoards.boards.begin();itr!=USBBoards.boards.end();itr++) {
			int armId = *itr;
#endif
			RunLevel::ARMS_ACTIVE[armId] = active;
		}
	} else {
		RunLevel::ARMS_ACTIVE[armId] = active;
	}
}
bool
RunLevel::isArmActive(int armId) const {
	boost::recursive_mutex::scoped_lock _l(runlevelMutex);
	if (!isPedalDown()) {
		return false;
	}
	std::map<int,bool>::const_iterator itr = armsActive_.find(armId);
	if (itr == armsActive_.end()) {
		return false;
	} else {
		return itr->second;
	}
}

boost::mutex loopNumberMutex;

std::map<std::string,int> LoopNumber::NAMED_INTERVALS;
boost::thread_specific_ptr< std::map<std::string,LoopNumber::CountInfo> > LoopNumber::NAMED_COUNTS;

boost::thread_specific_ptr<int> LoopNumber::LOOP_NUMBER;
int LoopNumber::MAIN_LOOP_NUMBER = -1;

//#define USE_LOOP_MUTEX_FOR_ALL
#define USE_LOOP_MUTEX_FOR_NAMED_INTERVALS

int
LoopNumber::internalGet() {
	if (ROS_LIKELY(!LOOP_NUMBER.get())) {
		LOOP_NUMBER.reset(new int(-1));
	}
	return *LOOP_NUMBER;
}

int
LoopNumber::get() {
#ifdef USE_LOOP_MUTEX_FOR_ALL
	boost::mutex::scoped_lock _l(loopNumberMutex);
#endif
	return internalGet();
}

int
LoopNumber::getMain() {
	boost::mutex::scoped_lock _l(loopNumberMutex);
	return MAIN_LOOP_NUMBER;
}

void
LoopNumber::increment(int amt) {
#ifdef USE_LOOP_MUTEX_FOR_ALL
	boost::mutex::scoped_lock _l(loopNumberMutex);
#endif
	if (ROS_UNLIKELY(!LOOP_NUMBER.get())) {
		LOOP_NUMBER.reset(new int(-1));
	}
	*LOOP_NUMBER += amt;
}

void
LoopNumber::incrementMain(int amt) {
	boost::mutex::scoped_lock _l(loopNumberMutex);
	if (ROS_UNLIKELY(!LOOP_NUMBER.get())) {
		LOOP_NUMBER.reset(new int(-1));
	}
	*LOOP_NUMBER += amt;
	MAIN_LOOP_NUMBER += amt;
}


bool
LoopNumber::is(int loop) {
	return get() == loop;
}

bool
LoopNumber::after(int loop) {
	return get() >= loop;
}

bool
LoopNumber::before(int loop) {
	return get() < loop;
}

bool
LoopNumber::between(int start_inclusive,int end_exclusive) {
	int loop = get();
	return start_inclusive <= loop && loop < end_exclusive;
}

bool
LoopNumber::every(int interval) {
	return get() % interval == 0;
}

bool
LoopNumber::everyMain(int interval) {
	return getMain() % interval == 0;
}

void
LoopNumber::setNamedInterval(const std::string& name,int interval) {
#if defined USE_LOOP_MUTEX_FOR_ALL || defined USE_LOOP_MUTEX_FOR_NAMED_INTERVALS
	boost::mutex::scoped_lock _l(loopNumberMutex);
#endif
	NAMED_INTERVALS[name] = interval;
}

int
LoopNumber::internalGetNamedInterval(const std::string& name) {
	std::map<std::string,int>::const_iterator itr = NAMED_INTERVALS.find(name);
	if (itr == NAMED_INTERVALS.end()) {
		return -1;
	} else {
		return itr->second;
	}
}

int
LoopNumber::getNamedInterval(const std::string& name) {
#if defined USE_LOOP_MUTEX_FOR_ALL || defined USE_LOOP_MUTEX_FOR_NAMED_INTERVALS
	boost::mutex::scoped_lock _l(loopNumberMutex);
#endif
	return internalGetNamedInterval(name);
}

bool
LoopNumber::every(const std::string& name) {
#if defined USE_LOOP_MUTEX_FOR_ALL || defined USE_LOOP_MUTEX_FOR_NAMED_INTERVALS
	boost::mutex::scoped_lock _l(loopNumberMutex);
#endif
	int loop = MAIN_LOOP_NUMBER;
	int interval = internalGetNamedInterval(name);
	return interval != -1 && loop % interval == 0;
}

bool
LoopNumber::everyMain(const std::string& name) {
#if defined USE_LOOP_MUTEX_FOR_ALL || defined USE_LOOP_MUTEX_FOR_NAMED_INTERVALS
	boost::mutex::scoped_lock _l(loopNumberMutex);
#endif
	int loop = internalGet();
	int interval = internalGetNamedInterval(name);
	return interval != -1 && loop % interval == 0;
}


bool
LoopNumber::once(const std::string& name) {
	return only(name,1);
}

LoopNumber::CountInfo
LoopNumber::internalGetNamedCount(const std::string& name) {
	int loop = internalGet();
	if (ROS_UNLIKELY(!NAMED_COUNTS.get())) {
		NAMED_COUNTS.reset(new std::map<std::string,CountInfo>());
	}
	CountInfo count;
	std::map<std::string,CountInfo>::const_iterator itr = NAMED_COUNTS->find(name);
	if (ROS_LIKELY(itr != NAMED_COUNTS->end())) {
		count = itr->second;
	} else {
		count.count = 1;
		count.loop = loop;
	}
	if (count.loop != loop) {
		count.count += 1;
		count.loop = loop;
	}
	(*NAMED_COUNTS)[name] = count;
	return count;
}

int
LoopNumber::getNamedCount(const std::string& name) {
#ifdef USE_LOOP_MUTEX_FOR_ALL
	boost::mutex::scoped_lock _l(loopNumberMutex);
#endif
	CountInfo count = internalGetNamedCount(name);
	return count.count;
}

bool
LoopNumber::only(const std::string& name, int limit) {
#ifdef USE_LOOP_MUTEX_FOR_ALL
	boost::mutex::scoped_lock _l(loopNumberMutex);
#endif
	CountInfo count = internalGetNamedCount(name);
	return count.count <= limit;
}

bool
LoopNumber::onlyAfter(const std::string& name,int min) {
#ifdef USE_LOOP_MUTEX_FOR_ALL
	boost::mutex::scoped_lock _l(loopNumberMutex);
#endif
	CountInfo count = internalGetNamedCount(name);
	return count.count > min;
}

bool
LoopNumber::internalOnlyEvery(const std::string& name, int limit, int interval) {
	if (interval == -1) {
		return false;
	}
	int loop = internalGet();
	if (loop % interval != 0) {
		return false;
	}
	return only(name,limit);
}

bool
LoopNumber::internalOnlyEveryAfter(const std::string& name, int min, int interval) {
	if (interval == -1) {
		return false;
	}
	int loop = internalGet();
	if (loop % interval != 0) {
		return false;
	}
	return onlyAfter(name,min);
}

bool
LoopNumber::onlyEvery(const std::string& name, int limit) {
#if defined USE_LOOP_MUTEX_FOR_ALL || defined USE_LOOP_MUTEX_FOR_NAMED_INTERVALS
	boost::mutex::scoped_lock _l(loopNumberMutex);
#endif
	int interval = internalGetNamedInterval(name);
	return internalOnlyEvery(name,limit,interval);
}

bool
LoopNumber::onlyEvery(const std::string& name, int limit, int interval) {
#ifdef USE_LOOP_MUTEX_FOR_ALL
	boost::mutex::scoped_lock _l(loopNumberMutex);
#endif
	return internalOnlyEvery(name,limit,interval);
}

bool
LoopNumber::onlyEveryAfter(const std::string& name, int min) {
#if defined USE_LOOP_MUTEX_FOR_ALL || defined USE_LOOP_MUTEX_FOR_NAMED_INTERVALS
	boost::mutex::scoped_lock _l(loopNumberMutex);
#endif
	int interval = internalGetNamedInterval(name);
	return internalOnlyEveryAfter(name,min,interval);
}

bool
LoopNumber::onlyEveryAfter(const std::string& name, int min, int interval) {
#ifdef USE_LOOP_MUTEX_FOR_ALL
	boost::mutex::scoped_lock _l(loopNumberMutex);
#endif
	return internalOnlyEveryAfter(name,min,interval);
}
