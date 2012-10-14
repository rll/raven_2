/*
 * device.cpp
 *
 *  Created on: Oct 1, 2012
 *      Author: benk
 */

#include <raven/state/device.h>
#include <raven/log.h>

#include <boost/foreach.hpp>

#include <boost/thread/mutex.hpp>

#include <sstream>

static ros::Time theUpdateTime(0);

#define USE_DEVICE_MUTEX
boost::mutex deviceInstanceMutex;
DevicePtr Device::INSTANCE;

//boost::circular_buffer<DevicePtr> Device::HISTORY = boost::circular_buffer<DevicePtr>(DEVICE_HISTORY_SIZE);
History<Device>::Type Device::HISTORY = History<Device>::Type(DEVICE_HISTORY_SIZE);

#define USE_RUNLEVEL_MUTEX
boost::mutex runlevelMutex;
RunLevel* Device::RUNLEVEL = new RunLevel(RunLevel::_E_STOP_HARDWARE_());
bool Device::RUNLEVEL_IS_INITED = false;

RunLevel RunLevel::_E_STOP_() { return RunLevel::_E_STOP_SOFTWARE_(); }
RunLevel RunLevel::_E_STOP_SOFTWARE_() { return RunLevel(0,1); }
RunLevel RunLevel::_E_STOP_HARDWARE_() { return RunLevel(0,0); }
RunLevel RunLevel::_INIT_(runlevel_t sublevel) { return RunLevel(1,sublevel); }
RunLevel RunLevel::_PEDAL_UP_() { return RunLevel(2); }
RunLevel RunLevel::_PEDAL_DOWN_() { return RunLevel(3); }

RunLevel RunLevel::fromNumber(runlevel_t level) { return RunLevel(level,0); }

bool RunLevel::isEstop() const {
	return value_ == 0;
}

bool RunLevel::isHardwareEstop() const {
	return isEstop() && sublevel_ == 0;
}
bool RunLevel::isSoftwareEstop() const {
	return isEstop() && sublevel_ == 1;
}

bool RunLevel::isInit(runlevel_t sublevel) const {
	return value_ == 1 && (sublevel_ == (runlevel_t)-1 || sublevel_ == this->sublevel_);
}

bool RunLevel::isPedalUp() const {
	return value_ == 2;
}

bool RunLevel::isPedalDown() const {
	return value_ == 3;
}

bool RunLevel::isActive() const {
	return value_ >= 2;
}

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

DevicePtr
Device::current() {
	//printf("Current\n");
#ifdef USE_DEVICE_MUTEX
	deviceInstanceMutex.lock();
#endif
	DevicePtr d = Device::INSTANCE->clone();
#ifdef USE_DEVICE_MUTEX
	deviceInstanceMutex.unlock();
#endif
	return d;
}

void
Device::current(DevicePtr& device) {
	//printf("Current\n");
#ifdef USE_DEVICE_MUTEX
	deviceInstanceMutex.lock();
#endif
	Device::INSTANCE->cloneInto(device);
#ifdef USE_DEVICE_MUTEX
	deviceInstanceMutex.unlock();
#endif
}

DevicePtr
Device::currentNoClone() {
	//printf("Current no clone\n");
	return Device::INSTANCE;
}

RunLevel
Device::runlevel() {
#ifdef USE_RUNLEVEL_MUTEX
	runlevelMutex.lock();
#endif
	RunLevel rl(*Device::RUNLEVEL);
#ifdef USE_RUNLEVEL_MUTEX
	runlevelMutex.unlock();
#endif
	return rl;
}
void Device::setRunlevel(RunLevel level) {
#ifdef USE_RUNLEVEL_MUTEX
	runlevelMutex.lock();
#endif
	//if (Device::RUNLEVEL->value_ != level.value_) {
	if (Device::RUNLEVEL->value_ != level.value_ || Device::RUNLEVEL->sublevel_ != level.sublevel_) {
		if (level.isPedalUpOrDown() && Device::RUNLEVEL->isInit()) {
			Device::RUNLEVEL_IS_INITED = true;
		}
		*Device::RUNLEVEL = level;

		log_msg("Entered runlevel %s", Device::RUNLEVEL->str().c_str());
	}
#ifdef USE_RUNLEVEL_MUTEX
	runlevelMutex.unlock();
#endif
}
void Device::setSublevel(runlevel_t sublevel) {
#ifdef USE_RUNLEVEL_MUTEX
	runlevelMutex.lock();
#endif
	if (Device::RUNLEVEL->isInit() && Device::RUNLEVEL->sublevel_ != sublevel) {
		Device::RUNLEVEL->sublevel_ = sublevel;
		log_msg("Entered runlevel %s", Device::RUNLEVEL->str().c_str());
	}
#ifdef USE_RUNLEVEL_MUTEX
	runlevelMutex.unlock();
#endif
}

bool
Device::isInitialized() {
	return Device::RUNLEVEL_IS_INITED;
}

void
Device::eStop() {
	setRunlevel(RunLevel::_E_STOP_SOFTWARE_());
}
bool
Device::getPedal() {
	return Device::runlevel().isPedalDown();
}
void
Device::setPedal(bool down) {
#ifdef USE_RUNLEVEL_MUTEX
	runlevelMutex.lock();
#endif
	if (down && Device::RUNLEVEL->value_ == 2) {
		*Device::RUNLEVEL = RunLevel::_PEDAL_DOWN_();
		log_msg("Entered runlevel %s", Device::RUNLEVEL->str().c_str());
	} else if (!down && Device::RUNLEVEL->value_ == 3) {
		*Device::RUNLEVEL = RunLevel::_PEDAL_UP_();
		log_msg("Entered runlevel %s", Device::RUNLEVEL->str().c_str());
	}
#ifdef USE_RUNLEVEL_MUTEX
	runlevelMutex.unlock();
#endif
}

static int numD = 0;
Device::Device(DeviceType type) : Updateable(), type_(type), timestamp_(0), surgeonMode_(false) {
	//printf("+D  %i %p\n",++numD,this);
}

Device::~Device() {
	//printf("-D  %i %p\n",--numD,this);
}

DevicePtr
Device::clone() const {
	//printf("+D  %i %p CLONING\n",++numD,this);
	DevicePtr newDevice(new Device(*this));
	newDevice->arms_.clear();
	BOOST_FOREACH(ArmPtr arm,arms_) {
		newDevice->arms_.push_back(arm->clone());
	}
	init(newDevice);
	return newDevice;
}

void
Device::cloneInto(DevicePtr& other) const {
	if (!other) {
		DevicePtr newDevice = clone();
		other.swap(newDevice);
		return;
	}
	for (size_t i=0;i<other->arms_.size() && i<arms_.size();i++) {
		arms_[i]->cloneInto(other->arms_[i]);
	}
	for (size_t i=other->arms_.size();i<arms_.size();i++) {
		ArmPtr newArm = arms_[i]->clone();
		other->arms_.push_back(newArm);
	}
	other->arms_.resize(arms_.size());
	init(other);
}

void
Device::init(DevicePtr dev) {
	BOOST_FOREACH(ArmPtr arm, dev->arms_) {
		arm->setUpdateableParent(dev);
	}
}

/*
void
Device::beginCurrentUpdate(ros::Time updateTime) {
	//deviceInstanceMutex.lock(); //pthread_mutex_lock(&deviceInstanceMutex);
	if (updateTime.isZero()) {
		theUpdateTime = Device::INSTANCE->timestamp();
	} else {
		//save to history
		Device::HISTORY.push_front(CloningWrapper<Device>(Device::INSTANCE));
	}
	theUpdateTime = updateTime;
	BOOST_FOREACH(ArmPtr arm,Device::INSTANCE->arms_) {
		arm->holdUpdateBegin();
	}
}

void
Device::finishCurrentUpdate() {
	BOOST_FOREACH(ArmPtr arm,Device::INSTANCE->arms_) {
		arm->motorFilter()->applyUpdate();
		arm->holdUpdateEnd();
	}
	Device::INSTANCE->timestamp_ = theUpdateTime;
	theUpdateTime = ros::Time(0);
	//deviceInstanceMutex.unlock(); //pthread_mutex_unlock(&deviceInstanceMutex);
}
*/
void
Device::beginCurrentUpdate(ros::Time updateTime) {
	//deviceInstanceMutex.lock(); //pthread_mutex_lock(&deviceInstanceMutex);
	if (updateTime.isZero()) {
		theUpdateTime = Device::INSTANCE->timestamp();
	} else {
		//save to history
		Device::HISTORY.push_front(CloningWrapper<Device>(Device::INSTANCE));
	}
	theUpdateTime = updateTime;
	Device::INSTANCE->beginUpdate();
}

void
Device::finishCurrentUpdate() {
	Device::INSTANCE->internalFinishUpdate(false);
	Device::INSTANCE->timestamp_ = theUpdateTime;
	theUpdateTime = ros::Time(0);
	//deviceInstanceMutex.unlock(); //pthread_mutex_unlock(&deviceInstanceMutex);
}

void
Device::beginUpdate() {
	BOOST_FOREACH(ArmPtr arm,Device::INSTANCE->arms_) {
		arm->holdUpdateBegin();
	}
}

void
Device::internalFinishUpdate(bool updateTimestamp) {
	BOOST_FOREACH(ArmPtr arm,Device::INSTANCE->arms_) {
		arm->motorFilter()->applyUpdate();
		arm->holdUpdateEnd();
		if (updateTimestamp && arm->timestamp() > timestamp_) {
			timestamp_ = arm->timestamp();
		}
	}
}

void
Device::finishUpdate() {
	internalFinishUpdate(true);
}

std::vector<DevicePtr>
Device::history(int numSteps) {
	if (numSteps < 0 || numSteps > (int)Device::HISTORY.size()) {
		numSteps = Device::HISTORY.size();
	}
	std::vector<DevicePtr> hist(numSteps);
	for (int i=0;i<numSteps;i++) {
		hist.push_back(Device::HISTORY[i].value);
	}
	return hist;
}

bool
Device::processNotification(Updateable* sender) {
	return true;
}

ArmPtr
Device::getArmById(Arm::IdType id) const {
	for (size_t i=0;i<arms_.size();i++) {
		if (arms_.at(i)->id() == id) {
			return arms_.at(i);
		}
	}
	return ArmPtr();
}
ArmPtr
Device::getArmByName(const std::string& name) const {
	for (size_t i=0;i<arms_.size();i++) {
		if (arms_.at(i)->name() == name) {
			return arms_.at(i);
		}
	}
	return ArmPtr();
}

Eigen::VectorXf
Device::jointPositionVector() const {
	size_t numEl = 0;
	BOOST_FOREACH(ArmPtr arm,arms_) {
		numEl += arm->joints().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	BOOST_FOREACH(ArmPtr arm,arms_) {
		size_t numElInArm = arm->joints().size();
		v.segment(ind,numElInArm) = arm->jointPositionVector();
		ind += numElInArm;
	}
	return v;
}

Eigen::VectorXf
Device::jointVelocityVector() const {
	size_t numEl = 0;
	BOOST_FOREACH(ArmPtr arm,arms_) {
		numEl += arm->joints().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	BOOST_FOREACH(ArmPtr arm,arms_) {
		size_t numElInArm = arm->joints().size();
		v.segment(ind,numElInArm) = arm->jointVelocityVector();
		ind += numElInArm;
	}
	return v;
}

Eigen::VectorXf
Device::motorPositionVector() const {
	size_t numEl = 0;
	BOOST_FOREACH(ArmPtr arm,arms_) {
		numEl += arm->motors().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	BOOST_FOREACH(ArmPtr arm,arms_) {
		size_t numElInArm = arm->motors().size();
		v.segment(ind,numElInArm) = arm->motorPositionVector();
		ind += numElInArm;
	}
	return v;
}

Eigen::VectorXf
Device::motorVelocityVector() const {
	size_t numEl = 0;
	BOOST_FOREACH(ArmPtr arm,arms_) {
		numEl += arm->motors().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	BOOST_FOREACH(ArmPtr arm,arms_) {
		size_t numElInArm = arm->motors().size();
		v.segment(ind,numElInArm) = arm->motorVelocityVector();
		ind += numElInArm;
	}
	return v;
}

Eigen::VectorXf
Device::motorTorqueVector() const {
	size_t numEl = 0;
	BOOST_FOREACH(ArmPtr arm,arms_) {
		numEl += arm->motors().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	BOOST_FOREACH(ArmPtr arm,arms_) {
		size_t numElInArm = arm->motors().size();
		v.segment(ind,numElInArm) = arm->motorTorqueVector();
		ind += numElInArm;
	}
	return v;
}
