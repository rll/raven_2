/*
 * device.cpp
 *
 *  Created on: Oct 1, 2012
 *      Author: benk
 */

#include <raven/state/device.h>
#include "log.h"

#include "defines.h"

#include <boost/foreach.hpp>

#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <sstream>

static ros::Time theUpdateTime(0);

#define USE_DEVICE_INSTANCE_MUTEX
boost::recursive_mutex deviceInstanceMutex;
DevicePtr Device::INSTANCE;

#define USE_DEVICE_HISTORY_MUTEX
boost::mutex deviceHistoryMutex;
History<Device>::Type Device::HISTORY = History<Device>::Type(DEVICE_HISTORY_SIZE,0,CloningWrapper<Device>());

DevicePtr
Device::current() {
#ifdef USE_DEVICE_INSTANCE_MUTEX
	boost::recursive_mutex::scoped_lock l(deviceInstanceMutex);
#endif
	DevicePtr d = Device::INSTANCE->clone();
	return d;
}

void
Device::current(DevicePtr& device) {
#ifdef USE_DEVICE_INSTANCE_MUTEX
	boost::recursive_mutex::scoped_lock l(deviceInstanceMutex);
#endif
	Device::INSTANCE->cloneInto(device);
}

DevicePtr
Device::currentNoClone() {
	return Device::INSTANCE;
}

ros::Time
Device::currentTimestamp() {
	ros::Time t;
#ifdef USE_DEVICE_INSTANCE_MUTEX
	boost::recursive_mutex::scoped_lock l(deviceInstanceMutex);
#endif
	t = Device::INSTANCE->timestamp_;
	return t;
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
DevicePtr
Device::beginCurrentUpdate(ros::Time updateTime) {
#ifdef USE_DEVICE_INSTANCE_MUTEX
	deviceInstanceMutex.lock();
#endif
	if (updateTime.isZero()) {
		theUpdateTime = Device::INSTANCE->timestamp();
	} else {
		//save to history
#ifdef USE_DEVICE_HISTORY_MUTEX
		deviceHistoryMutex.lock();
#endif
		Device::HISTORY.push_front(CloningWrapper<Device>(Device::INSTANCE));
#ifdef USE_DEVICE_HISTORY_MUTEX
		deviceHistoryMutex.unlock();
#endif
	}
	theUpdateTime = updateTime;
	Device::INSTANCE->beginUpdate();
	return Device::INSTANCE;
}

void
Device::finishCurrentUpdate() {
	Device::INSTANCE->internalFinishUpdate(false);
	Device::INSTANCE->timestamp_ = theUpdateTime;
	theUpdateTime = ros::Time(0);

#ifdef USE_DEVICE_INSTANCE_MUTEX
	deviceInstanceMutex.unlock();
#endif
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
	std::vector<DevicePtr> hist;
#ifdef USE_DEVICE_HISTORY_MUTEX
		deviceHistoryMutex.lock();
#endif
	if (numSteps < 0 || numSteps > (int)Device::HISTORY.size()) {
		numSteps = Device::HISTORY.size();
	}
	hist.reserve(numSteps);
	for (int i=0;i<numSteps;i++) {
		hist.push_back(Device::HISTORY[i].value);
	}
#ifdef USE_DEVICE_HISTORY_MUTEX
		deviceHistoryMutex.unlock();
#endif
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

JointPtr
Device::getJointByOldType(int type) const {
	JointPtr joint;
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);
	ArmPtr arm = getArmById(arm_id);
	return arm->getJointByOldType(type);
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
