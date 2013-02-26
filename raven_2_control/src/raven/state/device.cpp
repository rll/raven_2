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
#include <algorithm>

static ros::Time theUpdateTime(0);

bool Device::DEBUG_OUTPUT_TIMING = false;

boost::recursive_mutex deviceInstanceMutex;
DevicePtr Device::INSTANCE;

boost::mutex deviceHistoryMutex;
History<Device>::Type Device::HISTORY = History<Device>::Type(DEVICE_HISTORY_SIZE,0,CloningWrapper<Device>());

Arm::IdList Device::ARM_IDS;
Arm::IdList Device::DISABLED_ARM_IDS;
std::map<Arm::IdType,std::string> Device::ARM_NAMES;
std::map<Arm::IdType,Arm::Type> Device::ARM_TYPES;

std::map<Arm::IdType,size_t> Device::NUM_MOTORS;
size_t Device::TOTAL_NUM_MOTORS = 0;
std::map<Arm::IdType,size_t> Device::NUM_JOINTS;
size_t Device::TOTAL_NUM_JOINTS = 0;

DevicePtr
Device::current() {
	TRACER_VERBOSE_ENTER_SCOPE("Device::current()");
	boost::recursive_mutex::scoped_lock l(deviceInstanceMutex);
	DevicePtr d = Device::INSTANCE->clone();
	return d;
}

void
Device::current(DevicePtr& device) {
	TRACER_VERBOSE_ENTER_SCOPE("Device::current(dev)");
	boost::recursive_mutex::scoped_lock l(deviceInstanceMutex);
	Device::INSTANCE->cloneInto(device);
}

DeviceConstPtr
Device::currentNoClone() {
	TRACER_ENTER_SCOPE("Device::currentNoClone()");
	return Device::INSTANCE;
}

DevicePtr
Device::currentNoCloneMutable() {
	TRACER_VERBOSE_ENTER_SCOPE("Device::currentNoCloneMutable()");
	return Device::INSTANCE;
}

ros::Time
Device::currentTimestamp() {
	ros::Time t;
	boost::recursive_mutex::scoped_lock l(deviceInstanceMutex);
	t = Device::INSTANCE->timestamp_;
	return t;
}

Device::Device(DeviceType type) : Updateable(), type_(type), timestamp_(0) {

}

Device::~Device() {

}

DevicePtr
Device::clone() const {
	//TRACER_VERBOSE_ENTER_SCOPE("Device@%p::clone()",this);
	DevicePtr newDevice(new Device(*this));
	//TRACER_VERBOSE_PRINT("Device clone is %p",newDevice.get());
	newDevice->arms_.clear();
	BOOST_FOREACH(ArmPtr arm,arms_) {
		newDevice->arms_.push_back(arm->clone());
	}
	init(newDevice);
	return newDevice;
}

void
Device::cloneInto(DevicePtr& other) const {
	//TRACER_VERBOSE_ENTER_SCOPE("Device@%p::cloneInto()",this);
	if (!other) {
		DevicePtr newDevice = clone();
		other.swap(newDevice);
		return;
	}
	//TRACER_VERBOSE_PRINT("Device clone is %p",other.get());
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
	TRACER_VERBOSE_ENTER_SCOPE("Device@%p::init()",dev.get());
	BOOST_FOREACH(ArmPtr arm, dev->arms_) {
		arm->setUpdateableParent(dev);
	}
}

DevicePtr
Device::beginCurrentUpdate(ros::Time updateTime) {
	TRACER_ENTER_SCOPE("Device::beginCurrentUpdate()");
	deviceInstanceMutex.lock();
	if (updateTime.isZero()) {
		theUpdateTime = Device::INSTANCE->timestamp();
	} else {
		//save to history
		deviceHistoryMutex.lock();
		Device::HISTORY.push_front(CloningWrapper<Device>(Device::INSTANCE));
		deviceHistoryMutex.unlock();
	}
	theUpdateTime = updateTime;
	Device::INSTANCE->beginUpdate();
	return Device::INSTANCE;
}

void
Device::finishCurrentUpdate() {
	TRACER_ENTER_SCOPE("Device::finishCurrentUpdate()");
	Device::INSTANCE->internalFinishUpdate(false);
	Device::INSTANCE->timestamp_ = theUpdateTime;
	theUpdateTime = ros::Time(0);

	deviceInstanceMutex.unlock();
}

void
Device::beginUpdate() {
	TRACER_ENTER_SCOPE("Device@%p::beginUpdate()",this);
	BOOST_FOREACH(ArmPtr arm,arms_) {
		arm->holdUpdateBegin();
	}
}

void
Device::internalFinishUpdate(bool updateTimestamp) {
	TRACER_ENTER_SCOPE("Device@%p::internalFinishUpdate(%i)",this,updateTimestamp);
	ros::Time start = ros::Time::now();
	BOOST_FOREACH(ArmPtr arm,arms_) {
		if (DEBUG_OUTPUT_TIMING) {
			printf("Device@%p::internalFinishUpdate() arm %i %p %p\n",this,arm->id(),arm->controlMotorFilter().get(),arm->stateMotorFilter().get());
		}
		ros::Time arm1 = ros::Time::now();
		ros::Time smf1 = arm1;
		arm->stateMotorFilter()->applyUpdate();
		ros::Time smf2 = ros::Time::now();

		ros::Time cmf1 = smf2;
		arm->controlMotorFilter()->applyUpdate();
		ros::Time cmf2 = ros::Time::now();

		ros::Time hue1 = cmf2;
		arm->holdUpdateEnd();
		ros::Time hue2 = ros::Time::now();

		ros::Time arm2 = hue2;

		if (updateTimestamp && arm->timestamp() > timestamp_) {
			timestamp_ = arm->timestamp();
		}

		if (DEBUG_OUTPUT_TIMING) {
			printf("Arm %i:\t%8lli\n",arm->id(),(long long int)(arm2-arm1).toNSec());
			printf(" smf:\t%8lli\n",(long long int)(smf2-smf1).toNSec());
			printf(" cmf:\t%8lli\n",(long long int)(cmf2-cmf1).toNSec());
			printf(" hue:\t%8lli\n",(long long int)(hue2-hue1).toNSec());
		}
	}
	ros::Time end = ros::Time::now();
	if (DEBUG_OUTPUT_TIMING) {
		printf("Total:\t%8lli\n",(long long int)(end-start).toNSec());
	}
}

void
Device::finishUpdate() {
	TRACER_ENTER_SCOPE("Device@%p::finishUpdate()",this);
	internalFinishUpdate(true);
}

std::vector<DevicePtr>
Device::history(int numSteps) {
	std::vector<DevicePtr> hist;
		deviceHistoryMutex.lock();
	if (numSteps < 0 || numSteps > (int)Device::HISTORY.size()) {
		numSteps = Device::HISTORY.size();
	}
	hist.reserve(numSteps);
	for (int i=0;i<numSteps;i++) {
		hist.push_back(Device::HISTORY[i].value);
	}
		deviceHistoryMutex.unlock();
	return hist;
}

bool
Device::processNotification(Updateable* sender) {
	return true;
}

void
Device::addArm(ArmPtr arm) {
	if (arm->enabled()) {
		arms_.push_back(arm);
		ARM_IDS.push_back(arm->id());
		TOTAL_NUM_JOINTS += arm->joints().size();
		TOTAL_NUM_MOTORS += arm->motors().size();
	} else {
		disabledArms_.push_back(arm);
		DISABLED_ARM_IDS.push_back(arm->id());
	}
	ARM_NAMES[arm->id()] = arm->name();
	ARM_TYPES[arm->id()] = arm->type();
	NUM_JOINTS[arm->id()] = arm->joints().size();
	NUM_MOTORS[arm->id()] = arm->motors().size();
}



JointPtr
Device::getJointByOldType(int type) {
	JointPtr joint;
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);
	ArmPtr arm = getArmById(arm_id);
	return arm->getJointByOldType(type);
}

JointConstPtr
Device::getJointByOldType(int type) const {
	return JointConstPtr(const_cast<Device*>(this)->getJointByOldType(type));
}
