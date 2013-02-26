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

#define USE_DEVICE_INSTANCE_MUTEX
boost::recursive_mutex deviceInstanceMutex;
DevicePtr Device::INSTANCE;

#define USE_DEVICE_HISTORY_MUTEX
boost::mutex deviceHistoryMutex;
History<Device>::Type Device::HISTORY = History<Device>::Type(DEVICE_HISTORY_SIZE,0,CloningWrapper<Device>());

Arm::IdList Device::ARM_IDS;
Arm::IdList Device::DISABLED_ARM_IDS;
std::map<std::string,Arm::IdType> Device::ARM_NAMES;

std::map<Arm::IdType,size_t> Device::NUM_MOTORS;
size_t Device::TOTAL_NUM_MOTORS = 0;
std::map<Arm::IdType,size_t> Device::NUM_JOINTS;
size_t Device::TOTAL_NUM_JOINTS = 0;

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
Device::Device(DeviceType type) : Updateable(), type_(type), timestamp_(0) {
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
	ARM_NAMES[arm->name()] = arm->id();
	NUM_JOINTS[arm->id()] = arm->joints().size();
	NUM_MOTORS[arm->id()] = arm->motors().size();
}

size_t
Device::numArms() {
	return ARM_IDS.size();
}

std::vector<Arm::IdType>
Device::armIds() {
	return ARM_IDS;
}

ArmList
Device::arms() const {
	return arms_;
}

ArmPtr
Device::arm(size_t i) const {
	return arms_[i];
}

ArmPtr
Device::getArmById(Arm::IdType id) const {
	ArmList::const_iterator itr;
	for (itr=arms_.begin();itr!=arms_.end();itr++) {
		if ((*itr)->id() == id) {
			return *itr;
		}
	}
	for (itr=disabledArms_.begin();itr!=disabledArms_.end();itr++) {
		if ((*itr)->id() == id) {
			return *itr;
		}
	}
	return ArmPtr();
}

ArmList
Device::getArmsById(const Arm::IdList& ids,bool includeDisabled) const {
	ArmList arms;
	bool includeAll = std::find(ids.begin(),ids.end(),Arm::ALL_ARMS) != ids.end();
	ArmList::const_iterator itr;
	for (itr=arms_.begin();itr!=arms_.end();itr++) {
		if (includeAll || std::find(ids.begin(),ids.end(),(*itr)->id()) != ids.end()) {
			arms.push_back(*itr);
		}
	}
	if (includeDisabled) {
		for (itr=disabledArms_.begin();itr!=disabledArms_.end();itr++) {
			if (includeAll || std::find(ids.begin(),ids.end(),(*itr)->id()) != ids.end()) {
				arms.push_back(*itr);
			}
		}
	}
	return arms;
}

ArmPtr
Device::getArmByName(const std::string& name) const {
	ArmList::const_iterator itr;
	for (itr=arms_.begin();itr!=arms_.end();itr++) {
		if ((*itr)->name() == name) {
			return *itr;
		}
	}
	for (itr=disabledArms_.begin();itr!=disabledArms_.end();itr++) {
		if ((*itr)->name() == name) {
			return *itr;
		}
	}
	return ArmPtr();
}

Arm::IdType
Device::getArmIdByName(const std::string& name) {
	std::map<std::string,Arm::IdType>::const_iterator itr = ARM_NAMES.find(name);
	if (itr == ARM_NAMES.end()) {
		std::stringstream ss;
		ss << "Arm name " << name << " not found!";
		throw std::runtime_error(ss.str());
	}
	return itr->second;
}

size_t
Device::numDisabledArms() {
	return DISABLED_ARM_IDS.size();
}

Arm::IdList
Device::disabledArmIds() {
	return DISABLED_ARM_IDS;
}

ArmList
Device::disabledArms() const {
	return disabledArms_;
}

size_t
Device::numAllArms() {
	return numArms() + numDisabledArms();
}

Arm::IdList
Device::allArmIds() {
	Arm::IdList armIds = ARM_IDS;
	armIds.insert(armIds.end(),DISABLED_ARM_IDS.begin(),DISABLED_ARM_IDS.end());
	return armIds;
}


ArmList
Device::allArms() const {
	ArmList arms = arms_;
	arms.insert(arms.end(),disabledArms_.begin(),disabledArms_.end());
	return arms;
}

size_t
Device::numJoints() {
	return TOTAL_NUM_JOINTS;
}
size_t
Device::numJointsOnArm(size_t i) {
	return NUM_JOINTS[ARM_IDS[i]];
}
size_t Device::numJointsOnArmById(Arm::IdType id) {
	return NUM_JOINTS[id];
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

size_t
Device::numMotors() {
	return TOTAL_NUM_MOTORS;
}
size_t
Device::numMotorsOnArm(size_t i) {
	return NUM_MOTORS[ARM_IDS[i]];
}
size_t Device::numMotorsOnArmById(Arm::IdType id) {
	return NUM_MOTORS[id];
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
