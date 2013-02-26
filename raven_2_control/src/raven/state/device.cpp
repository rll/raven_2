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

static int numD = 0;
Device::Device(DeviceType type) : Updateable(), type_(type), timestamp_(0) {
	//printf("+D  %i %p\n",++numD,this);
}

Device::~Device() {
	//printf("-D  %i %p\n",--numD,this);
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
	BOOST_FOREACH(ArmPtr arm,arms_) {
		arm->stateMotorFilter()->applyUpdate();
		arm->controlMotorFilter()->applyUpdate();
		arm->holdUpdateEnd();
		if (updateTimestamp && arm->timestamp() > timestamp_) {
			timestamp_ = arm->timestamp();
		}
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

size_t
Device::numArms() {
	return ARM_IDS.size();
}

Arm::IdList
Device::armIds() {
	return ARM_IDS;
}

Arm::IdList&
Device::sortArmIds(Arm::IdList& armIds) {
	Arm::IdSet idSet = Arm::idSet(armIds);
	armIds = Device::sortArmIds(idSet);
	return armIds;
}

Arm::IdList
Device::sortArmIds(const Arm::IdSet& armIdSet) {
	Arm::IdList allIds = allArmIds();
	Arm::IdList sorted;
	for (size_t i=0;i<allIds.size();i++) {
		Arm::IdType id = allIds.at(i);
		if (armIdSet.count(id)) {
			sorted.push_back(id);
		}
	}
	return sorted;
}

Arm::Type
Device::getArmTypeFromId(Arm::IdType id) {
	std::map<Arm::IdType,Arm::Type>::const_iterator itr = ARM_TYPES.find(id);
	if (itr == ARM_TYPES.end()) {
		std::stringstream ss;
		ss << "Arm type for id " << id << " not found!";
		throw std::runtime_error(ss.str());
	}
	return itr->second;
}

ArmList
Device::arms() {
	return arms_;
}

ConstArmList
Device::arms() const {
	return constList(arms_);
}

ArmPtr
Device::arm(size_t i) {
	return arms_[i];
}

ArmConstPtr
Device::arm(size_t i) const {
	return ArmConstPtr(arms_[i]);
}

ArmPtr
Device::getArmById(Arm::IdType id) {
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

ArmConstPtr
Device::getArmById(Arm::IdType id) const {
	return ArmConstPtr(const_cast<Device*>(this)->getArmById(id));
}

ArmList
Device::getArmsById(const Arm::IdList& ids,bool includeDisabled) {
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

ConstArmList
Device::getArmsById(const Arm::IdList& ids,bool includeDisabled) const {
	return constList(const_cast<Device*>(this)->getArmsById(ids,includeDisabled));
}

ArmPtr
Device::getArmByName(const std::string& name) {
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

ArmConstPtr
Device::getArmByName(const std::string& name) const {
	return ArmConstPtr(const_cast<Device*>(this)->getArmByName(name));
}

Arm::IdType
Device::getArmIdFromName(const std::string& name) {
	std::map<Arm::IdType,std::string>::const_iterator itr;
	for (itr=ARM_NAMES.begin();itr!=ARM_NAMES.end();itr++) {
		if (itr->second == name) {
			return itr->first;
		}
	}
	std::stringstream ss;
	ss << "Arm id for name " << name << " not found!";
	throw std::runtime_error(ss.str());
}

std::string
Device::getArmNameFromId(Arm::IdType id) {
	std::map<Arm::IdType,std::string>::const_iterator itr = ARM_NAMES.find(id);
	if (itr == ARM_NAMES.end()) {
		std::stringstream ss;
		ss << "Arm name for id " << id << " not found!";
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
Device::disabledArms() {
	return disabledArms_;
}

ConstArmList
Device::disabledArms() const {
	return constList(const_cast<Device*>(this)->disabledArms_);
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
Device::allArms() {
	ArmList arms = arms_;
	arms.insert(arms.end(),disabledArms_.begin(),disabledArms_.end());
	return arms;
}

ConstArmList
Device::allArms() const {
	return constList(const_cast<Device*>(this)->allArms());
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
