/*
 * device.h
 *
 *  Created on: Oct 1, 2012
 *      Author: benk
 */

#ifndef DEVICE_H_
#define DEVICE_H_

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>

#include <vector>

#include <raven/state/updateable.h>
#include <raven/state/arm.h>

#include <raven/util/history.h>
#include <raven/util/pointers.h>

#define DEVICE_HISTORY_SIZE 10

#define USE_NEW_DEVICE

//class Device;
//typedef boost::shared_ptr<Device> DevicePtr;
//typedef boost::weak_ptr<Device> DeviceWeakPtr;
POINTER_TYPES(Device);

#ifndef FOREACH_ARM_IN_DEVICE
#include <algorithm>
#define FOREACH_ARM_IN_DEVICE(armVar,devicePtr) BOOST_FOREACH(ArmPtr armVar,devicePtr->arms())
#define FOREACH_ARM_IN_CONST_DEVICE(armVar,devicePtr) BOOST_FOREACH(ArmConstPtr armVar,devicePtr->arms())
#define FOREACH_ARM_IN_CURRENT_DEVICE(armVar,devicePtr) Device::current(devicePtr); FOREACH_ARM_IN_DEVICE(armVar,devicePtr)

#define FOREACH_ARM_IN_DEVICE_AND_ID_LIST(armVar,devicePtr,armIdList) FOREACH_ARM_IN_DEVICE(armVar,devicePtr) if (std::find(armIdList.begin(),armIdList.end(),armVar->id()) == armIdList.end()) continue; else
#define FOREACH_ARM_IN_CONST_DEVICE_AND_ID_LIST(armVar,devicePtr,armIdList) FOREACH_ARM_IN_CONST_DEVICE(armVar,devicePtr) if (std::find(armIdList.begin(),armIdList.end(),armVar->id()) == armIdList.end()) continue; else
#define FOREACH_ARM_ID_IN_LIST(armIdVar,armIdList) BOOST_FOREACH(Arm::IdType armIdVar,armIdList)
#define FOREACH_ARM_ID(armIdVar) FOREACH_ARM_ID_IN_LIST(armIdVar,Device::armIds())

#endif

class Device : public Updateable {
	friend class DeviceInitializer;
public:
	enum DeviceType { RAVEN_ROBOT };
private:
	static DevicePtr INSTANCE;
	static History<Device>::Type HISTORY;

	void addArm(ArmPtr arm);
public:
	static bool DEBUG_OUTPUT_TIMING;

	DeviceType type_;
	ros::Time timestamp_;

	ArmList arms_;
	ArmList disabledArms_;

	static Arm::IdList ARM_IDS;
	static Arm::IdList DISABLED_ARM_IDS;
	static std::map<Arm::IdType,std::string> ARM_NAMES;
	static std::map<Arm::IdType,Arm::Type> ARM_TYPES;

	static std::map<Arm::IdType,size_t> NUM_MOTORS;
	static size_t TOTAL_NUM_MOTORS;
	static std::map<Arm::IdType,size_t> NUM_JOINTS;
	static size_t TOTAL_NUM_JOINTS;

	Device(DeviceType type);

	static void init(DevicePtr dev);
	void internalFinishUpdate(bool updateTimestamp);
public:
	static DevicePtr current(); //locks, returns clone
	static void current(DevicePtr& device); //locks, returns clone
	static DeviceConstPtr currentNoClone(); //no lock, use carefully
	static DevicePtr currentNoCloneMutable(); //no lock, use carefully
	static ros::Time currentTimestamp();

	static DevicePtr beginCurrentUpdate(ros::Time updateTime);
	static void finishCurrentUpdate();

	static std::vector<DevicePtr> history(int numSteps=-1);

	DevicePtr clone() const;
	void cloneInto(DevicePtr& device) const;

	DeviceType type() const { return type_; }

	virtual ros::Time timestamp() const { return timestamp_; }

	static size_t numArms();
	static Arm::IdList armIds();
	static Arm::IdList& sortArmIds(Arm::IdList& armIds);
	static Arm::IdList sortArmIds(const Arm::IdSet& armIds);

	static Arm::Type getArmTypeFromId(Arm::IdType id);

	ArmList arms();
	ConstArmList arms() const;

	ArmPtr arm(size_t i);
	ArmConstPtr arm(size_t i) const;

	ArmPtr getArmById(Arm::IdType id);
	ArmConstPtr getArmById(Arm::IdType id) const;

	ArmList getArmsById(const Arm::IdList& ids,bool includeDisabled=false);
	ConstArmList getArmsById(const Arm::IdList& ids,bool includeDisabled=false) const;

	ArmPtr getArmByName(const std::string& name);
	ArmConstPtr getArmByName(const std::string& name) const;

	static Arm::IdType getArmIdFromName(const std::string& name);
	static std::string getArmNameFromId(Arm::IdType id);

	static size_t numDisabledArms();
	static Arm::IdList disabledArmIds();
	ArmList disabledArms();
	ConstArmList disabledArms() const;

	static size_t numAllArms();
	static Arm::IdList allArmIds();
	ArmList allArms();
	ConstArmList allArms() const;

	static size_t numJoints();
	static size_t numJointsOnArm(size_t i);
	static size_t numJointsOnArmById(Arm::IdType id);

	JointPtr getJointByOldType(int type);
	JointConstPtr getJointByOldType(int type) const;

	Eigen::VectorXf jointVector() const { return jointPositionVector(); }
	Eigen::VectorXf jointPositionVector() const;
	Eigen::VectorXf jointVelocityVector() const;

	static size_t numMotors();
	static size_t numMotorsOnArm(size_t i);
	static size_t numMotorsOnArmById(Arm::IdType id);

	Eigen::VectorXf motorPositionVector() const;
	Eigen::VectorXf motorVelocityVector() const;
	Eigen::VectorXf motorTorqueVector() const;

	void beginUpdate();
	void finishUpdate();

	virtual ~Device();

protected:
	virtual bool processNotification(Updateable* sender);
};


/*************************** INLINE METHODS **************************/

inline size_t
Device::numArms() {
	return ARM_IDS.size();
}

inline Arm::IdList
Device::armIds() {
	return ARM_IDS;
}

inline Arm::IdList&
Device::sortArmIds(Arm::IdList& armIds) {
	Arm::IdSet idSet = Arm::idSet(armIds);
	armIds = Device::sortArmIds(idSet);
	return armIds;
}

inline Arm::IdList
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

inline Arm::Type
Device::getArmTypeFromId(Arm::IdType id) {
	std::map<Arm::IdType,Arm::Type>::const_iterator itr = ARM_TYPES.find(id);
	if (itr == ARM_TYPES.end()) {
		std::stringstream ss;
		ss << "Arm type for id " << id << " not found!";
		throw std::runtime_error(ss.str());
	}
	return itr->second;
}

inline ArmList
Device::arms() {
	return arms_;
}

inline ConstArmList
Device::arms() const {
	return constList(arms_);
}

inline ArmPtr
Device::arm(size_t i) {
	return arms_[i];
}

inline ArmConstPtr
Device::arm(size_t i) const {
	return ArmConstPtr(arms_[i]);
}

inline ArmPtr
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

inline ArmConstPtr
Device::getArmById(Arm::IdType id) const {
	return ArmConstPtr(const_cast<Device*>(this)->getArmById(id));
}

inline ArmList
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

inline ConstArmList
Device::getArmsById(const Arm::IdList& ids,bool includeDisabled) const {
	return constList(const_cast<Device*>(this)->getArmsById(ids,includeDisabled));
}

inline ArmPtr
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

inline ArmConstPtr
Device::getArmByName(const std::string& name) const {
	return ArmConstPtr(const_cast<Device*>(this)->getArmByName(name));
}

inline Arm::IdType
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

inline std::string
Device::getArmNameFromId(Arm::IdType id) {
	std::map<Arm::IdType,std::string>::const_iterator itr = ARM_NAMES.find(id);
	if (itr == ARM_NAMES.end()) {
		std::stringstream ss;
		ss << "Arm name for id " << id << " not found!";
		throw std::runtime_error(ss.str());
	}
	return itr->second;
}

inline size_t
Device::numDisabledArms() {
	return DISABLED_ARM_IDS.size();
}

inline Arm::IdList
Device::disabledArmIds() {
	return DISABLED_ARM_IDS;
}

inline ArmList
Device::disabledArms() {
	return disabledArms_;
}

inline ConstArmList
Device::disabledArms() const {
	return constList(const_cast<Device*>(this)->disabledArms_);
}

inline size_t
Device::numAllArms() {
	return numArms() + numDisabledArms();
}

inline Arm::IdList
Device::allArmIds() {
	Arm::IdList armIds = ARM_IDS;
	armIds.insert(armIds.end(),DISABLED_ARM_IDS.begin(),DISABLED_ARM_IDS.end());
	return armIds;
}


inline ArmList
Device::allArms() {
	ArmList arms = arms_;
	arms.insert(arms.end(),disabledArms_.begin(),disabledArms_.end());
	return arms;
}

inline ConstArmList
Device::allArms() const {
	return constList(const_cast<Device*>(this)->allArms());
}

inline size_t
Device::numJoints() {
	return TOTAL_NUM_JOINTS;
}
inline size_t
Device::numJointsOnArm(size_t i) {
	return NUM_JOINTS[ARM_IDS[i]];
}
inline size_t
Device::numJointsOnArmById(Arm::IdType id) {
	return NUM_JOINTS[id];
}

inline Eigen::VectorXf
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

inline Eigen::VectorXf
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

inline size_t
Device::numMotors() {
	return TOTAL_NUM_MOTORS;
}
inline size_t
Device::numMotorsOnArm(size_t i) {
	return NUM_MOTORS[ARM_IDS[i]];
}
inline size_t Device::numMotorsOnArmById(Arm::IdType id) {
	return NUM_MOTORS[id];
}

inline Eigen::VectorXf
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

inline Eigen::VectorXf
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

inline Eigen::VectorXf
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

#endif /* DEVICE_H_ */
