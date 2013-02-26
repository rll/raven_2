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


#endif /* DEVICE_H_ */
