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

//#define USE_NEW_DEVICE

//class Device;
//typedef boost::shared_ptr<Device> DevicePtr;
//typedef boost::weak_ptr<Device> DeviceWeakPtr;
POINTER_TYPES(Device);

#ifndef FOREACH_ARM_IN_DEVICE
#define FOREACH_ARM_IN_DEVICE(armVar,devicePtr) BOOST_FOREACH(ArmPtr armVar,devicePtr->arms())
#define FOREACH_ARM_IN_CURRENT_DEVICE(armVar,devicePtr) Device::current(devicePtr); FOREACH_ARM_IN_DEVICE(armVar,devicePtr)
//#define FOREACH_ARM(armVar) FOREACH_ARM_IN_DEVICE(armVar,Device::current())
#endif

class Device : public Updateable {
	friend class DeviceInitializer;
public:
	enum DeviceType { RAVEN_ROBOT };
private:
	static DevicePtr INSTANCE;
	//static boost::circular_buffer<DevicePtr> HISTORY;
public:
	static History<Device>::Type HISTORY;

	DeviceType type_;
	ros::Time timestamp_;

	//RunLevel runlevel_;
	bool surgeonMode_;
	ArmList arms_;

	Device(DeviceType type);

	static void init(DevicePtr dev);
	void internalFinishUpdate(bool updateTimestamp);
public:
	static DevicePtr current(); //locks, returns clone
	static void current(DevicePtr& device); //locks, returns clone
	static DevicePtr currentNoClone(); //no lock, use carefully
	static ros::Time currentTimestamp();

	static DevicePtr beginCurrentUpdate(ros::Time updateTime);
	static void finishCurrentUpdate();

	static std::vector<DevicePtr> history(int numSteps=-1);

	DevicePtr clone() const;
	void cloneInto(DevicePtr& device) const;

	DeviceType type() const { return type_; }

	virtual ros::Time timestamp() const { return timestamp_; }

	bool surgeonMode() const { return surgeonMode_; }

	ArmList arms() const { return arms_; }
	ArmPtr arm(size_t i) const { return arms_[i]; }
	ArmPtr getArmById(Arm::IdType id) const;
	ArmPtr getArmByName(const std::string& name) const;

	JointPtr getJointByOldType(int type) const;

	Eigen::VectorXf jointVector() const { return jointPositionVector(); }
	Eigen::VectorXf jointPositionVector() const;
	Eigen::VectorXf jointVelocityVector() const;

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
