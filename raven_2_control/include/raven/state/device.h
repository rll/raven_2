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

typedef unsigned char runlevel_t;

class RunLevel {
	friend class Device;
private:
	runlevel_t value_;
	runlevel_t sublevel_;

public:

	bool isEstop() const;
	bool isHardwareEstop() const;
	bool isSoftwareEstop() const;
	bool isInit(runlevel_t sublevel = -1) const;
	bool isPedalUp() const;
	bool isPedalDown() const;
	bool isPedalUpOrDown() const { return isPedalDown() || isPedalUp(); }

	bool isActive() const;

	std::string str() const;

	static RunLevel _E_STOP_();
	static RunLevel _INIT_(runlevel_t sublevel = 0);
	static RunLevel _PEDAL_UP_();
	static RunLevel _PEDAL_DOWN_();

	static RunLevel fromNumber(runlevel_t level);
private:
	RunLevel(runlevel_t level, runlevel_t sub = 0) : value_(level), sublevel_(sub) {}
	static RunLevel _E_STOP_HARDWARE_();
	static RunLevel _E_STOP_SOFTWARE_();
};

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
	static History<Device>::Type HISTORY;

	static bool RUNLEVEL_IS_INITED;
	static RunLevel* RUNLEVEL;

	DeviceType type_;
	ros::Time timestamp_;

	//RunLevel runlevel_;
	bool surgeonMode_;
	ArmList arms_;

	Device(DeviceType type);

	static void init(DevicePtr dev);
	void internalFinishUpdate(bool updateTimestamp);
public:
	static RunLevel runlevel();
	static void setRunlevel(RunLevel level);
	static void setSublevel(runlevel_t sublevel);

	static void eStop();
	static bool getPedal();
	static void setPedal(bool down);

	static bool isInitialized();

	static DevicePtr current(); //locks, returns clone
	static void current(DevicePtr& device); //locks, returns clone
	static DevicePtr currentNoClone(); //no lock, use carefully

	static void beginCurrentUpdate(ros::Time updateTime);
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
