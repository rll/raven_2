/*
 * initializer.h
 *
 *  Created on: Oct 1, 2012
 *      Author: benk
 */

#ifndef INITIALIZER_H_
#define INITIALIZER_H_

#include <ros/ros.h>
#include <vector>

#include "device.h"

struct ArmData {
	int id;
	Arm::Type type;
	std::string name;
	Arm::ToolType toolType;
	ArmData(int _id, Arm::Type _type, Arm::ToolType _toolType) : id(_id), type(_type), name(_type.str()), toolType(_toolType) {}
	ArmData(int _id, Arm::Type _type, const std::string& _name, Arm::ToolType _toolType) : id(_id), type(_type), name(_name), toolType(_toolType) {}
};

class DeviceInitializer {
private:
	static std::vector<ArmData> ARM_DATA;

public:
	virtual ~DeviceInitializer() {}

	static std::vector<ArmData> getArms();
	static void addArm(int id, Arm::Type type, Arm::ToolType toolType);
	static void addArm(int id, Arm::Type type, const std::string& name, Arm::ToolType toolType);

	virtual void initializeDevice(DevicePtr device);
	virtual void initializeDeviceInstance();
};

#define GREEN_ARM_BASE_POSE btTransform(btQuaternion(0,0,M_PI_2),btVector3(-0.14858,0.002,0))

class YawGraspCoupler : public JointCoupler {
private:
	Arm::Type armType_;
public:
	YawGraspCoupler(Arm::Type armType) : armType_(armType) {}

	virtual std::vector<JointPtr> getBaseJoints(const std::vector<JointPtr>& joints) const;

	virtual std::vector<JointPtr> getDependentJoints(const std::vector<JointPtr>& joints) const;

	virtual void coupleForward(const std::vector<JointPtr>& baseJoints,const std::vector<JointPtr>& depJoints);
	virtual void coupleBackward(const std::vector<JointPtr>& depJoints,const std::vector<JointPtr>& baseJoints);
};

#endif /* INITIALIZER_H_ */
