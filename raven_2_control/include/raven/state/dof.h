/*
 * dof.h
 *
 *  Created on: Oct 1, 2012
 *      Author: benk
 */

#ifndef DOF_H_
#define DOF_H_

#include <ros/ros.h>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>

#include <raven/util/enum.h>
#include <raven/util/pointers.h>

#include "updateable.h"

POINTER_TYPES(Joint)
//typedef std::vector<JointPtr> JointList;
//typedef std::vector<JointConstPtr> ConstJointList;

BOOST_ENUM(JointIdType, (SHOULDER_)(ELBOW_)(INSERTION_)(ROTATION_)(WRIST_)(FINGER1_)(FINGER2_)(YAW_)(GRASP_ ));
BOOST_ENUM(JointType, (ROTATIONAL)(PRISMATIC))
BOOST_ENUM(JointState, (NOT_READY)(POS_UNKNOWN)(HOMING1)(HOMING2)(READY)(WAIT)(HARD_STOP));

BOOST_ENUM(MotorIdType, (SHOULDER_)(ELBOW_)(INSERTION_)(TOOL1_)(TOOL2_)(TOOL3_)(TOOL4_));
BOOST_ENUM(MotorType, (LARGE)(SMALL));
BOOST_ENUM(MotorTransmissionType, (TA)(TB));

BOOST_ENUM(CableType, (LARGE)(SMALL) );

class Joint : public Updateable {
	friend class DeviceInitializer;
	friend class CableCoupler;
public:
	typedef JointIdType IdType;
	typedef JointType Type;

	typedef JointState State;
private:
	IdType id_;
	bool toolJoint_;
	Type type_;

	bool hasMainMotor_;
	MotorIdType mainMotor_;

	State state_;

	float position_;
	float velocity_;

	// joint limits in radians
	float minPosition_;
	float maxPosition_;

	// starting position in radians
	float homePosition_;

	float speedLimit_; // radian / sec

	Joint(IdType id,Type type=Type::ROTATIONAL);
public:
	JointPtr clone() const;
	void cloneInto(JointPtr& joint) const;
	virtual ~Joint();

	IdType id() const;
	std::string idString() const;
	std::string idStringUpper() const;
	std::string idStringLower() const;

	bool isToolJoint() const;

	Type type() const;

	bool hasMainMotor() const;
	MotorIdType mainMotor() const;

	State state() const;

	void setState(State state);

	float position() const;
	float velocity() const;

	float minPosition() const;
	float maxPosition() const;

	float homePosition() const;
	float speedLimit() const;

	std::string str() const;

	void setPosition(float pos);
	void setVelocity(float vel);

	static Eigen::VectorXf positionVector(const JointList& joints);
	static Eigen::VectorXf velocityVector(const JointList& joints);
protected:
	virtual bool processNotification(Updateable* sender) { return true; }
	//virtual UpdateablePtr internalClone() const;
};

class JointCoupler {
public:
	JointCoupler() { /*printf("+JC X %p\n",this);*/ }
	virtual ~JointCoupler() { /*printf("-JC X %p\n",this);*/ }

	virtual JointList getBaseJoints(const JointList& joints) const = 0;

	virtual JointList getDependentJoints(const JointList& joints) const = 0;

	JointList getJoints(const JointList& joints) const {
		JointList theJoints = getBaseJoints(joints);
		JointList depJoints = getDependentJoints(joints);
		theJoints.insert(theJoints.end(),depJoints.begin(),depJoints.end());
		return theJoints;
	}

	virtual void coupleForward(const JointList& baseJoints,const JointList& depJoints)=0;
	virtual void coupleBackward(const JointList& depJoints,const JointList& baseJoints)=0;
};
POINTER_TYPES(JointCoupler)

POINTER_TYPES(Motor)

typedef std::vector<MotorPtr> MotorList;
typedef std::vector<MotorConstPtr> ConstMotorList;

class Motor : public Updateable {
	friend class DeviceInitializer;
	friend class CableCoupler;
	friend class MotorFilter;
public:
	typedef MotorIdType IdType;
	typedef MotorType Type;
	typedef MotorTransmissionType TransmissionType;
private:
	IdType id_;
	std::string name_;

	Type type_;
	TransmissionType transmissionType_;
	CableType cableType_;

	bool hasMainJoint_;
	Joint::IdType mainJoint_;

	float position_;
	float velocity_;
	float torque_;
	float gravitationalTorqueEstimate_;
	short int dacCommand_;
	int encoderValue_;
	int encoderOffset_;

	// encoder counts per revolution
	int encoderCountsPerRev_;

	int dacMax_;

	//DOF current variables
	//float i_max;
	//float i_cont;

	float transmissionRatio_;

	float tauPerAmp_;

	float dacCountsPerAmp_;

	Motor(IdType id, Type type, TransmissionType transType, CableType cableType);
public:
	MotorPtr clone() const;
	void cloneInto(MotorPtr& motor) const;
	virtual ~Motor();

	IdType id() const;
	std::string name() const;

	Type type() const;
	TransmissionType transmissionType() const;
	CableType cableType() const;

	bool hasMainJoint() const;
	Joint::IdType mainJoint() const;

	float position() const;
	float velocity() const;
	float torque() const;
	float gravitationalTorqueEstimate() const;
	short int dacCommand() const;
	int encoderValue() const;
	int encoderOffset() const;

	int encoderCountsPerRev() const;
	int dacMax() const;
	float torqueMax() const;

	float transmissionRatio() const;
	float tauPerAmp() const;
	float dacCountsPerAmp() const;

	void setPosition(float pos);
	void setVelocity(float vel);
	void setTorque(float t);
	void setGravitationalTorqueEstimate(float gte);
	void setDacCommand(short int cmd);
	void setEncoderValue(int val,bool updatePosition); //updates position
	void setEncoderOffset(int offset); //updates position

	virtual std::string str() const;

	static Eigen::VectorXf positionVector(const MotorList& motors);
	static Eigen::VectorXf velocityVector(const MotorList& motors);
	static Eigen::VectorXf torqueVector(const MotorList& motors);
protected:
	virtual bool processNotification(Updateable* sender) { return true; }
	//virtual UpdateablePtr internalClone() const;
};

POINTER_TYPES(MotorFilter)

class MotorFilter {
protected:
	MotorList motors_;
	bool motorsForUpdateReady_;
	MotorList motorsForUpdate_;

	virtual void internalApplyUpdate() = 0;
	virtual void internalCloneInto(MotorFilterPtr& other, const MotorList& newMotors) const {}

public:
	MotorFilter(const MotorList& motors);
	MotorFilter(const MotorFilter& other);
	virtual ~MotorFilter() {}

	MotorList motors();
	ConstMotorList motors() const;

	MotorList getMotorsForUpdate();

	void getMotorsForUpdate(MotorList& list);

	void applyUpdate();

	virtual void reset() = 0;

	virtual std::string str() const = 0;

	virtual MotorFilterPtr clone(const MotorList& newMotors) const = 0;
	void cloneInto(MotorFilterPtr& other, const MotorList& newMotors) const;
};

class NullMotorFilter : public MotorFilter {
protected:
	virtual void internalApplyUpdate() {
		TRACER_ENTER_SCOPE("NullMotorFilter::internalApplyUpdate");
		UpdateablePtr parent;
		for (size_t i=0;i<motorsForUpdate_.size();i++) {
			parent = motors_[i]->parent();
			motorsForUpdate_[i]->cloneInto(motors_[i]);
			motors_[i]->setUpdateableParent(parent);
			//*(motors_[i]) = *(motorsForUpdate_[i]);
		}
	}
	virtual void internalCloneInto(MotorFilterPtr& other, const MotorList& newMotors) const {
		if (!boost::dynamic_pointer_cast<NullMotorFilter>(other)) {
			MotorFilterPtr newMotorFilter = clone(newMotors);
			other.swap(newMotorFilter);
			return;
		}
		*other = *this;
	}
public:
	NullMotorFilter(const MotorList& motors);
	virtual ~NullMotorFilter();

	virtual void reset() {}

	virtual std::string str() const { return "NullMotorFilter"; }

	virtual MotorFilterPtr clone(const MotorList& newMotors) const;
};

POINTER_TYPES(CableCoupler)

class CableCoupler {
protected:
	Eigen::MatrixXf forwardMatrix_;
	std::vector<bool> forwardMask_;
	Eigen::MatrixXf backwardMatrix_;
	std::vector<bool> backwardMask_;
public:
	CableCoupler(const Eigen::MatrixXf& forwardMatrix,const Eigen::MatrixXf& backwardMatrix);

	virtual void setForwardMask(const std::vector<bool> mask) { forwardMask_ = mask; }
	virtual void setBackwardMask(const std::vector<bool> mask) { backwardMask_ = mask; }

	virtual void coupleForward(const MotorList& motors,const JointList& joints);
	virtual void coupleBackward(const JointList& joints,const MotorList& motors);

	virtual ~CableCoupler();
	virtual CableCouplerPtr clone() const;
	virtual void cloneInto(CableCouplerPtr& other) const;
};

#endif /* DOF_H_ */
