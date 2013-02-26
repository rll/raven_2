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

	inline JointList getJoints(const JointList& joints) const {
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
	virtual void internalApplyUpdate();
	virtual void internalCloneInto(MotorFilterPtr& other, const MotorList& newMotors) const;
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

/*************************** INLINE METHODS **************************/

#include <boost/algorithm/string.hpp>
#include <string>

inline Joint::IdType Joint::id() const { return id_; }
inline bool Joint::isToolJoint() const { return toolJoint_; }
inline std::string Joint::idString() const { return id_.str(); }
inline std::string Joint::idStringUpper() const { return boost::to_upper_copy(idString()); }
inline std::string Joint::idStringLower() const { return boost::to_lower_copy(idString()); }

inline Joint::Type Joint::type() const { return type_; }

inline bool Joint::hasMainMotor() const { return hasMainMotor_; }
inline Motor::IdType Joint::mainMotor() const { if (!hasMainMotor_) { throw std::runtime_error("No main motor!"); } else { return mainMotor_; } }

inline Joint::State Joint::state() const { return state_; }

inline void Joint::setState(State state ) { state_ = state; updateTimestamp(); }

inline float Joint::position() const { return position_; }
inline float Joint::velocity() const { return velocity_; }

inline float Joint::minPosition() const { return minPosition_; }
inline float Joint::maxPosition() const { return maxPosition_; }

inline float Joint::homePosition() const { return homePosition_; }
inline float Joint::speedLimit() const { return speedLimit_; }

inline Motor::IdType Motor::id() const { return id_; }
inline std::string Motor::name() const { return name_; }

inline Motor::Type Motor::type() const { return type_; }
inline Motor::TransmissionType Motor::transmissionType() const { return transmissionType_; }
inline CableType Motor::cableType() const { return cableType_; }

inline bool Motor::hasMainJoint() const { return hasMainJoint_; }
inline Joint::IdType Motor::mainJoint() const { if (!hasMainJoint_) { throw std::runtime_error("No main joint!"); } else { return mainJoint_; } }

inline float Motor::position() const { return position_; }
inline float Motor::velocity() const { return velocity_; }
inline float Motor::torque() const { return torque_; }
inline float Motor::gravitationalTorqueEstimate() const { return gravitationalTorqueEstimate_; }
inline short int Motor::dacCommand() const { return dacCommand_; }
inline int Motor::encoderValue() const { return encoderValue_; }
inline int Motor::encoderOffset() const { return encoderOffset_; }

inline int Motor::encoderCountsPerRev() const { return encoderCountsPerRev_; }
inline int Motor::dacMax() const { return dacMax_; }

inline float Motor::transmissionRatio() const { return transmissionRatio_; }
inline float Motor::tauPerAmp() const { return tauPerAmp_; }
inline float Motor::dacCountsPerAmp() const { return dacCountsPerAmp_; }

inline float
Motor::torqueMax() const {
	const float TFmotor     = 1 / tauPerAmp_;    // Determine the motor TF  = 1/(tau per amp)
	const float TFamplifier =     dacCountsPerAmp_;    // Determine the amplifier TF = (DAC_per_amp)

	return ((float)dacMax_) / (TFmotor * TFamplifier);
}

inline Eigen::VectorXf
Joint::positionVector(const JointList& joints) {
	Eigen::VectorXf v;
	v.resize(joints.size());
	for (size_t i=0;i<joints.size();i++) {
		v[i] = joints.at(i)->position();
	}
	return v;
}
inline Eigen::VectorXf
Joint::velocityVector(const JointList& joints) {
	Eigen::VectorXf v;
	v.resize(joints.size());
	for (size_t i=0;i<joints.size();i++) {
		v[i] = joints.at(i)->velocity();
	}
	return v;
}

inline Eigen::VectorXf
Motor::positionVector(const MotorList& motors) {
	Eigen::VectorXf v;
	v.resize(motors.size());
	for (size_t i=0;i<motors.size();i++) {
		v[i] = motors.at(i)->position();
	}
	return v;
}
inline Eigen::VectorXf
Motor::velocityVector(const MotorList& motors) {
	Eigen::VectorXf v;
	v.resize(motors.size());
	for (size_t i=0;i<motors.size();i++) {
		v[i] = motors.at(i)->velocity();
	}
	return v;
}
inline Eigen::VectorXf
Motor::torqueVector(const MotorList& motors) {
	Eigen::VectorXf v;
	v.resize(motors.size());
	for (size_t i=0;i<motors.size();i++) {
		v[i] = motors.at(i)->torque();
	}
	return v;
}



#endif /* DOF_H_ */
