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
typedef std::vector<JointPtr> JointList;

class Joint : public Updateable {
	friend class DeviceInitializer;
	friend class CableCoupler;
public:
	BOOST_ENUM(Type, (SHOULDER_)(ELBOW_)(INSERTION_)(TOOL_ROT_)(WRIST_)(GRIPPER1_)(GRIPPER2_)(YAW_)(GRASP_ ));

	BOOST_ENUM(State, (NOT_READY)(POS_UNKNOWN)(HOMING1)(HOMING2)(READY)(WAIT)(HARD_STOP));

	/*
	enum JointState{
	    jstate_not_ready   = 0,
	    jstate_pos_unknown = 1,
	    jstate_homing1     = 2,
	    jstate_homing2     = 3,
	    jstate_ready       = 4,
	    jstate_wait        = 5,
	    jstate_hard_stop   = 6,
	    jstate_last_type
	};
	*/
private:
	Type type_;
	bool toolJoint_;
	State state_;

	float position_;
	float velocity_;

	// joint limits in radians
	float minPosition_;
	float maxPosition_;

	// starting position in radians
	float homePosition_;

	float speedLimit_; // radian / sec

	Joint(Type type);
public:
	JointPtr clone() const;
	void cloneInto(JointPtr& joint) const;
	virtual ~Joint();

	Type type() const { return type_; }
	bool isToolJoint() const { return toolJoint_; }
	State state() const { return state_; }

	void setState(State state ) { state_ = state; updateTimestamp(); }

	float position() const { return position_; }
	float velocity() const { return velocity_; }

	float minPosition() const { return minPosition_; }
	float maxPosition() const { return maxPosition_; }

	float homePosition() const { return homePosition_; }
	float speedLimit() const { return speedLimit_; }

	std::string str() const;

	void setPosition(float pos) { position_ = pos; updateTimestamp(); }
	void setVelocity(float vel) { velocity_ = vel; updateTimestamp(); }

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

typedef std::vector<MotorPtr > MotorList;

class Motor : public Updateable {
	friend class DeviceInitializer;
	friend class CableCoupler;
	friend class MotorFilter;
public:
	BOOST_ENUM(Type, (LARGE)(SMALL));
	BOOST_ENUM(TransmissionType, (TA)(TB));
	BOOST_ENUM(CableType, (LARGE)(SMALL) );
private:
	Type type_;
	TransmissionType transmissionType_;
	CableType cableType_;

	bool hasMainJoint_;
	Joint::Type mainJoint_;

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

	Motor(Type type, TransmissionType transType, CableType cableType);
public:
	MotorPtr clone() const;
	void cloneInto(MotorPtr& motor) const;
	virtual ~Motor();

	Type type() const { return type_; }
	TransmissionType transmissionType() const { return transmissionType_; }
	CableType cableType() const { return cableType_; }

	bool hasMainJoint() const { return hasMainJoint_; }
	Joint::Type mainJoint() const { if (!hasMainJoint_) { throw std::runtime_error("No main joint!"); } else { return mainJoint_; } }

	float position() const { return position_; }
	float velocity() const { return velocity_; }
	float torque() const { return torque_; }
	float gravitationalTorqueEstimate() const { return gravitationalTorqueEstimate_; }
	short int dacCommand() const { return dacCommand_; }
	int encoderValue() const { return encoderValue_; }
	int encoderOffset() const { return encoderOffset_; }

	int encoderCountsPerRev() const { return encoderCountsPerRev_; }
	int dacMax() const { return dacMax_; }
	float torqueMax() const;

	float transmissionRatio() const { return transmissionRatio_; }
	float tauPerAmp() const { return tauPerAmp_; }
	float dacCountsPerAmp() const { return dacCountsPerAmp_; }

	void setPosition(float pos);
	void setVelocity(float vel) { velocity_ = vel; updateTimestamp(); }
	void setTorque(float t);
	void setGravitationalTorqueEstimate(float gte) { gravitationalTorqueEstimate_ = gte; updateTimestamp(); }
	void setDacCommand(short int cmd) { dacCommand_ = cmd; updateTimestamp(); }
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
	MotorFilter(const MotorList& motors) : motors_(motors), motorsForUpdateReady_(false), motorsForUpdate_(motors.size()) {}
	MotorFilter(const MotorFilter& other) : motors_(other.motors_), motorsForUpdateReady_(false), motorsForUpdate_(other.motors_.size()) {}
	virtual ~MotorFilter() {}

	MotorList motors() const {
		return motors_;
	}

	MotorList getMotorsForUpdate() {
		MotorList list;
		getMotorsForUpdate(list);
		return list;
	}

	void getMotorsForUpdate(MotorList& list) {
		static UpdateablePtr NULL_UPDATEABLE_PTR;
		if (!motorsForUpdateReady_) {
			for (size_t i=0;i<motorsForUpdate_.size() && i<motors_.size();i++) {
				motors_[i]->cloneInto(motorsForUpdate_[i]);
				motorsForUpdate_[i]->setUpdateableParent(NULL_UPDATEABLE_PTR);
			}
			for (size_t i=motorsForUpdate_.size();i<motors_.size();i++) {
				MotorPtr newMotor = motors_[i]->clone();
				motorsForUpdate_.push_back(newMotor);
			}
			motorsForUpdate_.resize(motors_.size());
		}
		motorsForUpdateReady_ = true;
		list = motorsForUpdate_;
	}

	void applyUpdate() {
		if (!motorsForUpdateReady_) {
			return;
		}
		internalApplyUpdate();
		for (size_t i=0;i<motors_.size();i++) {
			motors_[i]->update();
		}
		motorsForUpdateReady_ = false;
	}

	virtual void reset() = 0;

	virtual std::string str() const = 0;

	virtual MotorFilterPtr clone(const MotorList& newMotors) const = 0;
	void cloneInto(MotorFilterPtr& other, const MotorList& newMotors) const {
		internalCloneInto(other,newMotors);
		other->motorsForUpdateReady_ = false;
	}
};

class NullMotorFilter : public MotorFilter {
protected:
	virtual void internalApplyUpdate() {
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
