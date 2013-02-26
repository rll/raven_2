/*
 * arm.h
 *
 *  Created on: Oct 1, 2012
 *      Author: benk
 */

#ifndef ARM_H_
#define ARM_H_

#include <ros/ros.h>
#include <LinearMath/btTransform.h>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

#include <string>
#include <vector>
#include <set>
#include <memory>

#include <raven/kinematics/kinematics.h>


#include <raven/state/updateable.h>
#include <raven/state/dof.h>

#include <raven/util/enum.h>


//typedef unsigned char pins_t;

POINTER_TYPES(Arm)
//typedef std::vector<ArmPtr> ArmList;
//typedef std::vector<ArmConstPtr> ConstArmList;

#ifndef FOREACH_JOINT_IN_ARM
#define FOREACH_JOINT_IN_ARM(jointVar,armPtr) BOOST_FOREACH(JointPtr jointVar,armPtr->joints())
#define FOREACH_MOTOR_IN_ARM(motorVar,armPtr) BOOST_FOREACH(MotorPtr motorVar,armPtr->motors())
#endif

typedef int ArmIdType;
BOOST_ENUM(ArmType,(GOLD)(GREEN))
BOOST_ENUM(ArmToolType, (NONE)(GRASPER_10MM)(GRASPER_8MM));

class Arm : public Updateable {
	friend class Device;
	friend class DeviceInitializer;
public:
	typedef ArmIdType IdType;
	static const IdType ALL_ARMS;

	typedef ArmType Type;

	typedef ArmToolType ToolType;

	typedef std::vector<IdType> IdList;
	typedef std::set<IdType> IdSet;
	inline static IdSet idSet(const IdList& ids) {
		return Arm::IdSet(ids.begin(),ids.end());
	}
	inline static IdSet& idSetAdd(IdSet& idSet, const IdList& ids) {
		idSet.insert(ids.begin(),ids.end());
		return idSet;
	}
	inline static IdSet& idSetAdd(IdSet& idSet, const IdSet& ids) {
		idSet.insert(ids.begin(),ids.end());
		return idSet;
	}

private:
	IdType id_;
	Type type_;
	std::string name_;

	bool enabled_;

	ToolType toolType_;

	btTransform basePose_;

	JointList joints_;
	MotorList motors_;
	MotorFilterPtr stateMotorFilter_;
	MotorFilterPtr controlMotorFilter_;

	CableCouplerPtr cableCoupler_;

	std::vector<JointCouplerPtr> jointCouplers_;

	boost::shared_ptr<KinematicSolver> kinematicSolver_;

	Arm(int id, Type type, const std::string& name, ToolType toolType);
	Arm(const Arm& other);

	static void init(ArmPtr arm);

	void updateJointsFromMotors();
	void updateMotorsFromJoints();


public:
	ArmPtr clone() const;
	void cloneInto(ArmPtr& other) const;

	IdType id() const;
	std::string idString() const;

	Type type() const;
	bool isGold() const;
	bool isGreen() const;
	std::string typeString() const;
	std::string typeStringUpper() const;
	std::string typeStringLower() const;

	std::string name() const;
	std::string nameUpper() const;
	std::string nameLower() const;


	bool enabled() const;

	ToolType toolType() const;

	btTransform basePose() const;

	JointList joints();
	ConstJointList joints() const;

	JointPtr joint(size_t i);
	JointConstPtr joint(size_t i) const;

	JointPtr getJointById(Joint::IdType id);
	JointConstPtr getJointById(Joint::IdType id) const;

	JointPtr getJointByOldType(int type);
	JointConstPtr getJointByOldType(int type) const;

	Eigen::VectorXf jointVector() const { return jointPositionVector(); }
	Eigen::VectorXf jointPositionVector() const;
	Eigen::VectorXf jointVelocityVector() const;

	void addJointCoupler(JointCouplerPtr coupler);

	MotorList motors();
	ConstMotorList motors() const;

	MotorPtr motor(size_t i);
	MotorConstPtr motor(size_t i) const;

	MotorPtr getMotorByOldType(int type);
	MotorConstPtr getMotorByOldType(int type) const;

	Eigen::VectorXf motorPositionVector() const;
	Eigen::VectorXf motorVelocityVector() const;
	Eigen::VectorXf motorTorqueVector() const;

	MotorFilterPtr stateMotorFilter();
	MotorFilterConstPtr stateMotorFilter() const;
	void setStateMotorFilter(MotorFilterPtr filter);

	MotorFilterPtr controlMotorFilter();
	MotorFilterConstPtr controlMotorFilter() const;
	void setcontrolMotorFilter(MotorFilterPtr filter);

	KinematicSolver& kinematics() { return *kinematicSolver_; }
	const KinematicSolver& kinematics() const { return *kinematicSolver_; }

	btTransform pose() const;

	virtual ~Arm();
protected:
	virtual bool internalUpdate();
	virtual bool processNotification(Updateable* sender);
};

/*************************** INLINE METHODS **************************/

#include <boost/algorithm/string.hpp>
#include <string>

inline Arm::IdType Arm::id() const { return id_; }
inline std::string Arm::idString() const {
	#define ARM_ID_STRING_CHAR_BUFFER_SIZE 25
	char buf[ARM_ID_STRING_CHAR_BUFFER_SIZE];
	snprintf(buf,ARM_ID_STRING_CHAR_BUFFER_SIZE,"%i",id_);
	return std::string(buf);
}

inline Arm::Type Arm::type() const { return type_; }
inline bool Arm::isGold() const { return type_ == Type::GOLD; }
inline bool Arm::isGreen() const { return type_ == Type::GREEN; }

inline std::string
Arm::typeString() const {
	return type_.str();
}
inline std::string
Arm::typeStringUpper() const {
	return boost::to_upper_copy(typeString());
}
inline std::string
Arm::typeStringLower() const {
	return boost::to_lower_copy(typeString());
}

inline std::string Arm::name() const { return name_; }
inline std::string Arm::nameUpper() const { return boost::to_upper_copy(name_); }
inline std::string Arm::nameLower() const { return boost::to_lower_copy(name_); }

inline bool Arm::enabled() const { return enabled_; }

inline Arm::ToolType Arm::toolType() const { return toolType_; }

inline btTransform Arm::basePose() const { return basePose_; }

inline JointList Arm::joints() { return joints_; }
inline ConstJointList Arm::joints() const { return constList(joints_); }

inline JointPtr Arm::joint(size_t i) { return i>joints_.size() ? JointPtr() : joints_.at(i); }
inline JointConstPtr Arm::joint(size_t i) const { return i>joints_.size() ? JointConstPtr() : JointConstPtr(joints_.at(i)); }

inline JointPtr
Arm::getJointById(Joint::IdType id) {
	size_t ind = id.value();
	if (ind < joints_.size() && joints_[ind]->id() == id) {
		return joints_[ind];
	}
	std::vector<JointPtr>::const_iterator itr;
	for (itr=joints_.begin();itr!=joints_.end();itr++) {
		if ((*itr)->id() == id) {
			return *itr;
		}
	}
	return JointPtr();
}

inline JointConstPtr
Arm::getJointById(Joint::IdType id) const {
	return JointConstPtr(const_cast<Arm*>(this)->getJointById(id));
}

inline Eigen::VectorXf
Arm::jointPositionVector() const {
	return Joint::positionVector(joints_);
}
inline Eigen::VectorXf
Arm::jointVelocityVector() const {
	return Joint::velocityVector(joints_);
}

inline void Arm::addJointCoupler(JointCouplerPtr coupler) { jointCouplers_.push_back(coupler); }

inline MotorList Arm::motors() { return motors_; }
inline ConstMotorList Arm::motors() const { return constList(motors_); }

inline MotorPtr Arm::motor(size_t i) {
	return i>motors_.size() ? MotorPtr() : motors_.at(i);
}
inline MotorConstPtr Arm::motor(size_t i) const {
	return MotorConstPtr(const_cast<Arm*>(this)->motor(i));
}

inline Eigen::VectorXf
Arm::motorPositionVector() const {
	return Motor::positionVector(motors_);
}
inline Eigen::VectorXf
Arm::motorVelocityVector() const {
	return Motor::velocityVector(motors_);
}
inline Eigen::VectorXf
Arm::motorTorqueVector() const {
	return Motor::torqueVector(motors_);
}

inline btTransform
Arm::pose() const {
	TRACER_ENTER_SCOPE();
	return kinematicSolver_->forwardPose();
}

#endif /* ARM_H_ */
