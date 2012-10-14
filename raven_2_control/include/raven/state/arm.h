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
#include <memory>

#include <raven/kinematics/kinematics.h>


#include <raven/state/updateable.h>
#include <raven/state/dof.h>

#include <raven/util/enum.h>


//typedef unsigned char pins_t;

POINTER_TYPES(Arm)
typedef std::vector<ArmPtr> ArmList;

#ifndef FOREACH_JOINT_IN_ARM
#define FOREACH_JOINT_IN_ARM(jointVar,armPtr) BOOST_FOREACH(JointPtr jointVar,armPtr->joints())
#define FOREACH_MOTOR_IN_ARM(motorVar,armPtr) BOOST_FOREACH(MotorPtr motorVar,armPtr->motors())
#endif

class Arm : public Updateable {
	friend class Device;
	friend class DeviceInitializer;
public:
	BOOST_ENUM(Type,(GOLD)(GREEN))
	//BOOST_ENUM_STRINGS(Type,(GOLD)("Gold")(GREEN)("Green"))

	BOOST_ENUM(ToolType, (NONE)(GRASPER_10MM)(GRASPER_8MM));

	typedef int IdType;
private:
	IdType id_;
	Type type_;
	std::string name_;

	ToolType toolType_;

	btTransform basePose_;

	JointList joints_;
	MotorList motors_;
	MotorFilterPtr motorFilter_;

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

	IdType id() const { return id_; }

	Type type() const { return type_; }
	bool isGold() const { return type_ == Type::GOLD; }
	bool isGreen() const { return type_ == Type::GREEN; }

	std::string name() const { return name_; }

	ToolType toolType() const { return toolType_; }

	btTransform basePose() const { return basePose_; }

	JointList joints() const { return joints_; }
	JointPtr joint(size_t i) const { return i>joints_.size() ? JointPtr() : joints_.at(i); }
	JointPtr getJointByType(Joint::Type type) const;
	JointPtr getJointByOldType(int type) const;
	Eigen::VectorXf jointVector() const { return jointPositionVector(); }
	Eigen::VectorXf jointPositionVector() const;
	Eigen::VectorXf jointVelocityVector() const;

	void addJointCoupler(JointCouplerPtr coupler) { jointCouplers_.push_back(coupler); }

	MotorList motors() const { return motors_; }
	MotorPtr motor(size_t i) const { return i>motors_.size() ? MotorPtr() : motors_.at(i); }
	MotorPtr getMotorByOldType(int type) const;
	Eigen::VectorXf motorPositionVector() const;
	Eigen::VectorXf motorVelocityVector() const;
	Eigen::VectorXf motorTorqueVector() const;

	MotorFilterPtr motorFilter() const { return motorFilter_; }
	void setMotorFilter(MotorFilterPtr filter);

	KinematicSolver& kinematics() { return *kinematicSolver_; }
	const KinematicSolver& kinematics() const { return *kinematicSolver_; }

	btTransform pose() const;

	virtual bool update();

	virtual ~Arm();
protected:
	virtual bool processNotification(Updateable* sender);
};


#endif /* ARM_H_ */
