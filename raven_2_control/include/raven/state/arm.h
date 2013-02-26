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

	virtual bool update();

	virtual ~Arm();
protected:
	virtual bool processNotification(Updateable* sender);
};


#endif /* ARM_H_ */
