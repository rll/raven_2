/*
 * arm.cpp
 *
 *  Created on: Oct 1, 2012
 *      Author: benk
 */

#include <raven/state/arm.h>
#include <boost/foreach.hpp>

#include "defines.h"

#include <algorithm>

static int numA = 0;
Arm::Arm(int id, Type type, const std::string& name, ToolType toolType) : Updateable(), id_(id), type_(type), name_(name), toolType_(toolType),
	basePose_(btTransform::getIdentity()), cableCoupler_(), kinematicSolver_(new KinematicSolver(this)) {
	//printf("+A  %i %p\n",++numA,this);
}
Arm::Arm(const Arm& other) : Updateable(other), id_(other.id_), type_(other.type_), name_(other.name_), toolType_(other.toolType_),
		basePose_(other.basePose_) {
	for (MotorList::const_iterator itr=other.motors_.begin();itr!=other.motors_.end();itr++) {
		motors_.push_back((*itr)->clone());
	}
	MotorFilterPtr newMotorFilter = other.motorFilter_->clone(motors_);
	motorFilter_.swap(newMotorFilter);

	for (std::vector<JointPtr>::const_iterator itr=other.joints_.begin();itr!=other.joints_.end();itr++) {
		joints_.push_back((*itr)->clone());
	}

	//cableCoupler_.reset(new CableCoupler(*(other.cableCoupler_)));

	kinematicSolver_.reset(new KinematicSolver(*(other.kinematicSolver_)));
	kinematicSolver_->arm_ = this;
}

Arm::~Arm() {
	//printf("-A  %i %p\n",--numA,this);
}

ArmPtr
Arm::clone() const {
	ArmPtr newArm(new Arm(*this));
	//printf("+A  %i %p CLONING\n",++numA,this);

	init(newArm);

	return newArm;
}

void
Arm::cloneInto(ArmPtr& other) const {
	if (!other) {
		ArmPtr newArm = clone();
		other.swap(newArm);
		return;
	}
	other->id_ = id_;
	other->type_ = type_;
	other->name_ = name_;

	other->toolType_ = toolType_;

	other->basePose_ = basePose_;

	for (size_t i=0;i<other->joints_.size() && i<joints_.size();i++) {
		joints_[i]->cloneInto(other->joints_[i]);
	}
	for (size_t i=other->joints_.size();i<joints_.size();i++) {
		JointPtr newJoint = joints_[i]->clone();
		other->joints_.push_back(newJoint);
	}
	other->joints_.resize(joints_.size());

	for (size_t i=0;i<other->motors_.size() && i<motors_.size();i++) {
		motors_[i]->cloneInto(other->motors_[i]);
	}
	for (size_t i=other->motors_.size();i<motors_.size();i++) {
		MotorPtr newMotor = motors_[i]->clone();
		other->motors_.push_back(newMotor);
	}
	other->motors_.resize(motors_.size());

	//other->motorFilter_ = motorFilter_->clone(other->motors_);
	motorFilter_->cloneInto(other->motorFilter_,other->motors_);

	other->cableCoupler_ = cableCoupler_;

	other->jointCouplers_ = jointCouplers_;

	kinematicSolver_->cloneInto(other->kinematicSolver_,other.get());

	init(other);
}

void
Arm::init(ArmPtr arm) {
	arm->updateJointsFromMotors();

	BOOST_FOREACH(MotorPtr m,arm->motors_) {
		m->setUpdateableParent(arm);
	}
	if (!arm->motorFilter_) {
		arm->motorFilter_.reset(new NullMotorFilter(arm->motors_));
	}

	BOOST_FOREACH(JointPtr j,arm->joints_) {
		j->setUpdateableParent(arm);
	}

}


bool
Arm::processNotification(Updateable* sender) {
	MotorList::iterator motorItr;
	for (motorItr=motors_.begin();motorItr!=motors_.end();motorItr++) {
		if (motorItr->get() == sender) {
			break;
		}
	}
	//MotorList::iterator motorItr = find(motors_.begin(),motors_.end(),sender);
	if (motorItr != motors_.end()) {
		updateJointsFromMotors();
	}
	return true;
}

/*
void
Arm::holdUpdateBegin() {
	holdUpdates_ = true;
	heldUpdateTimestamp_ = ros::Time(0);
}

bool
Arm::holdUpdateEnd() {
	bool changed = heldUpdateTimestamp_ != ros::Time(0);
	if (changed) {
		//setUpdateableTimestamp(heldUpdateTimestamp_);
		update();
	}
	holdUpdates_ = false;
	heldUpdateTimestamp_ = ros::Time(0);
	return changed;
}
*/

void
Arm::updateJointsFromMotors() {
	if (!cableCoupler_) { return; }
	cableCoupler_->coupleForward(motors_,joints_);
	for (size_t i=0;i<jointCouplers_.size();i++) {
		std::vector<JointPtr> baseJoints = jointCouplers_[i]->getBaseJoints(joints_);
		std::vector<JointPtr> depJoints = jointCouplers_[i]->getDependentJoints(joints_);
		jointCouplers_[i]->coupleForward(baseJoints,depJoints);
	}
}

void
Arm::updateMotorsFromJoints() {
	for (size_t i=0;i<jointCouplers_.size();i++) {
		std::vector<JointPtr> baseJoints = jointCouplers_[i]->getBaseJoints(joints_);
		ros::Time baseJointTimestamp(0);
		for (size_t j=0;j<baseJoints.size();j++) {
			if (baseJoints[j]->timestamp() > baseJointTimestamp) {
				baseJointTimestamp = baseJoints[j]->timestamp();
			}
		}

		std::vector<JointPtr> depJoints = jointCouplers_[i]->getDependentJoints(joints_);
		ros::Time depJointTimestamp(0);
		for (size_t j=0;j<depJoints.size();j++) {
			if (depJoints[j]->timestamp() > depJointTimestamp) {
				depJointTimestamp = depJoints[j]->timestamp();
			}
		}

		if (baseJointTimestamp < depJointTimestamp) {
			jointCouplers_[i]->coupleBackward(depJoints,baseJoints);
		} else {
			jointCouplers_[i]->coupleForward(baseJoints,depJoints);
		}
	}
	if (!cableCoupler_) { return; }
	cableCoupler_->coupleBackward(joints_,motors_);
}

bool
Arm::update() {
	ros::Time latestMotorUpdate(0);
	for (MotorList::iterator itr=motors_.begin();itr!=motors_.end();itr++) {
		if ((*itr)->getUpdateableTimestamp() > latestMotorUpdate) {
			latestMotorUpdate = (*itr)->getUpdateableTimestamp();
		}
	}

	ros::Time latestJointUpdate(0);
	for (std::vector<JointPtr>::iterator itr=joints_.begin();itr!=joints_.end();itr++) {
		if ((*itr)->getUpdateableTimestamp() > latestJointUpdate) {
			latestJointUpdate = (*itr)->getUpdateableTimestamp();
		}
	}

	ros::Time compareTime;
	if (heldUpdateTimestamp_ != ros::Time(0)) {
		compareTime = heldUpdateTimestamp_;
	} else {
		compareTime = getUpdateableTimestamp();
	}

	if (latestMotorUpdate >= compareTime || latestJointUpdate >= compareTime) {
		if (latestMotorUpdate > latestJointUpdate) {
			updateJointsFromMotors();
		} else if (latestJointUpdate > latestMotorUpdate){
			updateMotorsFromJoints();
		}
	}

	return Updateable::update();
}

JointPtr
Arm::getJointByType(Joint::Type type) const {
	size_t ind = type.value();
	if (ind < joints_.size() && joints_[ind]->type() == type) {
		return joints_[ind];
	}
	std::vector<JointPtr>::const_iterator itr;
	for (itr=joints_.begin();itr!=joints_.end();itr++) {
		if ((*itr)->type() == type) {
			return *itr;
		}
	}
	return JointPtr();
}

JointPtr
Arm::getJointByOldType(int type) const {
	JointPtr joint;
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);
	if ((isGold() && arm_id != GOLD_ARM_ID) ||
			(isGreen() && arm_id != GREEN_ARM_ID)) {
		return joint;
	}
	if (joint_ind == 3) {
		return joint;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return joints_.at(joint_ind);
}

MotorPtr
Arm::getMotorByOldType(int type) const {
	MotorPtr motor;
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);
	if ((isGold() && arm_id != GOLD_ARM_ID) ||
			(isGreen() && arm_id != GREEN_ARM_ID)) {
		return motor;
	}
	if (joint_ind == 3) {
		return motor;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return motors_.at(joint_ind);
}


Eigen::VectorXf
Arm::jointPositionVector() const {
	return Joint::positionVector(joints_);
}
Eigen::VectorXf
Arm::jointVelocityVector() const {
	return Joint::velocityVector(joints_);
}

Eigen::VectorXf
Arm::motorPositionVector() const {
	return Motor::positionVector(motors_);
}
Eigen::VectorXf
Arm::motorVelocityVector() const {
	return Motor::velocityVector(motors_);
}
Eigen::VectorXf
Arm::motorTorqueVector() const {
	return Motor::torqueVector(motors_);
}

void
Arm::setMotorFilter(MotorFilterPtr filter) {
	if (!filter) {
		motorFilter_.reset(new NullMotorFilter(motors_));
	} else {
		if (filter->motors() == motors_) {
			motorFilter_ = filter;
		} else {
			motorFilter_ = filter->clone(motors_);
		}
	}
}

btTransform
Arm::pose() const {
	return kinematicSolver_->forwardPose();
}
