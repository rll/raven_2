/*
 * arm.cpp
 *
 *  Created on: Oct 1, 2012
 *      Author: benk
 */

#include <raven/state/arm.h>
#include <boost/foreach.hpp>

#include "defines.h"
#include "log.h"

#include <algorithm>

const Arm::IdType Arm::ALL_ARMS = -1;

static int numA = 0;
Arm::Arm(int id, Type type, const std::string& name, ToolType toolType) : Updateable(true,true), id_(id), type_(type), name_(name), enabled_(true), toolType_(toolType),
	basePose_(btTransform::getIdentity()), cableCoupler_(), kinematicSolver_(new KinematicSolver(this)) {
	//printf("+A  %i %p\n",++numA,this);
}
Arm::Arm(const Arm& other) : Updateable(other), id_(other.id_), type_(other.type_), name_(other.name_), enabled_(other.enabled_), toolType_(other.toolType_),
		basePose_(other.basePose_) {
	for (MotorList::const_iterator itr=other.motors_.begin();itr!=other.motors_.end();itr++) {
		motors_.push_back((*itr)->clone());
	}
	MotorFilterPtr newStateMotorFilter = other.stateMotorFilter_->clone(motors_);
	stateMotorFilter_.swap(newStateMotorFilter);

	MotorFilterPtr newControlMotorFilter = other.controlMotorFilter_->clone(motors_);
	controlMotorFilter_.swap(newControlMotorFilter);

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
	//TRACER_VERBOSE_ENTER_SCOPE("Arm[%s]@%p::clone()",name_.c_str(),this);
	ArmPtr newArm(new Arm(*this));
	//TRACER_VERBOSE_PRINT("Arm clone is %p",newArm.get());

	init(newArm);

	return newArm;
}

void
Arm::cloneInto(ArmPtr& other) const {
	//TRACER_VERBOSE_ENTER_SCOPE("Arm[%s]@%p::cloneInto()",name_.c_str(),this);
	if (!other) {
		ArmPtr newArm = clone();
		other.swap(newArm);
		return;
	}
	//TRACER_VERBOSE_PRINT("Arm clone is %p",other.get());
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
	stateMotorFilter_->cloneInto(other->stateMotorFilter_,other->motors_);
	controlMotorFilter_->cloneInto(other->controlMotorFilter_,other->motors_);

	other->cableCoupler_ = cableCoupler_;

	other->jointCouplers_ = jointCouplers_;

	kinematicSolver_->cloneInto(other->kinematicSolver_,other.get());

	init(other);
}

void
Arm::init(ArmPtr arm) {
	TRACER_VERBOSE_ENTER_SCOPE("Arm[%s]@%p::init()",arm->name_.c_str(),arm.get());
	arm->updateJointsFromMotors();

	BOOST_FOREACH(MotorPtr m,arm->motors_) {
		m->setUpdateableParent(arm);
	}
	if (!arm->stateMotorFilter_) {
		arm->stateMotorFilter_.reset(new NullMotorFilter(arm->motors_));
	}
	if (!arm->controlMotorFilter_) {
		arm->controlMotorFilter_.reset(new NullMotorFilter(arm->motors_));
	}

	BOOST_FOREACH(JointPtr j,arm->joints_) {
		j->setUpdateableParent(arm);
	}

}


bool
Arm::processNotification(Updateable* sender) {
	TRACER_ENTER_SCOPE("Arm[%s]@%p::processNotification(%s)",name_.c_str(),this,typeid(*sender).name());
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
	TRACER_ENTER_SCOPE("Arm[%s]@%p::updateJointsFromMotors()",name_.c_str(),this);
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
	TRACER_ENTER_SCOPE("Arm[%s]@%p::updateMotorsFromJoints()",name_.c_str(),this);
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
Arm::internalUpdate() {
	TRACER_ENTER_SCOPE("Arm[%s]@%p::internalUpdate()",name_.c_str(),this);
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

	return true;//Updateable::update();
}



JointPtr
Arm::getJointByOldType(int type) {
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

JointConstPtr
Arm::getJointByOldType(int type) const {
	return JointConstPtr(const_cast<Arm*>(this)->getJointByOldType(type));
}

MotorPtr
Arm::getMotorByOldType(int type) {
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

MotorConstPtr
Arm::getMotorByOldType(int type) const {
	return MotorConstPtr(const_cast<Arm*>(this)->getMotorByOldType(type));
}




MotorFilterPtr Arm::stateMotorFilter() {
	TRACER_VERBOSE_PRINT("Arm[%s]@%p state motor filter",name_.c_str(),this);
	return stateMotorFilter_;
}
MotorFilterConstPtr Arm::stateMotorFilter() const {
	TRACER_VERBOSE_PRINT("Arm[%s]@%p state motor filter (const)",name_.c_str(),this);
	return stateMotorFilter_;
}

void
Arm::setStateMotorFilter(MotorFilterPtr filter) {
	if (!filter) {
		stateMotorFilter_.reset(new NullMotorFilter(motors_));
	} else {
		if (filter->motors() == motors_) {
			stateMotorFilter_ = filter;
		} else {
			stateMotorFilter_ = filter->clone(motors_);
		}
	}
}

MotorFilterPtr Arm::controlMotorFilter() {
	TRACER_VERBOSE_PRINT("Arm[%s]@%p control motor filter",name_.c_str(),this);
	return controlMotorFilter_;
}
MotorFilterConstPtr Arm::controlMotorFilter() const {
	TRACER_VERBOSE_PRINT("Arm[%s]@%p control motor filter (const)",name_.c_str(),this);
	return controlMotorFilter_;
}

void
Arm::setcontrolMotorFilter(MotorFilterPtr filter) {
	if (!filter) {
		controlMotorFilter_.reset(new NullMotorFilter(motors_));
	} else {
		if (filter->motors() == motors_) {
			controlMotorFilter_ = filter;
		} else {
			controlMotorFilter_ = filter->clone(motors_);
		}
	}
}
