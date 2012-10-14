/*
 * lpf.cpp
 *
 *  Created on: Oct 4, 2012
 *      Author: benk
 */

#include <raven/state/motor_filters/lpf.h>

#include "defines.h"

static int numLF = 0;
template<int order>
LowPassMotorFilter<order>::LowPassMotorFilter(const MotorList& motors,Arm::Type armType) : MotorFilter(motors), armType_(armType),
		order_(ORDER), history_(motors.size()), filteredHistory_(motors.size()), lastCallTime_(0) {
	//printf("+LF %i %p\n",++numLF,this);
	//A_.resize(order_+1);
	A_ << 1.0000  ,  1.5189  , -0.9600  ,  0.2120;
	//B_.resize(order_+1);
	B_ << 0.02864 ,  0.08591 ,  0.08591 ,  0.02864;
}

template<int order>
void
LowPassMotorFilter<order>::internalCloneInto(MotorFilterPtr& other, const MotorList& newMotors) const {
	if (!boost::dynamic_pointer_cast<LowPassMotorFilter>(other)) {
		MotorFilterPtr newMotorFilter = clone(newMotors);
		other.swap(newMotorFilter);
		return;
	}
	*other = *this;
}

template<int order>
void
LowPassMotorFilter<order>::internalApplyUpdate() {
	ros::Time callTime = ros::Time::now();
	for (size_t i=0;i<motorsForUpdate_.size();i++) {
		bool resized = false;

		if (history_[i].empty()) {
			history_[i].resize(order,CloningWrapper<Motor>(motorsForUpdate_[i]));
			filteredHistory_[i].resize(order,CloningWrapper<Motor>(motorsForUpdate_[i]));
			resized = true;
		}

		//Eigen::VectorXf pos(order_+1);
		Eigen::Matrix<float,order+1,1> pos;
		pos.setZero();
		pos[0] = motorsForUpdate_[i]->position();
		pos.tail(order) = Motor::positionVector(history(i));

		Eigen::Matrix<float,order+1,1> filteredPos = Motor::positionVector(filteredHistory(i));

		float newFilteredPos = B_.dot(pos) + A_.tail(order).dot(filteredPos);

		float newFilteredVel = 0;
		if (!lastCallTime_.isZero()) {
			//FIXME: use call times
			newFilteredVel = (newFilteredPos - filteredPos[0]) / STEP_PERIOD; //(callTime-lastCallTime_).toSec();
		}

		motors_[i]->setEncoderValue(motorsForUpdate_[i]->encoderValue(),false);
		motors_[i]->setEncoderOffset(motorsForUpdate_[i]->encoderOffset());
		motors_[i]->setPosition(newFilteredPos);
		motors_[i]->setVelocity(newFilteredVel);

		history_[i].push_front(CloningWrapper<Motor>(motorsForUpdate_[i]));
		filteredHistory_[i].push_front(CloningWrapper<Motor>(motors_[i]));
	}
	lastCallTime_ = callTime;
}

template<int order>
LowPassMotorFilter<order>::~LowPassMotorFilter() {
	//printf("-LF %i %p\n",--numLF,this);
}

template<int order>
void
LowPassMotorFilter<order>::reset() {
	for (size_t i=0;i<motors_.size();i++) {
		history_[i].clear();
		filteredHistory_[i].clear();
	}
}


template<int order>
MotorList
LowPassMotorFilter<order>::history(size_t i) const {
	MotorList motors;
	motors.insert(motors.end(),history_[i].begin(),history_[i].end());
	return motors;
}

template<int order>
MotorList
LowPassMotorFilter<order>::filteredHistory(size_t i) const {
	MotorList motors;
	motors.insert(motors.end(),filteredHistory_[i].begin(),filteredHistory_[i].end());
	return motors;
}

template<int order>
std::string
LowPassMotorFilter<order>::str() const {
	return "LPMF";
}

template<int order>
MotorFilterPtr
LowPassMotorFilter<order>::clone(const MotorList& newMotors) const {
	return MotorFilterPtr(new LowPassMotorFilter(newMotors,armType_));
}
