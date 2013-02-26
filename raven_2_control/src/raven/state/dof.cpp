/*
 * dof.cpp
 *
 *  Created on: Oct 1, 2012
 *      Author: benk
 */




#include <raven/state/dof.h>

#include <iostream>

#include <boost/algorithm/string.hpp>
#include <string>


static int numJ = 0;
Joint::Joint(IdType id,Type type) : Updateable(), id_(id), type_(type), hasMainMotor_(false), state_(Joint::State::NOT_READY), position_(0), velocity_(0), minPosition_(0), maxPosition_(0), homePosition_(0), speedLimit_(0) {
	toolJoint_ = id_==IdType::ROTATION_ || id_==IdType::WRIST_ || id_ == IdType::FINGER1_ || id_ == IdType::FINGER2_ || id_ == IdType::YAW_ || id_ == IdType::GRASP_;
}

Joint::~Joint() {

}

JointPtr
Joint::clone() const {
	//TRACER_VERBOSE_ENTER_SCOPE("Joint[%s]@%p::clone()",id_.str(),this);
	JointPtr newJoint(new Joint(*this));
	//TRACER_VERBOSE_PRINT("Joint clone is %p",newJoint.get());
	return newJoint;
}

void
Joint::cloneInto(JointPtr& other) const {
	//TRACER_VERBOSE_ENTER_SCOPE("Joint[%s]@%p::cloneInto()",id_.str(),this);
	if (!other) {
		JointPtr newJoint = clone();
		other.swap(newJoint);
		return;
	}
	//TRACER_VERBOSE_PRINT("Joint clone is %p",other.get());
	*other = *this;
}

Joint::IdType Joint::id() const { return id_; }
bool Joint::isToolJoint() const { return toolJoint_; }
std::string Joint::idString() const { return id_.str(); }
std::string Joint::idStringUpper() const { return boost::to_upper_copy(idString()); }
std::string Joint::idStringLower() const { return boost::to_lower_copy(idString()); }

Joint::Type Joint::type() const { return type_; }

bool Joint::hasMainMotor() const { return hasMainMotor_; }
Motor::IdType Joint::mainMotor() const { if (!hasMainMotor_) { throw std::runtime_error("No main motor!"); } else { return mainMotor_; } }

Joint::State Joint::state() const { return state_; }

void Joint::setState(State state ) { state_ = state; updateTimestamp(); }

float Joint::position() const { return position_; }
float Joint::velocity() const { return velocity_; }

float Joint::minPosition() const { return minPosition_; }
float Joint::maxPosition() const { return maxPosition_; }

float Joint::homePosition() const { return homePosition_; }
float Joint::speedLimit() const { return speedLimit_; }

std::string
Joint::str() const {
	std::stringstream ss;
	ss << "[";
	ss << id_.str() << " ";
	//ss << state_;
	ss << "p:" << position_ << ",";
	ss << "v:" << velocity_ << ",";
	ss << "]";
	return ss.str();
}

void Joint::setPosition(float pos) {
	TRACER_ENTER_SCOPE("Joint[%s]@%p::setPosition(%f)",id_.str(),this,pos);
	position_ = pos;
	updateTimestamp();
}
void Joint::setVelocity(float vel) {
	TRACER_ENTER_SCOPE("Joint[%s]@%p::setVelocity(%f)",id_.str(),this,vel);
	velocity_ = vel;
	updateTimestamp();
}


static int numM = 0;
Motor::Motor(IdType id, Type type, TransmissionType transType, CableType cableType) :
		Updateable(), id_(id), name_(id.str()), type_(type), transmissionType_(transType), cableType_(cableType), hasMainJoint_(false),
		position_(0), velocity_(0), torque_(0), gravitationalTorqueEstimate_(0), dacCommand_(0), encoderValue_(0), encoderOffset_(0),
		encoderCountsPerRev_(0), dacMax_(0), transmissionRatio_(0), tauPerAmp_(0), dacCountsPerAmp_(0) {
	//printf("+M  %i %p\n",++numM,this);
}

Motor::~Motor() {
	//printf("-M  %i %p\n",--numM,this);
}

MotorPtr
Motor::clone() const {
	//TRACER_VERBOSE_ENTER_SCOPE("Motor[%s]@%p::clone()",id_.str(),this);
	MotorPtr newMotor(new Motor(*this));
	TRACER_VERBOSE_PRINT("Motor clone is %p",newMotor.get());
	return newMotor;
}

void
Motor::cloneInto(MotorPtr& other) const {
	//TRACER_VERBOSE_ENTER_SCOPE("Motor[%s]@%p::cloneInto()",id_.str(),this);
	if (!other) {
		MotorPtr newMotor = clone();
		other.swap(newMotor);
		return;
	}
	//TRACER_VERBOSE_PRINT("Motor clone is %p",other.get());
	*other = *this;
}

Motor::IdType Motor::id() const { return id_; }
std::string Motor::name() const { return name_; }

Motor::Type Motor::type() const { return type_; }
Motor::TransmissionType Motor::transmissionType() const { return transmissionType_; }
CableType Motor::cableType() const { return cableType_; }

bool Motor::hasMainJoint() const { return hasMainJoint_; }
Joint::IdType Motor::mainJoint() const { if (!hasMainJoint_) { throw std::runtime_error("No main joint!"); } else { return mainJoint_; } }

float Motor::position() const { return position_; }
float Motor::velocity() const { return velocity_; }
float Motor::torque() const { return torque_; }
float Motor::gravitationalTorqueEstimate() const { return gravitationalTorqueEstimate_; }
short int Motor::dacCommand() const { return dacCommand_; }
int Motor::encoderValue() const { return encoderValue_; }
int Motor::encoderOffset() const { return encoderOffset_; }

int Motor::encoderCountsPerRev() const { return encoderCountsPerRev_; }
int Motor::dacMax() const { return dacMax_; }

float Motor::transmissionRatio() const { return transmissionRatio_; }
float Motor::tauPerAmp() const { return tauPerAmp_; }
float Motor::dacCountsPerAmp() const { return dacCountsPerAmp_; }



void
Motor::setPosition(float pos) {
	TRACER_ENTER_SCOPE("Motor[%s]@%p::setPosition(%f)",id_.str(),this,pos);
	position_ = pos;
	//encoderValue_ = position_ * encoderCountsPerRev_ / (2.0*M_PI) + encoderOffset_;
	updateTimestamp();
}

void Motor::setVelocity(float vel) {
	TRACER_ENTER_SCOPE("Motor[%s]@%p::setVelocity(%f)",id_.str(),this,vel);
	velocity_ = vel;
	updateTimestamp();
}

int saturateShort(int value, short int *target) {
	//Short maximum and minimum
	#define SHORT_MAX 32767
	#define SHORT_MIN -32768

	//Return values
	#define SHORT_OVERFLOW    1
	#define SHORT_UNDERFLOW  -1

    //Overflow
    if (value > SHORT_MAX)
    {
        *target = SHORT_MAX;
        return SHORT_OVERFLOW;
    }
    //Underflow
    else if (value < SHORT_MIN)
    {
        *target = SHORT_MIN;
        return SHORT_UNDERFLOW;
    }
    //No problems
    else
    {
        *target = value;
        return 0;
    }
}

void
Motor::setTorque(float torque) {
	TRACER_ENTER_SCOPE("Motor[%s]@%p::setTorque(%f)",id_.str(),this,torque);
	torque_ = torque;

	const float TFmotor     = 1 / tauPerAmp_;    // Determine the motor TF  = 1/(tau per amp)
	const float TFamplifier =     dacCountsPerAmp_;    // Determine the amplifier TF = (DAC_per_amp)

	int DACVal = (int)(torque_ * TFmotor * TFamplifier);  //compute DAC value: DAC=[tau*(amp/torque)*(DACs/amp)]

	//Perform range checking and convert to short int
	//Note: saturateShort saturates at max value for short int.
	saturateShort(DACVal, &dacCommand_);
	TRACER_PRINT("Motor[%i]@%p::setDacCommand(%hi)",id_.str(),this,dacCommand_);

	updateTimestamp();
}

float
Motor::torqueMax() const {
	const float TFmotor     = 1 / tauPerAmp_;    // Determine the motor TF  = 1/(tau per amp)
	const float TFamplifier =     dacCountsPerAmp_;    // Determine the amplifier TF = (DAC_per_amp)

	return ((float)dacMax_) / (TFmotor * TFamplifier);
}

void Motor::setGravitationalTorqueEstimate(float gte) {
	gravitationalTorqueEstimate_ = gte;
	updateTimestamp();
}

void Motor::setDacCommand(short int cmd) {
	TRACER_ENTER_SCOPE("Motor[%s]@%p::setDacCommand(%hi)",id_.str(),this,cmd);
	dacCommand_ = cmd;
	updateTimestamp();
}


void
Motor::setEncoderValue(int val,bool updatePosition) {
	TRACER_ENTER_SCOPE("Motor[%s]@%p::setEncoderValue(%i,%i)",id_.str(),this,val,updatePosition);
	encoderValue_ = val;
	if (updatePosition) {
		position_ = (2.0*M_PI) * (1.0/((float)encoderCountsPerRev_)) * (encoderValue_ - encoderOffset_);
	}
	updateTimestamp();
}

void
Motor::setEncoderOffset(int offset) {
	TRACER_ENTER_SCOPE("Motor[%s]@%p::setEncoderOffset(%i)",id_.str(),this,offset);
	encoderOffset_ = offset;
	position_ = (2.0*M_PI) * (1.0/((float)encoderCountsPerRev_)) * (encoderValue_ - encoderOffset_);
	updateTimestamp();
}

Eigen::VectorXf
Joint::positionVector(const JointList& joints) {
	Eigen::VectorXf v;
	v.resize(joints.size());
	for (size_t i=0;i<joints.size();i++) {
		v[i] = joints.at(i)->position();
	}
	return v;
}
Eigen::VectorXf
Joint::velocityVector(const JointList& joints) {
	Eigen::VectorXf v;
	v.resize(joints.size());
	for (size_t i=0;i<joints.size();i++) {
		v[i] = joints.at(i)->velocity();
	}
	return v;
}

Eigen::VectorXf
Motor::positionVector(const MotorList& motors) {
	Eigen::VectorXf v;
	v.resize(motors.size());
	for (size_t i=0;i<motors.size();i++) {
		v[i] = motors.at(i)->position();
	}
	return v;
}
Eigen::VectorXf
Motor::velocityVector(const MotorList& motors) {
	Eigen::VectorXf v;
	v.resize(motors.size());
	for (size_t i=0;i<motors.size();i++) {
		v[i] = motors.at(i)->velocity();
	}
	return v;
}
Eigen::VectorXf
Motor::torqueVector(const MotorList& motors) {
	Eigen::VectorXf v;
	v.resize(motors.size());
	for (size_t i=0;i<motors.size();i++) {
		v[i] = motors.at(i)->torque();
	}
	return v;
}

std::string
Motor::str() const {
	std::stringstream ss;
	ss << "[";
	ss << "p:" << position_ << ",";
	ss << "v:" << velocity_ << ",";
	ss << "e:" << encoderValue_ << ",";
	ss << "o:" << encoderOffset_;
	ss << "]";
	return ss.str();
}

MotorFilter::MotorFilter(const MotorList& motors) : motors_(motors), motorsForUpdateReady_(false), motorsForUpdate_(motors.size()) {

}
MotorFilter::MotorFilter(const MotorFilter& other) : motors_(other.motors_), motorsForUpdateReady_(false), motorsForUpdate_(other.motors_.size()) {

}

MotorList MotorFilter::motors() {
	return motors_;
}

ConstMotorList MotorFilter::motors() const {
	return constList(motors_);
}

MotorList MotorFilter::getMotorsForUpdate() {
	MotorList list;
	getMotorsForUpdate(list);
	return list;
}

void MotorFilter::getMotorsForUpdate(MotorList& list) {
	TRACER_ENTER_SCOPE("MotorFilter::getMotorsForUpdate()");
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

void MotorFilter::applyUpdate() {
	TRACER_ENTER_SCOPE("MotorFilter::applyUpdate()");
	if (!motorsForUpdateReady_) {
		return;
	}
	internalApplyUpdate();
	for (size_t i=0;i<motors_.size();i++) {
		motors_[i]->update();
	}
	motorsForUpdateReady_ = false;
}

void MotorFilter::cloneInto(MotorFilterPtr& other, const MotorList& newMotors) const {
	internalCloneInto(other,newMotors);
	other->motorsForUpdateReady_ = false;
}


static int numNF = 0;
NullMotorFilter::NullMotorFilter(const MotorList& motors) : MotorFilter(motors) {
	//printf("+NF %i %p\n",++numNF,this);
}

NullMotorFilter::~NullMotorFilter() {
	//printf("-NF %i %p\n",--numNF,this);
}

MotorFilterPtr
NullMotorFilter::clone(const MotorList& newMotors) const {
	//printf("+NF %i %p\n",++numNF,this);
	return MotorFilterPtr(new NullMotorFilter(newMotors));
}

static int numCC = 0;
CableCoupler::CableCoupler(const Eigen::MatrixXf& forwardMatrix,const Eigen::MatrixXf& backwardMatrix) :
		forwardMatrix_(forwardMatrix), forwardMask_(forwardMatrix.rows(),true), backwardMatrix_(backwardMatrix),backwardMask_(backwardMatrix.rows(),true) {
	//printf("+CC %i %p\n",++numCC,this);
}

CableCoupler::~CableCoupler() {
	//printf("-CC %i %p\n",--numCC,this);
}


CableCouplerPtr
CableCoupler::clone() const {
	//printf("+CC %i %p\n",++numCC,this);
	CableCouplerPtr newPtr(new CableCoupler(*this));
	return newPtr;
}

void
CableCoupler::cloneInto(CableCouplerPtr& other) const {
	if (!other) {
		CableCouplerPtr newCC = clone();
		other.swap(newCC);
		return;
	}
	*other = *this;
}

void
CableCoupler::coupleForward(const MotorList& motors,const JointList& joints) {
	TRACER_ENTER_SCOPE("CableCoupler::coupleForward()");
	Eigen::VectorXf motors_p = Motor::positionVector(motors);
	Eigen::VectorXf motors_v = Motor::velocityVector(motors);

	Eigen::VectorXf joints_p = forwardMatrix_ * motors_p;
	Eigen::VectorXf joints_v = forwardMatrix_ * motors_v;

	for (size_t i=0;i<motors.size();i++) {
		TRACER_PRINT("Motor %s pos=%f, vel=%f",joints.at(i)->id_.str(),motors_p[i],motors_v[i]);
	}

	for (size_t i=0;i<joints.size();i++) {
		if (!forwardMask_[i]) {
			TRACER_VERBOSE_PRINT("skipping joint %i",i);
			continue;
		}
		TRACER_PRINT("setting joint %s pos=%f, vel=%f",joints.at(i)->id_.str(),joints_p[i],joints_v[i]);
		joints.at(i)->position_ = joints_p[i];
		joints.at(i)->velocity_ = joints_v[i];
	}
}

void
CableCoupler::coupleBackward(const JointList& joints,const MotorList& motors) {
	TRACER_ENTER_SCOPE("CableCoupler::coupleBackward()");
	Eigen::VectorXf joints_p = Joint::positionVector(joints);
	Eigen::VectorXf joints_v = Joint::velocityVector(joints);

	Eigen::VectorXf motors_p = backwardMatrix_ * joints_p;
	Eigen::VectorXf motors_v = backwardMatrix_ * joints_v;
	for (size_t i=0;i<motors.size();i++) {
		if (!backwardMask_[i]) {
			TRACER_VERBOSE_PRINT("skipping joint %i",i);
			continue;
		}
		TRACER_PRINT("setting motor %i pos=%f, vel=%f",i,joints_p[i],joints_v[i]);
		motors.at(i)->position_ = motors_p[i];
		motors.at(i)->velocity_ = motors_v[i];
	}
}
