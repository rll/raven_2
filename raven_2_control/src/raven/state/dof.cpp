/*
 * dof.cpp
 *
 *  Created on: Oct 1, 2012
 *      Author: benk
 */




#include <raven/state/dof.h>

#include <iostream>


static int numJ = 0;
Joint::Joint(Type type) : Updateable(), type_(type), state_(Joint::State::NOT_READY), position_(0), velocity_(0), minPosition_(0), maxPosition_(0), homePosition_(0), speedLimit_(0) {
	//printf("+J  %i %p\n",++numJ,this);
	toolJoint_ = type_==Type::TOOL_ROT_ || type_==Type::WRIST_ || type_ == Type::GRIPPER1_ || type_ == Type::GRIPPER2_ || type_ == Type::YAW_ || type_ == Type::GRASP_;
}

Joint::~Joint() {
	//printf("-J  %i %p\n",--numJ,this);
}

JointPtr
Joint::clone() const {
	//printf("+J  %i %p\n",++numJ,this);
	return JointPtr(new Joint(*this));
}

void
Joint::cloneInto(JointPtr& joint) const {
	if (!joint) {
		JointPtr newJoint = clone();
		joint.swap(newJoint);
		return;
	}
	*joint = *this;
}

std::string
Joint::str() const {
	std::stringstream ss;
	ss << "[";
	ss << type_.str() << " ";
	//ss << state_;
	ss << "p:" << position_ << ",";
	ss << "v:" << velocity_ << ",";
	ss << "]";
	return ss.str();
}

static int numM = 0;
Motor::Motor(Type type, TransmissionType transType, CableType cableType) :
		Updateable(), type_(type), transmissionType_(transType), cableType_(cableType),
		position_(0), velocity_(0), torque_(0), gravitationalTorqueEstimate_(0), dacCommand_(0), encoderValue_(0), encoderOffset_(0),
		encoderCountsPerRev_(0), dacMax_(0), transmissionRatio_(0), tauPerAmp_(0), dacCountsPerAmp_(0) {
	//printf("+M  %i %p\n",++numM,this);
}

Motor::~Motor() {
	//printf("-M  %i %p\n",--numM,this);
}

MotorPtr
Motor::clone() const {
	//printf("+M  %i %p\n",++numM,this);
	return MotorPtr(new Motor(*this));
}

void
Motor::cloneInto(MotorPtr& motor) const {
	if (!motor) {
		MotorPtr newMotor = clone();
		motor.swap(newMotor);
		return;
	}
	*motor = *this;
}

void
Motor::setPosition(float pos) {
	position_ = pos;
	//encoderValue_ = position_ * encoderCountsPerRev_ / (2.0*M_PI) + encoderOffset_;
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
	torque_ = torque;

	const float TFmotor     = 1 / tauPerAmp_;    // Determine the motor TF  = 1/(tau per amp)
	const float TFamplifier =     dacCountsPerAmp_;    // Determine the amplifier TF = (DAC_per_amp)

	int DACVal = (int)(torque_ * TFmotor * TFamplifier);  //compute DAC value: DAC=[tau*(amp/torque)*(DACs/amp)]

	//Perform range checking and convert to short int
	//Note: saturateShort saturates at max value for short int.
	saturateShort(DACVal, &dacCommand_);

	updateTimestamp();
}

float
Motor::torqueMax() const {
	const float TFmotor     = 1 / tauPerAmp_;    // Determine the motor TF  = 1/(tau per amp)
	const float TFamplifier =     dacCountsPerAmp_;    // Determine the amplifier TF = (DAC_per_amp)

	return ((float)dacMax_) / (TFmotor * TFamplifier);
}

void
Motor::setEncoderValue(int val,bool updatePosition) {
	encoderValue_ = val;
	if (updatePosition) {
		position_ = (2.0*M_PI) * (1.0/((float)encoderCountsPerRev_)) * (encoderValue_ - encoderOffset_);
	}
	updateTimestamp();
}

void
Motor::setEncoderOffset(int offset) {
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
	Eigen::VectorXf motors_p = Motor::positionVector(motors);
	Eigen::VectorXf motors_v = Motor::velocityVector(motors);

	Eigen::VectorXf joints_p = forwardMatrix_ * motors_p;
	Eigen::VectorXf joints_v = forwardMatrix_ * motors_v;
	for (size_t i=0;i<joints.size();i++) {
		if (!forwardMask_[i]) { continue; }
		joints.at(i)->position_ = joints_p[i];
		joints.at(i)->velocity_ = joints_v[i];
	}
}

void
CableCoupler::coupleBackward(const JointList& joints,const MotorList& motors) {
	Eigen::VectorXf joints_p = Joint::positionVector(joints);
	Eigen::VectorXf joints_v = Joint::velocityVector(joints);

	Eigen::VectorXf motors_p = backwardMatrix_ * joints_p;
	Eigen::VectorXf motors_v = backwardMatrix_ * joints_v;
	for (size_t i=0;i<motors.size();i++) {
		if (!backwardMask_[i]) { continue; }
		motors.at(i)->position_ = motors_p[i];
		motors.at(i)->velocity_ = motors_v[i];
	}
}
