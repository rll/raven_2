/*
 * initializer.cpp
 *
 *  Created on: Oct 1, 2012
 *      Author: benk
 */

#include <raven/state/initializer.h>

 #include <Eigen/LU>

#include <raven/state/dof.h>
#include <raven/state/motor_filters/lpf.h>

#include "log.h"

#include "motor.h"

#define GB_RATIO (GEAR_BOX_GP42_TR/GEAR_BOX_GP32_TR * (1.08 * CAPSTAN_RADIUS_GP32/CAPSTAN_RADIUS_GP42))

std::vector<ArmData> DeviceInitializer::ARM_DATA;

std::vector<ArmData> DeviceInitializer::getArms() {
	return ARM_DATA;
}
void DeviceInitializer::addArm(int id, Arm::Type type, Arm::ToolType toolType) {
	ROS_INFO_STREAM("Adding arm: id " << id << " type:" << type.str() << " toolType:" << toolType.str());
	ARM_DATA.push_back(ArmData(id,type,toolType));
}
void DeviceInitializer::addArm(int id, const std::string& name, Arm::Type type, Arm::ToolType toolType) {
	ROS_INFO_STREAM("Adding arm " << name << " id " << id << " type:" << type.str() << " toolType:" << toolType.str());
	ARM_DATA.push_back(ArmData(id,type,name,toolType));
}

void initForwardCableCoupling(Eigen::MatrixXf& matrix,ArmPtr arm);
void initBackwardCableCoupling(Eigen::MatrixXf& matrix,const Eigen::MatrixXf& fwdMatrix, ArmPtr arm);

void
DeviceInitializer::initializeDevice(DevicePtr device) {
	device->timestamp_ = ros::Time(0);

	device->surgeonMode_ = false;
	device->arms_.clear();

	std::vector<ArmData> armDataList = getArms();
	for (int i=0;i<(int)armDataList.size();i++) {
		ArmData armData = armDataList[i];

		ArmPtr arm(new Arm(armData.id,armData.type,armData.name,armData.toolType));
		ROS_INFO_STREAM("Initializing arm " << arm->name());

		arm->basePose_ = arm->isGold() ? btTransform::getIdentity() : GREEN_ARM_BASE_POSE;

		float torqueSign = 1;
		// Positive torque -> positive joint angle change
		// Make sure encoders line up the same way.
		if (arm->isGold())
			torqueSign = -1;
		else
			torqueSign = 1;

		MotorPtr shoulder_motor(new Motor(Motor::Type::LARGE,Motor::TransmissionType::TA,Motor::CableType::LARGE));
		shoulder_motor->dacMax_ = SHOULDER_MAX_DAC;
		shoulder_motor->transmissionRatio_ = arm->isGold() ? SHOULDER_TR_GOLD_ARM : SHOULDER_TR_GREEN_ARM;
		arm->motors_.push_back(shoulder_motor);

		MotorPtr elbow_motor(new Motor(Motor::Type::LARGE,Motor::TransmissionType::TA,Motor::CableType::LARGE));
		elbow_motor->dacMax_ = ELBOW_MAX_DAC;
		elbow_motor->transmissionRatio_ = arm->isGold() ? ELBOW_TR_GOLD_ARM : ELBOW_TR_GREEN_ARM;
		arm->motors_.push_back(elbow_motor);

		MotorPtr insertion_motor(new Motor(Motor::Type::LARGE,Motor::TransmissionType::TA,Motor::CableType::LARGE));
		insertion_motor->dacMax_ = Z_INS_MAX_DAC;
		insertion_motor->transmissionRatio_ = arm->isGold() ? Z_INS_TR_GOLD_ARM : Z_INS_TR_GREEN_ARM;
		arm->motors_.push_back(insertion_motor);

		MotorPtr tool_rot_motor(new Motor(Motor::Type::SMALL,Motor::TransmissionType::TB,Motor::CableType::SMALL));
		tool_rot_motor->dacMax_ = TOOL_ROT_MAX_DAC;
		tool_rot_motor->transmissionRatio_ = arm->isGold() ? TOOL_ROT_TR_GOLD_ARM : TOOL_ROT_TR_GREEN_ARM;
		arm->motors_.push_back(tool_rot_motor);

		MotorPtr wrist_motor(new Motor(Motor::Type::SMALL,Motor::TransmissionType::TB,Motor::CableType::SMALL));
		wrist_motor->dacMax_ = WRIST_MAX_DAC;
		wrist_motor->transmissionRatio_ = arm->isGold() ? WRIST_TR_GOLD_ARM : WRIST_TR_GREEN_ARM;
		arm->motors_.push_back(wrist_motor);

		MotorPtr grasp1_motor(new Motor(Motor::Type::SMALL,Motor::TransmissionType::TB,Motor::CableType::SMALL));
		grasp1_motor->dacMax_ = GRASP1_MAX_DAC;
		grasp1_motor->transmissionRatio_ = arm->isGold() ? GRASP1_TR_GOLD_ARM : GRASP1_TR_GREEN_ARM;
		arm->motors_.push_back(grasp1_motor);

		MotorPtr grasp2_motor(new Motor(Motor::Type::SMALL,Motor::TransmissionType::TB,Motor::CableType::SMALL));
		grasp2_motor->dacMax_ = GRASP2_MAX_DAC;
		grasp2_motor->transmissionRatio_ = arm->isGold() ? GRASP2_TR_GOLD_ARM : GRASP2_TR_GREEN_ARM;
		arm->motors_.push_back(grasp2_motor);

		for (size_t m=0;m<arm->motors_.size();m++) {
			MotorPtr motor = arm->motors_[m];

			motor->encoderCountsPerRev_ = ENC_CNTS_PER_REV_R_II;

			motor->tauPerAmp_ = torqueSign;
			if (motor->type() == Motor::Type::LARGE) {
				motor->tauPerAmp_ *= T_PER_AMP_BIG_MOTOR;
				motor->dacCountsPerAmp_ = K_DAC_PER_AMP_HIGH_CURRENT;
			} else {
				motor->tauPerAmp_ *= T_PER_AMP_SMALL_MOTOR;
				motor->dacCountsPerAmp_ = K_DAC_PER_AMP_LOW_CURRENT;
			}

			if (motor->transmissionType() == Motor::TransmissionType::TA) {
				motor->tauPerAmp_ *= GEAR_BOX_TR_BIG_MOTOR;
				//motor->transmissionRatio_ = GEAR_BOX_TR_BIG_MOTOR;
			} else {
				motor->tauPerAmp_ *= GEAR_BOX_TR_SMALL_MOTOR;
				//motor->transmissionRatio_ = GEAR_BOX_TR_SMALL_MOTOR;
			}
		}

		if (arm->isGold()) {
			shoulder_motor->setEncoderOffset(SHOULDER_GOLD_KIN_OFFSET * ENC_CNT_PER_DEG * shoulder_motor->transmissionRatio()); // Degrees * enc/degree *
			elbow_motor->setEncoderOffset(ELBOW_GOLD_KIN_OFFSET * ENC_CNT_PER_DEG * elbow_motor->transmissionRatio());
			insertion_motor->setEncoderOffset(Z_INS_GOLD_KIN_OFFSET * insertion_motor->transmissionRatio() * ENC_CNT_PER_RAD);  // use enc/rad because conversion from meters to revolutions is in radians
		} else {
			shoulder_motor->setEncoderOffset(SHOULDER_GREEN_KIN_OFFSET * ENC_CNT_PER_DEG * shoulder_motor->transmissionRatio()); // Degrees * enc/degree *
			elbow_motor->setEncoderOffset(ELBOW_GREEN_KIN_OFFSET * ENC_CNT_PER_DEG * elbow_motor->transmissionRatio());
			insertion_motor->setEncoderOffset(Z_INS_GREEN_KIN_OFFSET * insertion_motor->transmissionRatio() * ENC_CNT_PER_RAD);  // use enc/rad because conversion from meters to revolutions is in radians
		}


		//arm->setMotorFilter(MotorFilterPtr(new LowPassMotorFilter(arm->motors_,arm->type_)));

		//Joint Defines
		#undef SHOULDER
		#undef ELBOW
		#undef Z_INS
		#undef TOOL_ROT
		#undef WRIST
		#undef GRASP1
		#undef GRASP2

		JointPtr shoulder_joint(new Joint(Joint::Type::SHOULDER_));
		arm->joints_.push_back(shoulder_joint);
		shoulder_joint->minPosition_ = SHOULDER_MIN_LIMIT;
		shoulder_joint->maxPosition_ = SHOULDER_MAX_LIMIT;
		shoulder_joint->homePosition_ = SHOULDER_HOME_ANGLE;

		JointPtr elbow_joint(new Joint(Joint::Type::ELBOW_));
		arm->joints_.push_back(elbow_joint);
		elbow_joint->minPosition_ = ELBOW_MIN_LIMIT;
		elbow_joint->maxPosition_ = ELBOW_MAX_LIMIT;
		elbow_joint->homePosition_ = ELBOW_HOME_ANGLE;

		JointPtr insertion_joint(new Joint(Joint::Type::INSERTION_));
		arm->joints_.push_back(insertion_joint);
		insertion_joint->minPosition_ = Z_INS_MIN_LIMIT;
		insertion_joint->maxPosition_ = Z_INS_MAX_LIMIT;
		insertion_joint->homePosition_ = Z_INS_HOME_ANGLE;

		JointPtr tool_rot_joint(new Joint(Joint::Type::TOOL_ROT_));
		arm->joints_.push_back(tool_rot_joint);
		tool_rot_joint->minPosition_ = TOOL_ROLL_MIN_LIMIT;
		tool_rot_joint->maxPosition_ = TOOL_ROLL_MAX_LIMIT;
		tool_rot_joint->homePosition_ = TOOL_ROT_HOME_ANGLE;

		JointPtr wrist_joint(new Joint(Joint::Type::WRIST_));
		arm->joints_.push_back(wrist_joint);
		wrist_joint->minPosition_ = TOOL_WRIST_MIN_LIMIT;
		wrist_joint->maxPosition_ = TOOL_WRIST_MAX_LIMIT;
		wrist_joint->homePosition_ = WRIST_HOME_ANGLE;

		JointPtr gripper1_joint(new Joint(Joint::Type::GRIPPER1_));
		arm->joints_.push_back(gripper1_joint);
		gripper1_joint->minPosition_ = TOOL_GRASP1_MIN_LIMIT;
		gripper1_joint->maxPosition_ = TOOL_GRASP1_MAX_LIMIT;
		gripper1_joint->homePosition_ = GRASP1_HOME_ANGLE;

		JointPtr gripper2_joint(new Joint(Joint::Type::GRIPPER2_));
		arm->joints_.push_back(gripper2_joint);
		gripper2_joint->minPosition_ = TOOL_GRASP2_MIN_LIMIT;
		gripper2_joint->maxPosition_ = TOOL_GRASP2_MAX_LIMIT;
		gripper2_joint->homePosition_ = GRASP2_HOME_ANGLE;

		std::vector<bool> cableCouplingForwardMask(arm->joints_.size(),true);

		for (size_t m=0;m<arm->joints_.size();m++) {
			arm->motors_[m]->hasMainJoint_ = true;
			arm->motors_[m]->mainJoint_ = arm->joints_[m]->type_;
		}

		JointPtr yaw_joint(new Joint(Joint::Type::YAW_));
		arm->joints_.push_back(yaw_joint);
		yaw_joint->minPosition_ = -TOOL_GRASP_LIMIT;
		yaw_joint->maxPosition_ = TOOL_GRASP_LIMIT;
		yaw_joint->homePosition_ = 0;
		cableCouplingForwardMask.push_back(false);

		JointPtr grasp_joint(new Joint(Joint::Type::GRASP_));
		arm->joints_.push_back(grasp_joint);
		grasp_joint->minPosition_ = 0;
		grasp_joint->maxPosition_ = 2 * TOOL_GRASP_LIMIT;
		grasp_joint->homePosition_ = GRASP1_HOME_ANGLE + GRASP1_HOME_ANGLE;
		cableCouplingForwardMask.push_back(false);

		for (size_t j=0;j<arm->joints_.size();j++) {
			JointPtr joint = arm->joints_[j];
			joint->speedLimit_ = fabs(joint->maxPosition_ - joint->homePosition_)/5;
		}

		arm->addJointCoupler(JointCouplerPtr(new YawGraspCoupler(arm->type())));

		Eigen::MatrixXf forwardCableCoupling = Eigen::MatrixXf::Zero(arm->joints_.size(),arm->motors_.size());
		Eigen::MatrixXf backwardCableCoupling = Eigen::MatrixXf::Zero(arm->motors_.size(),arm->joints_.size());

		initForwardCableCoupling(forwardCableCoupling,arm);
		//initBackwardCableCoupling(backwardCableCoupling);
		backwardCableCoupling.leftCols(arm->joints_.size()-2) = forwardCableCoupling.topRows(arm->joints_.size()-2).inverse();

		CableCouplerPtr cableCoupler(new CableCoupler(forwardCableCoupling,backwardCableCoupling));
		cableCoupler->setForwardMask(cableCouplingForwardMask);

		arm->cableCoupler_ = cableCoupler;

		Arm::init(arm);
		device->arms_.push_back(arm);
	}

	Device::init(device);
}

void
DeviceInitializer::initializeDeviceInstance() {
	if (!Device::INSTANCE) {
		Device::INSTANCE = DevicePtr(new Device(Device::RAVEN_ROBOT));

		initializeDevice(Device::INSTANCE);
	}
}

JointList
YawGraspCoupler::getBaseJoints(const JointList& joints) const {
	JointList bj;
	for (size_t i=0;i<joints.size();i++) {
		if (joints.at(i)->type() == Joint::Type::GRIPPER1_ || joints.at(i)->type() == Joint::Type::GRIPPER2_) {
			bj.push_back(joints.at(i));
		}
	}
	return bj;
}

JointList
YawGraspCoupler::getDependentJoints(const JointList& joints) const {
	JointList bj;
	for (size_t i=0;i<joints.size();i++) {
		if (joints.at(i)->type() == Joint::Type::YAW_ || joints.at(i)->type() == Joint::Type::GRASP_) {
			bj.push_back(joints.at(i));
		}
	}
	return bj;
}

void
YawGraspCoupler::coupleForward(const JointList& baseJoints,const JointList& depJoints) {
	JointPtr gripper1;
	JointPtr gripper2;
	BOOST_FOREACH(JointPtr bj,baseJoints) {
		if (bj->type() == Joint::Type::GRIPPER1_) { gripper1 = bj; }
		if (bj->type() == Joint::Type::GRIPPER2_) { gripper2 = bj; }
	}
	JointPtr yaw;
	JointPtr grasp;
	BOOST_FOREACH(JointPtr dj,depJoints) {
		if (dj->type() == Joint::Type::YAW_) { yaw = dj; }
		if (dj->type() == Joint::Type::GRASP_) { grasp = dj; }
	}
	if (armType_ == Arm::Type::GOLD) {
		yaw->setPosition((gripper2->position() - gripper1->position())/2);
		yaw->setVelocity((gripper2->velocity() - gripper1->velocity())/2);
		grasp->setPosition(gripper2->position() + gripper1->position());
		grasp->setVelocity(gripper2->velocity() + gripper1->velocity());
	} else {
		yaw->setPosition(-(gripper2->position() - gripper1->position())/2);
		yaw->setVelocity(-(gripper2->velocity() - gripper1->velocity())/2);
		grasp->setPosition(gripper2->position() + gripper1->position());
		grasp->setVelocity(gripper2->velocity() + gripper1->velocity());
	}
}

void
YawGraspCoupler::coupleBackward(const JointList& depJoints,const JointList& baseJoints) {
	//TODO: implement
}

void initForwardCableCoupling(Eigen::MatrixXf& M,ArmPtr arm) {
	float tr1=0, tr2=0, tr3=0, tr4=0, tr5=0, tr6=0, tr7=0;


	tr1 = arm->motor(0)->transmissionRatio();
	tr2 = arm->motor(1)->transmissionRatio();
	//TODO: THIS IS BACKWARDS
	tr3 = arm->motor(3)->transmissionRatio();
	tr4 = arm->motor(2)->transmissionRatio();
	tr5 = arm->motor(4)->transmissionRatio();
	tr6 = arm->motor(5)->transmissionRatio();
	tr7 = arm->motor(6)->transmissionRatio();

	// Forward Cable Coupling equations
	//   Originally based on 11/7/2005, Mitch notebook pg. 169
	//   Updated from UCSC code.  Code simplified by HK 8/11
	M(0,0) = 1.0/tr1; //th1 = (1.0/tr1) * m1;
	M(1,1) = 1.0/tr2; //th2 = (1.0/tr2) * m2;
	M(2,2) = 1.0/tr4; //d4  = (1.0/tr4) * m4;



	// Tool degrees of freedom ===========================================
	M(3,2) = -(1.0/tr3) / GB_RATIO;
	M(3,3) =  1.0/tr3; //th3 = (1.0/tr3) * (m3 - m4/GB_RATIO);
	M(4,2) = -(1.0/tr5) / GB_RATIO;
	M(4,4) =  1.0/tr5;//th5 = (1.0/tr5) * (m5 - m4/GB_RATIO);

	if (arm->toolType() == Arm::ToolType::GRASPER_10MM) {
		M(5,2) = -(1.0/tr6) / GB_RATIO;
		M(5,5) =   1.0/tr6; //th6 = (1.0/tr6) * (m6 - m4/GB_RATIO);
		M(6,2) = -(1.0/tr7) / GB_RATIO;
		M(6,6) =   1.0/tr7; //th7 = (1.0/tr7) * (m7 - m4/GB_RATIO);
	} else if (arm->toolType() == Arm::ToolType::GRASPER_8MM) {
		log_err("CableCoupling: tool is 8mm IMPLEMENT");
		// Note: sign of the last term changes for GOLD vs GREEN arm
		/*
		int sgn = (mech->type == GOLD_ARM) ? 1 : -1;
		th6 = (1.0/tr6) * (m6 - m4/GB_RATIO - sgn * (tr5*th5) * (tr5/tr6));
		th7 = (1.0/tr7) * (m7 - m4/GB_RATIO + sgn *(tr5*th5) * (tr5/tr6));
		*/
	} else if (arm->toolType() == Arm::ToolType::NONE)  {// there's tool in the robot
		log_err("CableCoupling: tool is none IMPLEMENT");
		// coupling goes until the tool adapter pulleys
		/*
		th6 = (1.0/tr6) * (m6 - m4/GB_RATIO);
		th7 = (1.0/tr7) * (m7 - m4/GB_RATIO);
		*/
	}
}
