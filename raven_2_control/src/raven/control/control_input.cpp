/*
 * control_input.cpp
 *
 *  Created on: Oct 9, 2012
 *      Author: benk
 */

#include <raven/control/control_input.h>
#include <raven/control/controller.h>

#include "defines.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <raven/state/runlevel.h>

boost::mutex inputMutex;
std::map<std::pair<Arm::IdType,std::string>,ControlInputPtr> ControlInput::CONTROL_INPUT;


boost::recursive_mutex oldInputMutex;
OldControlInputPtr ControlInput::OLD_CONTROL_INPUT;

boost::mutex masterMutex;
std::map<Arm::IdType,MasterMode2> MasterMode2::MASTER_MODES;
std::map<Arm::IdType,std::set<MasterMode2> > MasterMode2::MASTER_MODE_CONFLICTS;
std::map<Arm::IdType,ros::Time> MasterMode2::LAST_CHECK_TIMES;

MasterMode2 const MasterMode2::NONE = MasterMode2("");

MasterMode2
MasterMode2::get(Arm::IdType armId) {
	boost::mutex::scoped_lock(masterMutex);
	return MASTER_MODES[armId];
}

bool
MasterMode2::check(Arm::IdType armId, const MasterMode2& mode) {
	if (mode.isNone()) {
			return false;
		} else if (!RunLevel::hasHomed()) {
			return false;
		}
		bool succeeded = false;
		boost::mutex::scoped_lock(masterMutex);
		printf("checking master mode\n");
		ros::Time now = ros::Time::now();
		if (armId == Arm::ALL_ARMS) {
			bool any_failed = false;
			FOREACH_ARM_ID(armId) {
				LAST_CHECK_TIMES[armId] = now;
				if (MASTER_MODES[armId] == mode || MASTER_MODES[armId].isNone()) {
					MASTER_MODES[armId] = mode;
				} else {
					MASTER_MODE_CONFLICTS[armId].insert(mode);
					any_failed = true;
				}
			}
			succeeded = !any_failed;
		} else {
			LAST_CHECK_TIMES[armId] = now;
			if (MASTER_MODES[armId] == mode || MASTER_MODES[armId].isNone()) {
				MASTER_MODES[armId] = mode;
				succeeded = true;
			} else {
				MASTER_MODE_CONFLICTS[armId].insert(mode);
			}
		}
		return succeeded;
}

bool
MasterMode2::reset(Arm::IdType armId) {
	boost::mutex::scoped_lock(masterMutex);
	if (armId == Arm::ALL_ARMS) {
		MASTER_MODES.clear();
	} else {
		MASTER_MODES[armId] = NONE;
	}
	return true;
}

bool
MasterMode2::getStatus(MasterModeStatusMap& status) {
	boost::mutex::scoped_lock(masterMutex);
	FOREACH_ARM_ID(armId) {
		MasterModeStatus arm_status;
		arm_status.mode = MASTER_MODES[armId];
		arm_status.conflicts = MASTER_MODE_CONFLICTS[armId];
		status[armId] = arm_status;
	}
	MASTER_MODE_CONFLICTS.clear();
	return true;
}

const ros::Duration MasterMode2::TIMEOUT(0.5);

void
MasterMode2::checkTimeout() {
	boost::mutex::scoped_lock(masterMutex);
	printf("checking master mode2\n");
	ros::Time now = ros::Time::now();
	Arm::IdList resetIds;
	FOREACH_ARM_ID(armId) {
		if (!MASTER_MODES[armId].isNone()) {
			if (now > LAST_CHECK_TIMES[armId] + TIMEOUT) {
				MASTER_MODES[armId] = NONE;
				MASTER_MODE_CONFLICTS[armId].clear();
				resetIds.push_back(armId);
				RunLevel::setArmActive(armId,false);
			}
		}
	}
}

void
ControlInput::setControlInput(Arm::IdType arm, const std::string& type,ControlInputPtr input) {
	boost::mutex::scoped_lock(inputMutex);
	ControlInput::CONTROL_INPUT[std::pair<Arm::IdType,std::string>(arm,type)] = input;
}

/*
ControlInputPtr
ControlInput::getControlInput() {
	return getControlInput(Controller::getController()->inputType());
}
*/

ControlInputPtr
ControlInput::getControlInput(Arm::IdType arm, const std::string& type) {
	boost::mutex::scoped_lock(inputMutex);
	ControlInputPtr input;
	std::pair<Arm::IdType,std::string> pair_type(arm,type);
	if (type == "old") {
		input = getOldControlInput();
	} else if (arm == Arm::ALL_ARMS) {
		std::map<std::pair<Arm::IdType,std::string>,ControlInputPtr>::iterator itr = CONTROL_INPUT.find(pair_type);
		if (itr != CONTROL_INPUT.end()) {
			input = itr->second;
		} else {
			//FIXME: make a fuss?
		}
	} else {
		input = CONTROL_INPUT[pair_type];
	}
	return input;
}

OldControlInputPtr
ControlInput::getOldControlInput() {
	boost::mutex::scoped_lock(inputMutex);
	OldControlInputPtr ptr;
	if (ROS_UNLIKELY(!OLD_CONTROL_INPUT)) {
		OLD_CONTROL_INPUT.reset(new OldControlInput());
	}
	ptr = OLD_CONTROL_INPUT;
	return ptr;
}

OldControlInputPtr
ControlInput::oldControlInputUpdateBegin() {
	TRACER_ENTER_SCOPE("ControlInput::oldControlInputUpdateBegin()");
	oldInputMutex.lock();
	return getOldControlInput();
}
void
ControlInput::oldControlInputUpdateEnd() {
	TRACER_ENTER_SCOPE("ControlInput::oldControlInputUpdateEnd()");
	oldInputMutex.unlock();
}

ros::Time
MultipleControlInput::timestamp() const {
	ros::Time stamp(0);
	Map::const_iterator itr;
	for (itr=inputs_.begin();itr!=inputs_.end();itr++) {
		ros::Time t =  itr->second->timestamp();
		if (t>stamp) {
			stamp = t;
		}
	}
	return stamp;
}
void
MultipleControlInput::setTimestamp(ros::Time time) {
	Map::iterator itr;
	for (itr=inputs_.begin();itr!=inputs_.end();itr++) {
		itr->second->setTimestamp(time);
	}
}

Arm::IdList
MultipleControlInput::armIds() const {
	Arm::IdSet ids;
	Map::const_iterator itr;
	for (itr=inputs_.begin();itr!=inputs_.end();itr++) {
		Arm::IdList theseIds = itr->second->armIds();
		ids.insert(theseIds.begin(),theseIds.end());
	}
	return Device::sortArmIds(ids);
}

void
MultipleControlInput::setFrom(DeviceConstPtr dev) {
	Map::iterator itr;
	for (itr=inputs_.begin();itr!=inputs_.end();itr++) {
		itr->second->setFrom(dev);
	}
}

MultipleControlInput::ConstMap
MultipleControlInput::inputs() const {
	ConstMap map;
	Map::const_iterator itr;
	for (itr=inputs_.begin();itr!=inputs_.end();itr++) {
		map[itr->first] = ControlInputConstPtr(itr->second);
	}
	return map;
}

bool
MultipleControlInput::hasInput(const std::string& name) const {
	return inputs_.find(name) != inputs_.end();
}
ControlInputPtr
MultipleControlInput::getInput(const std::string& name) {
	if (!hasInput(name)) {
		return ControlInputPtr();
	}
	return inputs_.at(name);
}
ControlInputConstPtr
MultipleControlInput::getInput(const std::string& name) const {
	if (!hasInput(name)) {
		return ControlInputConstPtr();
	}
	return inputs_.at(name);
}

void
MultipleControlInput::setInput(const std::string& name,ControlInputPtr input) {
	inputs_[name] = input;
}
void
MultipleControlInput::removeInput(const std::string& name) {
	inputs_.erase(name);
}
void
MultipleControlInput::clearInputs() {
	inputs_.clear();
}


OldArmInputData::OldArmInputData(const OldArmInputData& other) : //id_(other.id_),
			motorPositions_(new std::vector<float>(other.motorPositions_->begin(),other.motorPositions_->end())),
			motorVelocities_(new std::vector<float>(other.motorVelocities_->begin(),other.motorVelocities_->end())),
			motorTorques_(new std::vector<float>(other.motorTorques_->begin(),other.motorTorques_->end())),
			jointPositions_(new std::vector<float>(other.jointPositions_->begin(),other.jointPositions_->end())),
			jointVelocities_(new std::vector<float>(other.jointVelocities_->begin(),other.jointVelocities_->end())),
			pose_(new btTransform(*(other.pose_))), grasp_(new float(*(other.grasp_))) {

}

//OldArmInputData::OldArmInputData(int id) : id_(id),
//OldArmInputData::OldArmInputData() :
OldArmInputData::OldArmInputData(int id,size_t numMotors,size_t numJoints) :
		motorPositions_(new std::vector<float>(numMotors,0)),
		motorVelocities_(new std::vector<float>(numMotors,0)),
		motorTorques_(new std::vector<float>(numMotors,0)),
		jointPositions_(new std::vector<float>(numJoints,0)),
		jointVelocities_(new std::vector<float>(numJoints,0)),
		pose_(new btTransform(btTransform::getIdentity())), grasp_(new float(0)) {

}


OldControlInput::OldControlInput() : SeparateArmControlInput<OldArmInputData>(Device::armIds()) {
	/*static DevicePtr device;
	FOREACH_ARM_IN_CURRENT_DEVICE(arm,device) {
		arms_.push_back(OldArmInputData(arm->id()));
	}*/
}

void
OldControlInput::setFrom(DeviceConstPtr dev) {
	TRACER_ENTER_SCOPE("OldControlInput::setFrom()");
	FOREACH_ARM_IN_CONST_DEVICE(arm_in,dev) {
		OldArmInputData& arm_curr = armById(arm_in->id());
		for (size_t i=0;i<arm_curr.motorPositions().size();i++) {
			arm_curr.motorPositions()[i]= arm_in->motor(i)->position();
		}
		for (size_t i=0;i<arm_curr.motorVelocities().size();i++) {
			arm_curr.motorVelocities()[i]= arm_in->motor(i)->velocity();
		}
		for (size_t i=0;i<arm_curr.motorTorques().size();i++) {
			arm_curr.motorTorques()[i]= arm_in->motor(i)->torque();
		}
		for (size_t i=0;i<arm_curr.jointPositions().size();i++) {
			arm_curr.jointPositions()[i]= arm_in->joint(i)->position();
		}
		for (size_t i=0;i<arm_curr.jointVelocities().size();i++) {
			arm_curr.jointVelocities()[i]= arm_in->joint(i)->velocity();
		}
		arm_curr.pose() = arm_in->pose();
		arm_curr.grasp() = arm_in->joint(Joint::IdType::GRASP_)->position();
	}
}

Eigen::VectorXf OldControlInput::motorPositionVector() const {
	size_t numEl = 0;
	for (size_t i=0;i<arms_.size();i++) {
		numEl += arms_[i].motorPositions().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = arms_[i].motorPositions().size();
		v.segment(ind,numElInArm) = arms_[i].motorPositionVector();
		ind += numElInArm;
	}
	return v;
}
Eigen::VectorXf OldControlInput::motorVelocityVector() const {
	size_t numEl = 0;
	for (size_t i=0;i<arms_.size();i++) {
		numEl += arms_[i].motorVelocities().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = arms_[i].motorVelocities().size();
		v.segment(ind,numElInArm) = arms_[i].motorVelocityVector();
		ind += numElInArm;
	}
	return v;
}
Eigen::VectorXf OldControlInput::motorTorqueVector() const {
	size_t numEl = 0;
	for (size_t i=0;i<arms_.size();i++) {
		numEl += arms_[i].motorTorques().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = arms_[i].motorTorques().size();
		v.segment(ind,numElInArm) = arms_[i].motorTorqueVector();
		ind += numElInArm;
	}
	return v;
}
Eigen::VectorXf OldControlInput::jointPositionVector() const {
	size_t numEl = 0;
	for (size_t i=0;i<arms_.size();i++) {
		numEl += arms_[i].jointPositions().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = arms_[i].jointPositions().size();
		v.segment(ind,numElInArm) = arms_[i].jointPositionVector();
		ind += numElInArm;
	}
	return v;
}
Eigen::VectorXf OldControlInput::jointVelocityVector() const {
	size_t numEl = 0;
	for (size_t i=0;i<arms_.size();i++) {
		numEl += arms_[i].jointVelocities().size();
	}
	Eigen::VectorXf v(numEl);
	size_t ind = 0;
	for (size_t i=0;i<arms_.size();i++) {
		size_t numElInArm = arms_[i].jointVelocities().size();
		v.segment(ind,numElInArm) = arms_[i].jointVelocityVector();
		ind += numElInArm;
	}
	return v;
}

float&
OldControlInput::motorPositionByOldType(int type) {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].motorPosition(joint_ind);
}

const float&
OldControlInput::motorPositionByOldType(int type) const {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].motorPosition(joint_ind);
}

float&
OldControlInput::motorVelocityByOldType(int type) {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].motorVelocity(joint_ind);
}

const float&
OldControlInput::motorVelocityByOldType(int type) const {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].motorVelocity(joint_ind);
}

float&
OldControlInput::motorTorqueByOldType(int type) {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].motorTorque(joint_ind);
}

const float&
OldControlInput::motorTorqueByOldType(int type) const {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].motorTorque(joint_ind);
}

float&
OldControlInput::jointPositionByOldType(int type) {
	int arm_id;
		int joint_ind;
		getArmAndJointIndices(type,arm_id,joint_ind);

		if (joint_ind == 3) {
			//return 0;
		}
		if (joint_ind > 3) {
			joint_ind -= 1;
		}
		return arms_[arm_id].jointPosition(joint_ind);
}

const float&
OldControlInput::jointPositionByOldType(int type) const {
	int arm_id;
		int joint_ind;
		getArmAndJointIndices(type,arm_id,joint_ind);

		if (joint_ind == 3) {
			//return 0;
		}
		if (joint_ind > 3) {
			joint_ind -= 1;
		}
		return arms_[arm_id].jointPosition(joint_ind);
}

float&
OldControlInput::jointVelocityByOldType(int type) {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].jointVelocity(joint_ind);
}

const float&
OldControlInput::jointVelocityByOldType(int type) const {
	int arm_id;
	int joint_ind;
	getArmAndJointIndices(type,arm_id,joint_ind);

	if (joint_ind == 3) {
		//return 0;
	}
	if (joint_ind > 3) {
		joint_ind -= 1;
	}
	return arms_[arm_id].jointVelocity(joint_ind);
}

//OldArmInputData&
//OldControlInput::armById(int id) {
//	for (size_t i=0;i<arms_.size();i++) {
//		if (arms_.at(i).id() == id) {
//			return arms_.at(i);
//		}
//	}
//}
//
//const OldArmInputData&
//OldControlInput::armById(int id) const {
//	for (size_t i=0;i<arms_.size();i++) {
//		if (arms_.at(i).id() == id) {
//			return arms_.at(i);
//		}
//	}
//}
