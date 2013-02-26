/*
 * control_input.h
 *
 *  Created on: Oct 9, 2012
 *      Author: benk
 */

#ifndef CONTROL_INPUT_H_
#define CONTROL_INPUT_H_

#include <raven/state/device.h>
#include <sstream>
#include <stdexcept>
#include <map>

POINTER_TYPES(ControlInput)
POINTER_TYPES(OldControlInput)

class ControlInput {
	friend class Controller;
private:
	static std::map<std::string,ControlInputPtr> CONTROL_INPUT;
	static OldControlInputPtr OLD_CONTROL_INPUT;
	static ControlInputPtr getControlInput(const std::string& type);
protected:
	ros::Time timestamp_;
public:
	virtual ~ControlInput() {}

	virtual ros::Time timestamp() const { return timestamp_; }
	virtual void setTimestamp(ros::Time time) { timestamp_ = time; }
	void updateTimestamp() { setTimestamp(ros::Time::now()); }

	virtual void setFrom(DevicePtr dev) = 0;

	static void setControlInput(const std::string& type,ControlInputPtr input);

	template<class C>
	static boost::shared_ptr<C> getControlInput(const std::string& type) {
		return boost::dynamic_pointer_cast<C,ControlInput>(getControlInput(type));
	}

	static OldControlInputPtr getOldControlInput();
	static OldControlInputPtr oldControlInputUpdateBegin();
	static void oldControlInputUpdateEnd();
};

class MultipleControlInput : public ControlInput {
private:
	std::map<std::string,ControlInputPtr> inputs_;
public:
	virtual ros::Time timestamp() const;
	virtual void setTimestamp(ros::Time time);

	virtual void setFrom(DevicePtr dev);

	std::map<std::string,ControlInputPtr> inputs() const { return inputs_; }

	void setInput(const std::string& name,ControlInputPtr input);
	void removeInput(const std::string& name);
	void clearInputs();

	bool hasInput(const std::string& name) const;
	ControlInputPtr getInput(const std::string& name);
	ControlInputConstPtr getInput(const std::string& name) const;

	template<class T>
	boost::shared_ptr<T> getInput(const std::string& name) {
		ControlInputPtr p = getInput(name);
		boost::shared_ptr<T> out = boost::dynamic_pointer_cast<T>(p);
		return out;
	}

	template<class T>
	boost::shared_ptr<const T> getInput(const std::string& name) const {
		ControlInputConstPtr p = getInput(name);
		boost::shared_ptr<const T> out = boost::dynamic_pointer_cast<const T>(p);
		return out;
	}

	template<class T>
	bool getInput(const std::string& name,boost::shared_ptr<T>& input) {
		ControlInputPtr p = getInput(name);
		input = boost::dynamic_pointer_cast<T>(p);
		return input.get();
	}

	template<class T>
	bool getInput(const std::string& name,boost::shared_ptr<const T>& input) const {
		ControlInputConstPtr p = getInput(name);
		input = boost::dynamic_pointer_cast<T>(p);
		return input.get();
	}
};
POINTER_TYPES(MultipleControlInput)

template<typename T>
class SeparateArmControlInput : public ControlInput {
protected:
	std::vector<Arm::IdType> armIds_;
	std::vector<T> arms_;
public:
	SeparateArmControlInput() {
		static DevicePtr device;
		FOREACH_ARM_IN_CURRENT_DEVICE(arm,device) {
			Arm::IdType id = arm->id();
			armIds_.push_back(id);
			arms_.push_back(T(id,arm->motors().size(),arm->joints().size()));
		}
	}

	T& arm(size_t i) { return arms_.at(i); }
	const T& arm(size_t i) const { return arms_.at(i); }

	T& armById(Arm::IdType id) {
		for (size_t i=0;i<armIds_.size();i++) {
			if (id == armIds_.at(i)) {
				return arms_.at(i);
			}
		}
		std::stringstream ss;
		ss << "Arm " << id << " not found!";
		throw std::out_of_range(ss.str());
	}
	const T& armById(Arm::IdType id) const  {
		for (size_t i=0;i<armIds_.size();i++) {
			if (id == armIds_.at(i)) {
				return arms_.at(i);
			}
		}
		std::stringstream ss;
		ss << "Arm " << id << " not found!";
		throw std::out_of_range(ss.str());
	}
};

#include <memory>
#include <LinearMath/btTransform.h>

#define ARM_INPUT_DATA_ENSURE_SIZE(field) do { if (field->size() <= i) { const_cast<OldArmInputData*>(this)->field->resize(i+1); } } while(false)

class OldArmInputData {
private:
	std::auto_ptr<std::vector<float> > motorPositions_;
	std::auto_ptr<std::vector<float> > motorVelocities_;
	std::auto_ptr<std::vector<float> > motorTorques_;
	std::auto_ptr<std::vector<float> > jointPositions_;
	std::auto_ptr<std::vector<float> > jointVelocities_;

	std::auto_ptr<btTransform> pose_;
	std::auto_ptr<float> grasp_;
public:
	OldArmInputData(const OldArmInputData& other);
	OldArmInputData(int id,size_t numMotors,size_t numJoints);
	//OldArmInputData();

	std::vector<float>& motorPositions() { return *motorPositions_; }
	const std::vector<float>& motorPositions() const { return *motorPositions_; }
	Eigen::Map<const Eigen::VectorXf> motorPositionVector() const { return Eigen::Map<const Eigen::VectorXf>(&motorPositions_->at(0),motorPositions_->size()); }
	Eigen::Map<Eigen::VectorXf> motorPositionVector() { return Eigen::Map<Eigen::VectorXf>(&motorPositions_->at(0),motorPositions_->size()); }

	float& motorPosition(size_t i) { do { if (motorPositions_->size() <= i) { const_cast<OldArmInputData*>(this)->motorPositions_->resize(i+1); } } while(false); return motorPositions_->at(i); }
	const float& motorPosition(size_t i) const { ARM_INPUT_DATA_ENSURE_SIZE(motorPositions_); return motorPositions_->at(i); }

	std::vector<float>& motorVelocities() { return *motorVelocities_; }
	const std::vector<float>& motorVelocities() const { return *motorVelocities_; }
	Eigen::Map<const Eigen::VectorXf> motorVelocityVector() const { return Eigen::Map<const Eigen::VectorXf>(&motorVelocities_->at(0),motorVelocities_->size()); }
	Eigen::Map<Eigen::VectorXf> motorVelocityVector() { return Eigen::Map<Eigen::VectorXf>(&motorVelocities_->at(0),motorVelocities_->size()); }

	float& motorVelocity(size_t i) { ARM_INPUT_DATA_ENSURE_SIZE(motorVelocities_); return motorVelocities_->at(i); }
	const float& motorVelocity(size_t i) const { ARM_INPUT_DATA_ENSURE_SIZE(motorVelocities_); return motorVelocities_->at(i); }

	std::vector<float>& motorTorques() { return *motorTorques_; }
	const std::vector<float>& motorTorques() const { return *motorTorques_; }
	Eigen::Map<const Eigen::VectorXf> motorTorqueVector() const { return Eigen::Map<const Eigen::VectorXf>(&motorTorques_->at(0),motorTorques_->size()); }
	Eigen::Map<Eigen::VectorXf> motorTorqueVector() { return Eigen::Map<Eigen::VectorXf>(&motorTorques_->at(0),motorTorques_->size()); }

	float& motorTorque(size_t i) { ARM_INPUT_DATA_ENSURE_SIZE(motorTorques_); return motorTorques_->at(i); }
	const float& motorTorque(size_t i) const { ARM_INPUT_DATA_ENSURE_SIZE(motorTorques_); return motorTorques_->at(i); }

	std::vector<float> jointPositions() { return *jointPositions_; }
	const std::vector<float> jointPositions() const { return *jointPositions_; }
	Eigen::Map<const Eigen::VectorXf> jointPositionVector() const { return Eigen::Map<const Eigen::VectorXf>(&jointPositions_->at(0),jointPositions_->size()); }
	Eigen::Map<Eigen::VectorXf> jointPositionVector() { return Eigen::Map<Eigen::VectorXf>(&jointPositions_->at(0),jointPositions_->size()); }

	float& jointPosition(size_t i) { ARM_INPUT_DATA_ENSURE_SIZE(jointPositions_); return jointPositions_->at(i); }
	const float& jointPosition(size_t i) const { ARM_INPUT_DATA_ENSURE_SIZE(jointPositions_); return jointPositions_->at(i); }

	std::vector<float>& jointVelocities() { return *jointVelocities_; }
	const std::vector<float>& jointVelocities() const { return *jointVelocities_; }
	Eigen::Map<const Eigen::VectorXf> jointVelocityVector() const { return Eigen::Map<const Eigen::VectorXf>(&jointVelocities_->at(0),jointVelocities_->size()); }
	Eigen::Map<Eigen::VectorXf> jointVelocityVector() { return Eigen::Map<Eigen::VectorXf>(&jointVelocities_->at(0),jointVelocities_->size()); }

	float& jointVelocity(size_t i) { ARM_INPUT_DATA_ENSURE_SIZE(jointVelocities_); return jointVelocities_->at(i); }
	const float& jointVelocity(size_t i) const { ARM_INPUT_DATA_ENSURE_SIZE(jointVelocities_); return jointVelocities_->at(i); }

	btTransform& pose() { return *pose_; }
	const btTransform& pose() const { return *pose_; }

	float& grasp() { return *grasp_; }
	const float& grasp() const { return *grasp_; }
};

class OldControlInput : public SeparateArmControlInput<OldArmInputData> {
public:
	OldControlInput();

	virtual void setFrom(DevicePtr dev);

	Eigen::VectorXf motorPositionVector() const;
	Eigen::VectorXf motorVelocityVector() const;
	Eigen::VectorXf motorTorqueVector() const;
	Eigen::VectorXf jointPositionVector() const;
	Eigen::VectorXf jointVelocityVector() const;

	float& motorPositionByOldType(int type);
	const float& motorPositionByOldType(int type) const;

	float& motorVelocityByOldType(int type);
	const float& motorVelocityByOldType(int type) const;

	float& motorTorqueByOldType(int type);
	const float& motorTorqueByOldType(int type) const;

	float& jointPositionByOldType(int type);
	const float& jointPositionByOldType(int type) const;

	float& jointVelocityByOldType(int type);
	const float& jointVelocityByOldType(int type) const;
};


#endif /* CONTROL_INPUT_H_ */
