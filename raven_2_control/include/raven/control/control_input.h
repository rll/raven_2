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
#include <algorithm>
#include <set>

POINTER_TYPES(ControlInput)
POINTER_TYPES(OldControlInput)

class MasterModeStatus;

typedef std::map<Arm::IdType,MasterModeStatus> MasterModeStatusMap;

class MasterMode2 {
private:
	std::string str_;

	static std::map<Arm::IdType,MasterMode2> MASTER_MODES;
	static std::map<Arm::IdType,std::set<MasterMode2> > MASTER_MODE_CONFLICTS;
	static std::map<Arm::IdType,ros::Time> LAST_CHECK_TIMES;
public:
	MasterMode2() : str_("") {}
	MasterMode2(const std::string& str) : str_(str) {}
	MasterMode2(const char*& str) : str_(str) {}

	bool isNone() const { return str_.empty(); }

	std::string str() const { return str_; }
	bool operator==(const MasterMode2& other) const { return str_ == other.str_; }
	bool operator<(const MasterMode2& other) const { return str_ < other.str_; }
	std::ostream& operator<<(std::ostream& o) const { o << str_; return o; }

	static const MasterMode2 NONE;

	static MasterMode2 get(Arm::IdType armId);

	static bool check(Arm::IdType armId, const MasterMode2& mode);

	static bool reset(Arm::IdType armId);

	static bool getStatus(MasterModeStatusMap& status);

	static const ros::Duration TIMEOUT;
	static void checkTimeout();
};

struct MasterModeStatus {
	MasterMode2 mode;
	std::set<MasterMode2> conflicts;
};

class ControlInput {
	friend class Controller;
private:
	static std::map<std::pair<Arm::IdType,std::string>,ControlInputPtr> CONTROL_INPUT;
	static OldControlInputPtr OLD_CONTROL_INPUT;
	static ControlInputPtr getControlInput(Arm::IdType armId, const std::string& type);
protected:
	ros::Time timestamp_;
public:
	virtual ~ControlInput() {}

	virtual ros::Time timestamp() const { return timestamp_; }
	virtual void setTimestamp(ros::Time time) { timestamp_ = time; }
	void updateTimestamp() { setTimestamp(ros::Time::now()); }

	virtual Arm::IdList armIds() const=0;
	size_t numArms() const { return armIds().size(); }
	bool hasArmId(Arm::IdType id) const { return std::find(armIds().begin(),armIds().end(),id) != armIds().end(); }

	virtual void setFrom(DeviceConstPtr dev) = 0;

	static void setControlInput(Arm::IdType armId, const std::string& type, ControlInputPtr input);

	template<class C>
	static boost::shared_ptr<C> getControlInput(Arm::IdType armId, const std::string& type) {
		return boost::dynamic_pointer_cast<C,ControlInput>(getControlInput(armId, type));
	}

	static OldControlInputPtr getOldControlInput();
	static OldControlInputPtr oldControlInputUpdateBegin();
	static void oldControlInputUpdateEnd();

};

class MultipleControlInput : public ControlInput {
public:
	typedef std::map<std::string,ControlInputPtr> Map;
	typedef std::map<std::string,ControlInputConstPtr> ConstMap;
private:
	Map inputs_;
public:
	virtual ros::Time timestamp() const;
	virtual void setTimestamp(ros::Time time);

	virtual Arm::IdList armIds() const;

	virtual void setFrom(DeviceConstPtr dev);

	Map inputs() { return inputs_; }
	ConstMap inputs() const;

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

template<typename T1,typename T2>
class DualControlInput : public ControlInput {
public:
	typedef boost::shared_ptr<T1> FirstPtr;
	typedef boost::shared_ptr<const T1> FirstConstPtr;
	typedef boost::shared_ptr<T2> SecondPtr;
	typedef boost::shared_ptr<const T2> SecondConstPtr;

	typedef std::pair<FirstPtr,SecondPtr> Pair;
	typedef std::pair<FirstConstPtr,SecondConstPtr> ConstPair;

	typedef boost::shared_ptr<DualControlInput> Ptr;
	typedef boost::shared_ptr<const DualControlInput> ConstPtr;
protected:
	FirstPtr first_;
	SecondPtr second_;
public:
	virtual ros::Time timestamp() const {
		ros::Time stamp = first_->timestamp();
		if (second_->timestamp() > stamp) {
			stamp = second_->timestamp();
		}
		return stamp;
	}
	virtual void setTimestamp(ros::Time time) {
		first_->setTimestamp(time);
		second_->setTimestamp(time);
	}

	virtual Arm::IdList armIds() const {
		Arm::IdSet ids = Arm::idSet(first_->armIds());
		Arm::IdList secondIds = second_->armIds();
		ids.insert(secondIds.begin(),secondIds.end());
		return Device::sortArmIds(ids);
	}

	virtual void setFrom(DeviceConstPtr dev) {
		first_->setFrom(dev);
		second_->setFrom(dev);
	}

	Pair inputs() { return Pair(first_,second_); }
	ConstPair inputs() const { return ConstPair(first_,second_); }

	FirstPtr first() { return first_; }
	FirstConstPtr first() const { return first_; }

	SecondPtr second() { return second_; }
	SecondConstPtr second() const { return second_; }

	void setFirst(FirstPtr newFirst) { first_ = newFirst; }
	void setSecond(SecondPtr newSecond) { second_ = newSecond; }
};

template<typename T>
class SeparateArmControlInput : public ControlInput {
private:
	void init() {
		for (size_t i=0;i<armIds_.size();i++) {
			Arm::IdType id = armIds_.at(i);
			arms_.push_back(T(id,Device::numMotorsOnArmById(id),Device::numJointsOnArmById(id)));
		}
	}
protected:
	Arm::IdList armIds_;
	std::vector<T> arms_;

	SeparateArmControlInput(const Arm::IdList& armIds) {
		armIds_ = armIds;
		init();
	}
public:


	typedef boost::shared_ptr<SeparateArmControlInput<T> > Ptr;

	virtual Arm::IdList armIds() const { return armIds_; }
	const Arm::IdList& ids() const { return armIds_; }
	bool hasId(Arm::IdType id) const { return hasArmId(id); }


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

template<typename SeparateArmControlInputSubClass>
static boost::shared_ptr<SeparateArmControlInputSubClass> createSeparateArmControlInput() {
	boost::shared_ptr<SeparateArmControlInputSubClass> newPtr(new SeparateArmControlInputSubClass(Device::armIds()));
	return newPtr;
}

template<typename SeparateArmControlInputSubClass>
static boost::shared_ptr<SeparateArmControlInputSubClass> createSeparateArmControlInput(Arm::IdType armId) {
	Arm::IdList ids;
	ids.push_back(armId);
	boost::shared_ptr<SeparateArmControlInputSubClass> newPtr(new SeparateArmControlInputSubClass(ids));
	return newPtr;
}

template<typename SeparateArmControlInputSubClass>
static boost::shared_ptr<SeparateArmControlInputSubClass> createSeparateArmControlInput(const Arm::IdList& armIds) {
	boost::shared_ptr<SeparateArmControlInputSubClass> newPtr(new SeparateArmControlInputSubClass(armIds));
	return newPtr;
}

template<typename T>
class SingleArmControlInput {
protected:
	SingleArmControlInput() {}
public:
	virtual ~SingleArmControlInput() {}

	virtual Arm::IdType id() const=0;

	virtual T& data()=0;
	virtual const T& data() const=0;

	#define SINGLE_ARM_CONTROL_INPUT_METHODS(TypeName) \
		virtual Arm::IdType id() const { return this->ids().at(0); }\
		virtual TypeName& data() { return this->arm(0); } \
		virtual const TypeName& data() const { return this->arm(0); }
};
template<typename SingleArmControlInputSubClass>
static boost::shared_ptr<SingleArmControlInputSubClass> createSingleArmControlInput(Arm::IdType armId) {
	boost::shared_ptr<SingleArmControlInputSubClass> newPtr(new SingleArmControlInputSubClass(armId));
	return newPtr;
}

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

	virtual void setFrom(DeviceConstPtr dev);

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
