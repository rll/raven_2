/*
 * controller.h
 *
 *  Created on: Oct 2, 2012
 *      Author: benk
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <raven/state/device.h>

#include <raven/control/control_input.h>

#include <raven/util/pointers.h>

#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>
#include <raven/util/history.h>

#include <string>
#include <vector>
#include <map>
#include <utility>

POINTER_TYPES(ControllerState)

class ControllerState {
public:
	ControlInputConstPtr input;
	const DeviceConstPtr device;
	int returnCode;
public:
	ControllerState(DevicePtr dev) : device(dev), returnCode(0) {}
	virtual ~ControllerState() {}

	virtual ControllerStatePtr clone(DevicePtr dev) const=0;
};

POINTER_TYPES(Controller)

class Controller {
private:
	static std::string CURRENT_CONTROLLER;
	static std::map<std::string,ControllerPtr> CONTROLLERS;
	static DevicePtr CONTROL_OUTPUT;
	static std::map<std::string,History<Device>::Type> CONTROL_OUTPUT_HISTORY;
	static int internalExecuteControl(ControllerPtr controller, const std::string& type);
	static void setControlOutput(DevicePtr dev);

	boost::circular_buffer<ControllerStatePtr> history_;
	void saveState(ControllerStatePtr state);
	virtual ControllerStatePtr internalApplyControl(DevicePtr device)=0;

	ControlInputPtr input_;

	boost::shared_ptr<ros::Rate> rate_;
public:

	static void registerController(const std::string& type,ControllerPtr controller);
	static bool setController(const std::string& type);
	static ControllerPtr getController();
	static ControllerPtr getControllerAndType(std::string& type);

	static int executeInProcessControl();
	static int executeOutOfProcessControl();
	static DevicePtr getControlOutput();

	virtual ~Controller() {}

	virtual std::string name() const=0;
	virtual std::string type() const=0;

	virtual bool inProcess() const { return true; }
	ros::Rate& rate() const { return *rate_; }

	virtual void setInput(ControlInputPtr input) { input_ = input; }
	virtual void clearInput() { input_.reset(); }
	virtual ControlInputPtr getInput() const { return input_; }
	template<class T>
	boost::shared_ptr<T> getInput() const {
		ControlInputPtr p = getInput();
		boost::shared_ptr<T> out = boost::dynamic_pointer_cast<T>(p);
		return out;
	}

	template<class T>
	bool getInput(boost::shared_ptr<T>& input) const {
		ControlInputPtr p = getInput();
		input = boost::dynamic_pointer_cast<T>(p);
		return input.get();
	}



	virtual int applyControl(DevicePtr device);

	std::vector<ControllerStatePtr> stateHistory(int numSteps=-1) const;
	ControllerStatePtr lastState() const;

	template<typename T>
	std::vector<boost::shared_ptr<T> > stateHistory(int numSteps=-1) const {
		if (numSteps < 0 || numSteps > (int)history_.size()) {
			numSteps = history_.size();
		}
		std::vector<boost::shared_ptr<T> > hist(numSteps);
		for (int i=0;i<numSteps;i++) {
			boost::shared_ptr<T> ptr = boost::dynamic_pointer_cast<T>(history_[i]);
			if (!ptr) {
				throw std::runtime_error("History cast failed!");
			}
			hist.push_back(ptr);
		}
		return hist;
	}
	template<typename T>
	boost::shared_ptr<T> lastState() const {
		return boost::dynamic_pointer_cast<T>(lastState());
	}

	virtual ControllerPtr clone() const { return ControllerPtr(); };

protected:
	Controller(size_t history_size=0);

	void setRate(double hz) {
		rate_.reset(new ros::Rate(hz));
	}

	template<typename T>
	T getParameter(const std::string& paramName,T defaultValue=T()) {
		std::vector<ros::NodeHandle> nh_list;
		nh_list.push_back(ros::NodeHandle(name()));
		nh_list.push_back(ros::NodeHandle(type()));
		nh_list.push_back(ros::NodeHandle(""));
		T value = defaultValue;
		std::vector<ros::NodeHandle>::iterator nh;
		for (nh=nh_list.begin();nh!=nh_list.end();nh++) {
			if (nh->hasParam(paramName)) {
				XmlRpc::XmlRpcValue val;
				nh->getParam(paramName,val);
				value = (T)val;
				break;
			}
		}
		return value;
	}

	template<typename T>
	std::vector<T> getParameterVector(const std::string& paramName) {
		std::vector<ros::NodeHandle> nh_list;
		nh_list.push_back(ros::NodeHandle(name()));
		nh_list.push_back(ros::NodeHandle(type()));
		nh_list.push_back(ros::NodeHandle(""));
		std::vector<T> values;
		std::vector<ros::NodeHandle>::iterator nh;
		for (nh=nh_list.begin();nh!=nh_list.end();nh++) {
			//printf("checking %s\n",nh->getNamespace().c_str());
			if (nh->hasParam(paramName)) {
				//printf("has param\n");
				XmlRpc::XmlRpcValue val;
				nh->getParam(paramName,val);
				//printf("value: %s %i\n",val.toXml().c_str(),val.size());
				for (int i=0;i<val.size();i++) {
					values.push_back((T)val[i]);
				}
				break;
			}
		}
		return values;
	}

	template<typename T>
	std::vector<boost::shared_ptr<T> > getHistory(int numSteps = -1) const {
		if (numSteps < 0 || numSteps > (int)history_.size()) {
			numSteps = history_.size();
		}
		std::vector<ControllerStatePtr> hist(numSteps);
		for (int i=0;i<numSteps;i++) {
			hist.push_back(boost::dynamic_pointer_cast<T>(history_[i]));
		}
		return hist;
	}

	template<typename T>
	boost::shared_ptr<T> getHistory(size_t i) const {
		ControllerStatePtr state = history_.at(i);
		return boost::dynamic_pointer_cast<T>(state);
	}

	template<typename T>
	boost::shared_ptr<T> getLastState() const {
		if (history_.empty()) {
			return boost::shared_ptr<T>();
		} else {
			ControllerStatePtr state = history_.at(0);
			return boost::dynamic_pointer_cast<T>(state);
		}
	}

	template<typename T>
	boost::shared_ptr<T> cloneLastState(DevicePtr dev) const {
		if (history_.empty()) {
			return boost::shared_ptr<T>();
		} else {
			ControllerStatePtr state = history_.at(0);
			return boost::dynamic_pointer_cast<T>(state->clone(dev));
		}
	}
};

#endif /* CONTROLLER_H_ */
