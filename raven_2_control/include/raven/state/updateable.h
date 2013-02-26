/*
 * updateable.h
 *
 *  Created on: Oct 1, 2012
 *      Author: benk
 */

#ifndef UPDATEABLE_H_
#define UPDATEABLE_H_

#include <ros/ros.h>
#include <vector>
#include <list>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/foreach.hpp>
#include <stdio.h>
#include <stdexcept>

#include "log.h"

class Updateable;

typedef boost::shared_ptr<Updateable> UpdateablePtr;
typedef boost::weak_ptr<Updateable> UpdateableWeakPtr;
typedef std::vector<UpdateableWeakPtr> UpdateableList;

class Updateable {
public:
	Updateable(bool useInternalUpdate,bool useProcessNotification) : timestamp_(0),
			useInternalUpdate_(useInternalUpdate), useProcessNotification_(useProcessNotification),
			immediateUpdate_(true), holdUpdates_(false), heldUpdateTimestamp_(0) {}
	Updateable(const Updateable& other) : timestamp_(other.timestamp_),
			useInternalUpdate_(other.useInternalUpdate_), useProcessNotification_(other.useProcessNotification_),
			immediateUpdate_(other.immediateUpdate_), holdUpdates_(other.holdUpdates_), heldUpdateTimestamp_(other.heldUpdateTimestamp_) {
		//TODO: if holding updates, throw exception?
	}

	inline UpdateablePtr parent() const { return parent_; }

	virtual ros::Time timestamp() const { return timestamp_; }
	inline ros::Time getUpdateableTimestamp() const { return timestamp_; }

	inline bool update() {
		TRACER_ENTER_SCOPE_OF(this,"update()");
		bool ret = true;
		if (ROS_UNLIKELY(useInternalUpdate_)) {
			ret = internalUpdate();
		}
		if (heldUpdateTimestamp_ != ros::Time(0)) {
			TRACER_PRINT("held update timestamp");
			setUpdateableTimestamp(heldUpdateTimestamp_);
		}
		if (parent_) {
			parent_->notify(this);
		}
		return ret;
	}

	/*virtual*/ inline void updateTimestamp(ros::Time t = ros::Time::now()) {
		timestamp_ = t;
		if (isImmediateUpdate()) {
			update();
		}
	}

	virtual ~Updateable() {}

	inline void setUpdateableParent(UpdateablePtr obj) { parent_ = obj; }

	inline bool isImmediateUpdate() const  { return immediateUpdate_; }
	inline void setImmediateUpdate(bool immediate) { immediateUpdate_ = immediate; }

	inline void holdUpdateBegin() {
		TRACER_ENTER_SCOPE_OF(this,"holdUpdateBegin()");
		holdUpdates_ = true;
		heldUpdateTimestamp_ = ros::Time(0);
	}
	inline bool holdUpdateEnd() {
		TRACER_ENTER_SCOPE_OF(this,"holdUpdateEnd()");
		bool changed = heldUpdateTimestamp_ != ros::Time(0);
		if (changed) {
			update();
		}
		holdUpdates_ = false;
		heldUpdateTimestamp_ = ros::Time(0);
		return changed;
	}
protected:
	ros::Time timestamp_;
	UpdateablePtr parent_;
	bool useInternalUpdate_;
	bool useProcessNotification_;
	bool immediateUpdate_;

	bool holdUpdates_;
	ros::Time heldUpdateTimestamp_;
	inline void setUpdateableTimestamp(ros::Time stamp) { timestamp_ = stamp; }

	virtual bool internalUpdate() {
		throw std::logic_error("Updateable::internalUpdate() not overridden!");
	}

	/*virtual*/ inline void notify(Updateable* sender) {
		TRACER_ENTER_SCOPE_OF(this,"notify() from %s@%p",typeid(*sender).name(),sender);
		if (sender->getUpdateableTimestamp() <= getUpdateableTimestamp()) {
			return;
		}
		if (holdUpdates_) {
			TRACER_PRINT("Holding updates");
			heldUpdateTimestamp_ = sender->getUpdateableTimestamp();
			return;
		} else {
			bool doUpdate = true;
			if (ROS_UNLIKELY(useProcessNotification_)) {
				doUpdate = processNotification(sender);
			}
			if (doUpdate) {
				setUpdateableTimestamp(sender->getUpdateableTimestamp());
				update();
			}
		}
	}

	virtual bool processNotification(Updateable* sender) {
		throw std::logic_error("Updateable::processNotification() not overridden!");
	};
};

#endif /* UPDATEABLE_H_ */
