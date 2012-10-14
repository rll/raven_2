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

class Updateable;

typedef boost::shared_ptr<Updateable> UpdateablePtr;
typedef boost::weak_ptr<Updateable> UpdateableWeakPtr;
typedef std::vector<UpdateableWeakPtr> UpdateableList;
//typedef std::list<UpdateableWeakPtr> UpdateableList;

class Updateable {
public:
	Updateable() : immediateUpdate_(true), timestamp_(0), /*notifyList_(),*/ holdUpdates_(false), heldUpdateTimestamp_(0) {}
	Updateable(const Updateable& other) : immediateUpdate_(other.immediateUpdate_), timestamp_(other.timestamp_),
			holdUpdates_(other.holdUpdates_), heldUpdateTimestamp_(other.heldUpdateTimestamp_) {}

	UpdateablePtr parent() const { return parent_; }

	virtual ros::Time timestamp() const { return timestamp_; }
	ros::Time getUpdateableTimestamp() const { return timestamp_; }

	virtual bool update() {
		if (heldUpdateTimestamp_ != ros::Time(0)) {
			setUpdateableTimestamp(heldUpdateTimestamp_);
		}
		if (parent_) {
			parent_->notify(this);
		}
		return true;
	}

	virtual void updateTimestamp(ros::Time t = ros::Time::now()) {
		timestamp_ = t;
		if (isImmediateUpdate()) {
			update();
		}
	}

	virtual ~Updateable() {}

	//virtual void addToNotifyList(UpdateablePtr obj) { notifyList_.push_back(UpdateableWeakPtr(obj)); if (notifyList_.size() >= 2) { printf("big\n"); } }
	void setUpdateableParent(UpdateablePtr obj) { parent_ = obj; }

	bool isImmediateUpdate() const  { return immediateUpdate_; }
	void setImmediateUpdate(bool immediate) { immediateUpdate_ = immediate; }

	void holdUpdateBegin() {
		holdUpdates_ = true;
		heldUpdateTimestamp_ = ros::Time(0);
	}
	bool holdUpdateEnd() {
		bool changed = heldUpdateTimestamp_ != ros::Time(0);
		if (changed) {
			update();
		}
		holdUpdates_ = false;
		heldUpdateTimestamp_ = ros::Time(0);
		return changed;
	}
protected:
	bool immediateUpdate_;
	ros::Time timestamp_;
	UpdateablePtr parent_;

	bool holdUpdates_;
	ros::Time heldUpdateTimestamp_;

	void setUpdateableTimestamp(ros::Time stamp) { timestamp_ = stamp; }

	virtual void notify(Updateable* sender) {
		if (sender->getUpdateableTimestamp() <= getUpdateableTimestamp()) {
			return;
		}
		if (holdUpdates_) {
			heldUpdateTimestamp_ = sender->getUpdateableTimestamp();
			return;
		} else {
			bool doUpdate = processNotification(sender);
			if (doUpdate) {
				setUpdateableTimestamp(sender->getUpdateableTimestamp());
				update();
			}
		}
	}

	virtual bool processNotification(Updateable* sender) = 0;
};

#endif /* UPDATEABLE_H_ */
