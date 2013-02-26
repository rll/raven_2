/*
 * lpf.h
 *
 *  Created on: Oct 4, 2012
 *      Author: benk
 */

#ifndef LPF_H_
#define LPF_H_

#include <vector>
#include <boost/circular_buffer.hpp>
#include <Eigen/Core>

#include <raven/state/arm.h>
#include <raven/state/dof.h>
#include <raven/util/history.h>

#define ORDER 3

template<int order>
class LowPassMotorFilter : public MotorFilter {
private:
	Arm::Type armType_;
	int order_;

	Eigen::Matrix<float,order+1,1> A_;
	Eigen::Matrix<float,order+1,1> B_;

	std::vector<History<Motor>::Type> history_;
	std::vector<History<Motor>::Type> filteredHistory_;

	ros::Time lastCallTime_;
protected:
	virtual void internalApplyUpdate();
	virtual void internalCloneInto(MotorFilterPtr& other, const MotorList& newMotors) const;
public:
	LowPassMotorFilter(const MotorList& motors,Arm::Type armType);
	virtual ~LowPassMotorFilter();

	virtual void reset();

	virtual MotorList history(size_t i) const;
	virtual MotorList filteredHistory(size_t i) const;

	virtual std::string str() const;

	virtual MotorFilterPtr clone(const MotorList& newMotors) const;
};


#endif /* LPF_H_ */
