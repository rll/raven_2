/*
 * kinematics.h
 *
 *  Created on: Oct 2, 2012
 *      Author: benk
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <ros/ros.h>

#include <raven/state/dof.h>

#include <LinearMath/btTransform.h>

#include <Eigen/Core>

#include <boost/shared_ptr.hpp>
#include <raven/util/pointers.h>

#include <map>
#include <limits>

class Arm;

class InverseKinematicsOptions {
	friend class KinematicSolver;
private:
	bool checkJointLimits_;
	bool truncateJointsAtLimits_;

	bool truncateJointDifferences_;
	std::map<Joint::Type,float> maxJointDifferences_;
public:
	InverseKinematicsOptions() : checkJointLimits_(true), truncateJointsAtLimits_(true), truncateJointDifferences_(false) {}

	bool checkJointLimits() const;
	void setCheckJointLimits(bool on);

	bool truncateJointsAtLimits() const;
	void setTruncateJointsAtLimits(bool on);

	bool truncateJointDifferences() const;
	void setTruncateJointDifferences(bool on);

	std::map<Joint::Type,float> maxJointDifferences() const;
	float getMaxJointDifference(Joint::Type type) const;

	void setMaxJointDifferences(const std::map<Joint::Type,float>& diffs);
	void setMaxJointDifference(Joint::Type type, float diff);

	void clearMaxJointDifference(Joint::Type type);
	void clearMaxJointDifferences();
};

class InverseKinematicsReport {
	friend class KinematicSolver;
private:
	bool success_;
public:
	InverseKinematicsReport();
	virtual ~InverseKinematicsReport() {}

	bool success() const { return success_; }
};
POINTER_TYPES(InverseKinematicsReport)

class KinematicSolver;
typedef boost::shared_ptr<KinematicSolver> KinematicSolverPtr;

class KinematicSolver {
	friend class Arm;
private:
	Arm* arm_;

	InverseKinematicsOptions defaultIKOptions_;

	btTransform forwardKinCached_;
	ros::Time forwardKinTimestamp_;

	btTransform invKinCached_;
	InverseKinematicsReportPtr invKinReport_;
	ros::Time invKinTimestamp_;

	virtual InverseKinematicsReportPtr internalInverseSoln(const btTransform& pose, Arm* soln,const InverseKinematicsOptions& options) const;
public:
	KinematicSolver(Arm* arm);
	virtual void cloneInto(KinematicSolverPtr& other,Arm* arm) const;
	virtual ~KinematicSolver();

	InverseKinematicsOptions getDefaultIKOptions() const { return defaultIKOptions_; }
	void setDefaultIKOptions(const InverseKinematicsOptions& options){ defaultIKOptions_ = options; }

	virtual int forward(btTransform& pose) const;
	btTransform forwardPose() const;
	InverseKinematicsReportPtr inverse(const btTransform& pose);
	InverseKinematicsReportPtr inverse(const btTransform& pose,const InverseKinematicsOptions& options);
	InverseKinematicsReportPtr inverseSoln(const btTransform& pose, boost::shared_ptr<Arm>& soln) const;
	virtual InverseKinematicsReportPtr inverseSoln(const btTransform& pose, boost::shared_ptr<Arm>& soln,const InverseKinematicsOptions& options) const;

	Eigen::VectorXf jointVector() const { return jointPositionVector(); }
	Eigen::VectorXf jointPositionVector() const;
	Eigen::VectorXf jointVelocityVector() const;

	Eigen::VectorXf motorPositionVector() const;
	Eigen::VectorXf motorVelocityVector() const;
	Eigen::VectorXf motorTorqueVector() const;
private:
	bool checkJointLimits1(float d_act, float thp_act, float g1_act, float g2_act,int validity[4]) const;
	bool checkJointLimits2(float ths_act, float the_act, float thr_act,float validity[3]) const;
	int setJointsWithLimits1(Arm* arm, float d_act, float thp_act, float g1_act, float g2_act) const;
	int setJointsWithLimits2(Arm* arm, float ths_act, float the_act, float thr_act) const;
};


#endif /* KINEMATICS_H_ */
