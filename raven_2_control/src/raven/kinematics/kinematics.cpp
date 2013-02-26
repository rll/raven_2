/*
 * kinematics.cpp
 *
 *  Created on: Oct 2, 2012
 *      Author: benk
 */

#include <raven/kinematics/kinematics.h>
#include <raven/kinematics/kinematics_defines.h>

#include "log.h"

#include <raven/state/arm.h>

InverseKinematicsReport::InverseKinematicsReport() : success_(false) {

}

static int numKS = 0;
KinematicSolver::KinematicSolver(Arm* arm) : arm_(arm), checkJointLimits_(true), truncateJointsAtLimits_(true), forwardKinTimestamp_(0), invKinTimestamp_(0) {
	//printf("+KS %i %p\n",++numKS,this);
}

KinematicSolver::~KinematicSolver() {
	//printf("-KS %i %p\n",--numKS,this);
}

void
KinematicSolver::cloneInto(KinematicSolverPtr& other,Arm* arm) const {
	if (!other) {
		other.reset(new KinematicSolver(*this));
		other->arm_ = arm;
		return;
	}
	*other = *this;
	other->arm_ = arm;
}

int
KinematicSolver::forward(btTransform& pose) const {
	if (forwardKinTimestamp_ == arm_->timestamp()) {
		pose = forwardKinCached_;
		return 0;
	}

	Arm::IdType armId = arm_->id();

	pose = actual_world_to_ik_world(armId)
						* Tw2b
						* Zs(THS_TO_IK(armId,arm_->getJointByType(Joint::Type::SHOULDER_)->position()))
						* Xu
						* Ze(THE_TO_IK(armId,arm_->getJointByType(Joint::Type::ELBOW_)->position()))
						* Xf
						* Zr(THR_TO_IK(armId,arm_->getJointByType(Joint::Type::ROTATION_)->position()))
						* Zi(D_TO_IK(armId,arm_->getJointByType(Joint::Type::INSERTION_)->position()))
						* Xip
						* Zp(THP_TO_IK(armId,arm_->getJointByType(Joint::Type::WRIST_)->position()))
						* Xpy
						* Zy(THY_TO_IK_FROM_FINGERS(armId,arm_->getJointByType(Joint::Type::FINGER1_)->position(),arm_->getJointByType(Joint::Type::FINGER2_)->position()))
						* Tg;

	//int grasp = MECH_GRASP_FROM_MECH_FINGERS(armId,arm_->getJointByType(Joint::Type::GRIPPER1_)->position(),arm_->getJointByType(Joint::Type::GRIPPER2_)->position());

	const_cast<KinematicSolver*>(this)->forwardKinCached_ = pose;
	const_cast<KinematicSolver*>(this)->forwardKinTimestamp_ = arm_->timestamp();
	return 0;
}

btTransform
KinematicSolver::forwardPose() const {
	btTransform pose;
	int ret = forward(pose);
	return pose;
}

InverseKinematicsReportPtr
KinematicSolver::inverse(const btTransform& pose) {
	if (invKinTimestamp_ == arm_->timestamp() && pose == invKinCached_) {
		return invKinReport_;
	}

	arm_->holdUpdateBegin();
	InverseKinematicsReportPtr report = internalInverseSoln(pose,arm_);
	arm_->holdUpdateEnd();

	invKinCached_ = pose;
	invKinReport_ = report;
	invKinTimestamp_ = arm_->timestamp();
	return report;
}

InverseKinematicsReportPtr
KinematicSolver::inverseSoln(const btTransform& pose, boost::shared_ptr<Arm>& soln) const {
	arm_->cloneInto(soln);

	soln->holdUpdateBegin();
	InverseKinematicsReportPtr report = internalInverseSoln(pose,soln.get());
	soln->holdUpdateEnd();

	if (!report->success()) {
		soln.reset();
	}

	return report;
}

Eigen::VectorXf
KinematicSolver::jointPositionVector() const {
	//TODO: implement
	return arm_->jointPositionVector();
}
Eigen::VectorXf
KinematicSolver::jointVelocityVector() const {
	//TODO: implement
	return arm_->jointVelocityVector();
}

Eigen::VectorXf
KinematicSolver::motorPositionVector() const {
	//TODO: implement
	return arm_->motorPositionVector();
}
Eigen::VectorXf
KinematicSolver::motorVelocityVector() const {
	//TODO: implement
	return arm_->motorVelocityVector();
}
Eigen::VectorXf
KinematicSolver::motorTorqueVector() const {
	//TODO: implement
	return arm_->motorTorqueVector();
}

InverseKinematicsReportPtr
KinematicSolver::internalInverseSoln(const btTransform& pose, Arm* arm) const {
	InverseKinematicsReportPtr report(new InverseKinematicsReport());

	Arm::IdType armId = arm->id();


	// desired tip position
//	btVector3 currentPoint = btVector3(mech->pos.x,mech->pos.y,mech->pos.z) / MICRON_PER_M;
//	btVector3 actualPoint = btVector3(pos_d->x,pos_d->y,pos_d->z) / MICRON_PER_M;
//	btMatrix3x3 actualOrientation = toBt(ori_d->R);
//	btTransform actualPose(actualOrientation,actualPoint);

//	btTransform actualPose_fk = pose;

//	tb_angles currentPoseAngles = tb_angles(mech->ori.R);
//	tb_angles actualPoseAngles = tb_angles(ori_d->R);

	//float grasp = GRASP_TO_IK(armId,mech->ori_d.grasp);
	float grasp = arm->getJointByType(Joint::Type::GRASP_)->position();

//	if (print) {
//		log_msg("j s % 2.1f e % 2.1f r % 2.1f i % 1.3f p % 2.1f y % 2.1f g % 2.1f g1 % 2.1f g2 % 2.1f",
//				arm->getJointByType(Joint::Type::SHOULDER_)->position() RAD2DEG,
//				arm->getJointByType(Joint::Type::ELBOW_)->position() RAD2DEG,
//				arm->getJointByType(Joint::Type::TOOL_ROT_)->position() RAD2DEG,
//				arm->getJointByType(Joint::Type::INSERTION__)->position(),
//				arm->getJointByType(Joint::Type::WRIST_)->position() RAD2DEG,
//				fix_angle(arm->getJointByType(Joint::Type::GRIPPER1_)->position() - arm->getJointByType(Joint::Type::GRIPPER2_)->position()) / 2 RAD2DEG,
//				(arm->getJointByType(Joint::Type::GRIPPER1_)->position() + arm->getJointByType(Joint::Type::GRIPPER2_)->position()) RAD2DEG,
//				arm->getJointByType(Joint::Type::GRIPPER1_)->position() RAD2DEG,arm->getJointByType(Joint::Type::GRIPPER2_)->position() RAD2DEG);
//		log_msg("v s % 2.1f e % 2.1f r % 2.1f i % 1.3f p % 2.1f y % 2.1f g % 2.1f g1 % 2.1f g2 % 2.1f",
//				arm->getJointByType(Joint::Type::SHOULDER].jvel RAD2DEG,
//				arm->getJointByType(Joint::Type::ELBOW].jvel RAD2DEG,
//				arm->getJointByType(Joint::Type::TOOL_ROT].jvel RAD2DEG,
//				arm->getJointByType(Joint::Type::INSERTION_].jvel,
//				arm->getJointByType(Joint::Type::WRIST].jvel RAD2DEG,
//				(arm->getJointByType(Joint::Type::GRASP1].jvel - arm->getJointByType(Joint::Type::GRASP2].jvel) / 2 RAD2DEG,
//				arm->getJointByType(Joint::Type::GRASP1].jvel + arm->getJointByType(Joint::Type::GRASP2].jvel RAD2DEG,
//				arm->getJointByType(Joint::Type::GRASP1].jvel RAD2DEG,arm->getJointByType(Joint::Type::GRASP2].jvel RAD2DEG);
//		log_msg("t s % 1.3f e % 1.3f r % 1.3f i % 1.3f p % 1.3f g1 % 1.3f g2 % 1.3f",arm->getJointByType(Joint::Type::SHOULDER].tau_d,
//				arm->getJointByType(Joint::Type::ELBOW].tau_d,arm->getJointByType(Joint::Type::TOOL_ROT].tau_d,arm->getJointByType(Joint::Type::INSERTION_].tau_d,arm->getJointByType(Joint::Type::WRIST].tau_d,
//				arm->getJointByType(Joint::Type::GRASP1].tau_d,arm->getJointByType(Joint::Type::GRASP2].tau_d);
//		log_msg("d s %d e %d r %d i %d p %d g1 %d g2 %d",tToDACVal(&(arm->getJointByType(Joint::Type::SHOULDER])),
//				tToDACVal(&(arm->getJointByType(Joint::Type::ELBOW])),tToDACVal(&(arm->getJointByType(Joint::Type::TOOL_ROT])),tToDACVal(&(arm->getJointByType(Joint::Type::INSERTION_])),tToDACVal(&(arm->getJointByType(Joint::Type::WRIST])),
//				tToDACVal(&(arm->getJointByType(Joint::Type::GRASP1])),tToDACVal(&(arm->getJointByType(Joint::Type::GRASP2])));
//		log_msg("cp (% 1.3f,% 1.3f,% 1.3f\typr (% 1.3f,% 1.3f,% 1.3f))",
//				currentPoint.x(),currentPoint.y(),currentPoint.z(),
//				currentPoseAngles.yaw_deg,currentPoseAngles.pitch_deg,currentPoseAngles.roll_deg);
//		log_msg("pt (% 1.3f,% 1.3f,% 1.3f)\typr (% 1.3f,% 1.3f,% 1.3f)\tg % 1.3f",
//				actualPoint.x(),actualPoint.y(),actualPoint.z(),
//				actualPoseAngles.yaw_deg,actualPoseAngles.pitch_deg,actualPoseAngles.roll_deg,
//				grasp);
//
//		btVector3 point = actualPose_fk.getOrigin();
//		tb_angles angles = tb_angles(actualPose_fk.getBasis());
//		log_msg("fp (% 1.3f,% 1.3f,% 1.3f)\typr (% 2.1f,% 2.1f,% 2.1f)\tg % 1.3f",
//				point.x(),point.y(),point.z(),
//				angles.yaw_deg,angles.pitch_deg,angles.roll_deg,
//				grasp);
//
//	}

	/*
	 * Actual pose is in the actual world frame, so we have <actual_world to gripper>
	 * The ik world frame is the base frame.
	 * Therefore, we need <base to actual_world> to get <base to gripper>.
	 * <base to actual_world> is inverse of <actual_world to base>
	 *
	 * Since the ik is based on the yaw frame (to which the gripper is fixed), we
	 * take the pose of the yaw frame, not the gripper frame
	 */
	btTransform ik_pose = ik_world_to_actual_world(armId) * pose * Tg.inverse();

	btMatrix3x3 ik_orientation = ik_pose.getBasis();
	btVector3 ik_point = ik_pose.getOrigin();

	tb_angles ikPoseAngles = tb_angles(ik_pose);
//	if (print) {
//		log_msg("ik (%0.4f,%0.4f,%0.4f)\typr (%0.4f,%0.4f,%0.4f)",
//				ik_point.x(),ik_point.y(),ik_point.z(),
//				ikPoseAngles.yaw_deg,ikPoseAngles.pitch_deg,ikPoseAngles.roll_deg);
//	}

	float ths_offset, thr_offset, thp_offset;
	if (arm->isGold()) {
		ths_offset = SHOULDER_OFFSET_GOLD;
		thr_offset = TOOL_ROT_OFFSET_GOLD;
		thp_offset = WRIST_OFFSET_GOLD;
	} else {
		ths_offset = SHOULDER_OFFSET_GREEN;
		thr_offset = TOOL_ROT_OFFSET_GREEN;
		thp_offset = WRIST_OFFSET_GREEN;
	}

	const float th12 = THETA_12;
	const float th23 = THETA_23;

	const float ks12 = sin(th12);
	const float kc12 = cos(th12);
	const float ks23 = sin(th23);
	const float kc23 = cos(th23);

	const float dw = DW;

	btTransform Tworld_to_gripper = ik_pose;
	btTransform Tgripper_to_world = ik_pose.inverse();

//	if (print) {
//		log_msg("Tw2g: %d",PRINT_EVERY);
//		for (int i=0;i<3;i++) {
//			log_msg("   %0.4f\t%0.4f\t%0.4f\t%0.4f",ik_pose.getBasis()[i][0],ik_pose.getBasis()[i][1],ik_pose.getBasis()[i][2],ik_pose.getOrigin()[i]);
//		}
//	}

	btVector3 origin_in_gripper_frame = Tgripper_to_world.getOrigin();
	float px = origin_in_gripper_frame.x();
	float py = origin_in_gripper_frame.y();
	float pz = origin_in_gripper_frame.z();

	float thy = atan2f(py,-px);

	float thp;
	if (fabs(thy) < 0.001) {
		thp = atan2f(-pz, -px/cos(thy) - dw);
		//			if (print) { log_msg("zero thy: %0.4f, (%0.4f, %0.4f, %0.4f)",thp,px,py,pz); }
	} else {
		thp = atan2f(-pz,  py/sin(thy) - dw);
	}

	float d = -pz / sin(thp);

	float d_act, thp_act, thy_act, g1_act, g2_act;
	d_act = D_FROM_IK(armId,d);
	thp_act = THP_FROM_IK(armId,thp);
	thy_act = THY_FROM_IK(armId,thy,grasp);
	g1_act = FINGER1_FROM_IK(armId,thy,grasp);
	g2_act = FINGER2_FROM_IK(armId,thy,grasp);

	//check angles
	int validity1[4];
	bool valid1 = checkJointLimits1(d_act,thp_act,g1_act,g2_act,validity1);
	if (!valid1) {
//		if (_curr_rl == 3 && !(DISABLE_ALL_PRINTING)) {
//			printf("ik %d invalid --1-- d [%d] % 2.4f \tp [%d] % 3.1f\ty [%d %d] % 3.1f\n",
//					armId,
//					validity1[0],              d_act,
//					validity1[1],              thp_act RAD2DEG,
//					validity1[2],validity1[3], thy_act RAD2DEG);
//		}
		return report;
	}

	//set joints
	//setJointsWithLimits1(mech,d_act,thp_act,thy_act,grasp);


	btVector3 z_roll_in_world = btTransform(Zi(d) * Xip * Zp(thp) * Xpy * Zy(thy) * Tg * Tgripper_to_world).invXform(btVector3(0,0,1));
	btVector3 x_roll_in_world = btTransform(Zi(d) * Xip * Zp(thp) * Xpy * Zy(thy) * Tg * Tgripper_to_world).invXform(btVector3(1,0,0));

	float zx = z_roll_in_world.x();
	float zy = z_roll_in_world.y();
	float zz = z_roll_in_world.z();

	float xx = x_roll_in_world.x();
	float xy = x_roll_in_world.y();
	float xz = x_roll_in_world.z();

	float cthe = (zy + kc12*kc23) / (ks12*ks23);

	float the_1 = acos(cthe);
	float the_2 = -acos(cthe);

	float the_opt[2];
	the_opt[0] = the_1;
	the_opt[1] = the_2;

	float ths_opt[2];
	float thr_opt[2];

	bool opts_valid[2];
	float validity2[2][4];

	float ths_act[2];
	float the_act[2];
	float thr_act[2];

	for (int i=0;i<2;i++) {
		float sthe_tmp = sin(the_opt[i]);
		float C1 = ks12*kc23 + kc12*ks23*cthe;
		float C2 = ks23 * sthe_tmp;
		float C3 = C2 + C1*C1 / C2;

		ths_opt[i] = atan2(
				-sgn(C3)*(zx - C1 * zz / C2),
				sgn(C3)*(zz + C1 * zx / C2));

		float sths_tmp = sin(ths_opt[i]);
		float cths_tmp = cos(ths_opt[i]);

		float C4 = ks12 * sin(the_opt[i]);
		float C5 = kc12 * ks23 + ks12 * kc23 * cos(the_opt[i]);
		float C6 = kc23*(sthe_tmp * sths_tmp - kc12*cthe*cths_tmp) + cths_tmp*ks12*ks23;
		float C7 = cthe*sths_tmp + kc12*cths_tmp*sthe_tmp;

		thr_opt[i] = atan2(
				(xx - C7 * xy / C4) / (C6 + C7*C5/C4),
				(xx + C6 * xy / C5) / (-C6*C4/C5 - C7));

		ths_act[i] = THS_FROM_IK(armId,ths_opt[i]);
		the_act[i] = THE_FROM_IK(armId,the_opt[i]);
		thr_act[i] = THR_FROM_IK(armId,thr_opt[i]);

//		if (print) {
//			log_msg("j s % 3.1f e % 3.1f r % 3.1f i % 1.3f p % 3.1f y % 3.1f g % 3.1f g1 % 3.1f g2 % 3.1f",
//					arm->getJointByType(Joint::Type::SHOULDER_)->position() RAD2DEG,
//					arm->getJointByType(Joint::Type::ELBOW_)->position() RAD2DEG,
//					arm->getJointByType(Joint::Type::TOOL_ROT_)->position() RAD2DEG,
//					arm->getJointByType(Joint::Type::INSERTION__)->position(),
//					arm->getJointByType(Joint::Type::WRIST_)->position() RAD2DEG,
//					THY_MECH_FROM_FINGERS(armIdFromMechType(mech->type),arm->getJointByType(Joint::Type::GRIPPER1_)->position(), arm->getJointByType(Joint::Type::GRIPPER2_)->position()) RAD2DEG,//fix_angle(arm->getJointByType(Joint::Type::GRIPPER2_)->position() - arm->getJointByType(Joint::Type::GRIPPER1_)->position(),0) / 2  RAD2DEG,
//					mech->ori.grasp * 1000. RAD2DEG,
//					fix_angle(arm->getJointByType(Joint::Type::GRIPPER1_)->position() + arm->getJointByType(Joint::Type::GRIPPER2_)->position(),0) RAD2DEG,
//					arm->getJointByType(Joint::Type::GRIPPER1_)->position() RAD2DEG, arm->getJointByType(Joint::Type::GRIPPER2_)->position() RAD2DEG);
//			log_msg("%d s % 3.1f e % 3.1f r % 3.1f i % 1.3f p % 3.1f y % 3.1f g % 3.1f g1 % 3.1f g2 % 3.1f",i,
//					ths_act[i] RAD2DEG,
//					the_act[i] RAD2DEG,
//					thr_act[i] RAD2DEG,
//					d_act,
//					thp_act RAD2DEG,
//					thy_act RAD2DEG,
//					grasp RAD2DEG,
//					g1_act RAD2DEG,
//					g2_act RAD2DEG);
//
//			if (ths_act != ths_act) {
//				log_msg("C1 %0.4f\tC2 %0.4f\tC3 %0.4f\tC4 %0.4f\tC5 %0.4f\tC6 %0.4f\tC7 %0.4f\t",C1,C2,C3,C4,C5,C6,C7);
//				log_msg("ks23 %0.4f\tsthe_tmp %0.4f",ks23 , sthe_tmp);
//				log_msg("cthe %0.4f\tzy %0.4f\tkc12 %0.4f\tkc23 %0.4f\tks12 %0.4f\tks23 %0.4f",cthe,zy,kc12,kc23,ks12,ks23);
//			}
//		}

		bool valid2 = checkJointLimits2(ths_act[i],the_act[i],thr_act[i],validity2[i]);
		opts_valid[i] = valid2;

		if (valid2) {
			float ths_diff, the_diff, d_diff, thr_diff, thp_diff, thg1_diff, thg2_diff;
			//set joints
			setJointsWithLimits1(arm,d_act,thp_act,g1_act,g2_act);
			setJointsWithLimits2(arm,ths_act[i],the_act[i],thr_act[i]);

			ths_diff = arm->getJointByType(Joint::Type::SHOULDER_)->position() - ths_act[i];
			the_diff = arm->getJointByType(Joint::Type::ELBOW_)->position()    - the_act[i];
			d_diff = arm->getJointByType(Joint::Type::INSERTION_)->position()    - d_act;
			thr_diff = arm->getJointByType(Joint::Type::ROTATION_)->position() - thr_act[i];
			thp_diff = arm->getJointByType(Joint::Type::WRIST_)->position()    - thp_act;
			thg1_diff = arm->getJointByType(Joint::Type::FINGER1_)->position()   - g1_act;
			thg2_diff = arm->getJointByType(Joint::Type::FINGER2_)->position()   - g2_act;
			/*
			ths_diff = arm->getJointByType(Joint::Type::SHOULDER_)->position()_d - ths_act;
			the_diff = arm->getJointByType(Joint::Type::ELBOW_)->position()_d    - the_act;
			d_diff = arm->getJointByType(Joint::Type::INSERTION__)->position()_d    - d_act;
			thr_diff = arm->getJointByType(Joint::Type::TOOL_ROT_)->position()_d - thr_act;
			thp_diff = arm->getJointByType(Joint::Type::WRIST_)->position()_d    - thp_act;
			thg1_diff = arm->getJointByType(Joint::Type::GRIPPER1_)->position()_d   - g1_act;
			thg2_diff = arm->getJointByType(Joint::Type::GRIPPER2_)->position()_d   - g2_act;
			 */

//			if (print) {
//				log_msg("%d s % 2.1f e % 2.1f r % 2.1f i % 1.3f p % 2.1f        g % 2.1f g1 % 2.1f g2 % 2.1f",i,
//						arm->getJointByType(Joint::Type::SHOULDER_)->position()_d RAD2DEG,
//						arm->getJointByType(Joint::Type::ELBOW_)->position()_d RAD2DEG,
//						arm->getJointByType(Joint::Type::TOOL_ROT_)->position()_d RAD2DEG,
//						arm->getJointByType(Joint::Type::INSERTION__)->position()_d,
//						arm->getJointByType(Joint::Type::WRIST_)->position()_d RAD2DEG,
//						grasp RAD2DEG,
//						arm->getJointByType(Joint::Type::GRIPPER1_)->position()_d RAD2DEG,
//						arm->getJointByType(Joint::Type::GRIPPER2_)->position()_d RAD2DEG);
//				log_msg("diff:");
//				log_msg("R s %0.4f e %0.4f r %0.4f d %0.4f p %0.4f                     g1 %0.4f  g2 %0.4f",
//						ths_diff,the_diff,thr_diff,d_diff,thp_diff,thg1_diff,thg2_diff);
//				log_msg("D s %0.4f e %0.4f r %0.4f d %0.4f p %0.4f                     g1 %0.4f  g2 %0.4f",
//						ths_diff*180/M_PI,the_diff*180/M_PI,thr_diff*180/M_PI,d_diff,
//						thp_diff*180/M_PI,thg1_diff*180/M_PI,thg2_diff*180/M_PI);
//
//			}
//
//			if (i==1 && _curr_rl == 3) {
//				//printf("ik ok! %d\n",_ik_counter);
//			}
		}
	}

	if (opts_valid[0]) {
		report->success_ = true;
		return report;
	} else if (opts_valid[1]) {
//		bool ENABLE_PARTIAL_INVALID_IK_PRINTING = false;
//		if (ENABLE_PARTIAL_INVALID_IK_PRINTING && (_curr_rl == 3 || print) && !(DISABLE_ALL_PRINTING)) {
//			printf("ik %d    ok   **2** s %1.4f %1.4f\te %1.4f %1.4f\tr %1.4f %1.4f\n",
//					armId,
//					ths_act[0] RAD2DEG,validity2[0][0],
//					the_act[0] RAD2DEG,validity2[0][1],
//					thr_act[0] RAD2DEG,validity2[0][2]);
//			/*
//			printf("%7d          d %0.4f  \tp %0.4f  \ty %0.4f\n",_ik_counter,
//								d_act,thp_act RAD2DEG,thy_act RAD2DEG);
//			printf("x (%f, %f, %f)  z (%f, %f, %f) %f\n",xx,xy,xz,zx,zy,zz,cthe);
//			printf("norms %f %f\n",x_roll_in_world.length(),z_roll_in_world.length());
//			printf("(zy + kc12*kc23): %f (%f, %f)\n",(zy + kc12*kc23),zy, kc12*kc23);
//			printf("(ks12*ks23): %f\n",(ks12*ks23));
//			 */
//		}
		report->success_ = true;
		return report;
	} else {
		const float maxValidDist = 3 DEG2RAD;
		float valid_dist[2];
		for (int i=0;i<2;i++) {
			float sum = 0;
			for (int j=0;j<3;j++) {
				float v = fabs(validity2[i][j]);
				sum += v*v;
			}
			valid_dist[i] = sqrt(sum);
		}

		bool use0 = valid_dist[0] < maxValidDist && valid_dist[0] < valid_dist[1];
		bool use1 = valid_dist[1] < maxValidDist && valid_dist[0] > valid_dist[1];
		printf("ik validity distances: (%s | %s) % 1.3f\t%f\n",use0?"Y":" ",use1?"Y":" ",valid_dist[0],valid_dist[1]);
		if (valid_dist[0] < maxValidDist && valid_dist[0] < valid_dist[1]) {
			printf("setting joints to ik soln 1\n");
			setJointsWithLimits1(arm,d_act,thp_act,g1_act,g2_act);
			setJointsWithLimits2(arm,ths_act[0],the_act[0],thr_act[0]);
		} else if (valid_dist[1] < maxValidDist) {
			printf("setting joints to ik soln 2\n");
			setJointsWithLimits1(arm,d_act,thp_act,g1_act,g2_act);
			setJointsWithLimits2(arm,ths_act[1],the_act[1],thr_act[1]);
		}

//		if ((_curr_rl == 3 || print) && !(DISABLE_ALL_PRINTING)) {
//			printf("ik %d invalid **2** s %1.4f % 2.1f\te %1.4f % 2.1f\tr %1.4f % 2.1f\n",
//					armId,
//					ths_act[0] RAD2DEG,validity2[0][0] RAD2DEG,
//					the_act[0] RAD2DEG,validity2[0][1] RAD2DEG,
//					thr_act[0] RAD2DEG,validity2[0][2] RAD2DEG);
//			printf("                   s %1.4f % 2.1f\te %1.4f % 2.1f\tr %1.4f % 2.1f\n",
//					ths_act[1] RAD2DEG,validity2[1][0] RAD2DEG,
//					the_act[1] RAD2DEG,validity2[1][1] RAD2DEG,
//					thr_act[1] RAD2DEG,validity2[1][2] RAD2DEG);
//		}
		return report;
	}

	return report;
}

int
KinematicSolver::setJointsWithLimits1(Arm* arm, float d_act, float thp_act, float g1_act, float g2_act) const {
	//	if (_ik_counter % PRINT_EVERY == 0) {
	//		log_msg("setting joints 1");
	//	}
	arm->getJointByType(Joint::Type::INSERTION_)->setPosition(d_act);
	arm->getJointByType(Joint::Type::WRIST_)->setPosition(thp_act); //WRIST_HOME_ANGLE; int WARNING_WRIST_NOT_SET;
	arm->getJointByType(Joint::Type::FINGER1_)->setPosition(g1_act); //GRASP1_HOME_ANGLE; int WARNING_GRASP1_NOT_SET;
	arm->getJointByType(Joint::Type::FINGER2_)->setPosition(g2_act); //GRASP2_HOME_ANGLE; int WARNING_GRASP2_NOT_SET;

	int limits=0;

	if (arm->getJointByType(Joint::Type::INSERTION_)->position()  < Z_INS_MIN_LIMIT) {
		limits++;
		log_msg("insertion min limit");
		arm->getJointByType(Joint::Type::INSERTION_)->setPosition(Z_INS_MIN_LIMIT);
	} else if (arm->getJointByType(Joint::Type::INSERTION_)->position()  > Z_INS_MAX_LIMIT) {
		limits++;
		log_msg("insertion max limit");
		arm->getJointByType(Joint::Type::INSERTION_)->setPosition(Z_INS_MAX_LIMIT);
	}

	if (arm->getJointByType(Joint::Type::WRIST_)->position()  < TOOL_WRIST_MIN_LIMIT) {
		log_msg("wrist min limit");
		limits++;
		arm->getJointByType(Joint::Type::WRIST_)->setPosition(TOOL_WRIST_MIN_LIMIT);
	} else if (arm->getJointByType(Joint::Type::WRIST_)->position()  > TOOL_WRIST_MAX_LIMIT) {
		limits++;
		log_msg("wrist max limit");
		arm->getJointByType(Joint::Type::WRIST_)->setPosition(TOOL_WRIST_MAX_LIMIT);
	}

	if (arm->getJointByType(Joint::Type::FINGER1_)->position()  < TOOL_GRASP1_MIN_LIMIT) {
		limits++;
		log_msg("grasp1 min limit");
		arm->getJointByType(Joint::Type::FINGER1_)->setPosition(TOOL_GRASP1_MIN_LIMIT);
	} else if (arm->getJointByType(Joint::Type::FINGER1_)->position()  > TOOL_GRASP1_MAX_LIMIT) {
		limits++;
		log_msg("grasp1 max limit");
		arm->getJointByType(Joint::Type::FINGER1_)->setPosition(TOOL_GRASP1_MAX_LIMIT);
	}

	if (arm->getJointByType(Joint::Type::FINGER2_)->position()  < TOOL_GRASP2_MIN_LIMIT) {
		log_msg("grasp2 min limit");
		limits++;
		arm->getJointByType(Joint::Type::FINGER2_)->setPosition(TOOL_GRASP2_MIN_LIMIT);
	} else if (arm->getJointByType(Joint::Type::FINGER2_)->position()  > TOOL_GRASP2_MAX_LIMIT) {
		limits++;
		log_msg("grasp2 max limit");
		arm->getJointByType(Joint::Type::FINGER2_)->setPosition(TOOL_GRASP2_MAX_LIMIT);
	}

	return limits;
}

int
KinematicSolver::setJointsWithLimits2(Arm* arm, float ths_act, float the_act, float thr_act) const {
	//	if (_ik_counter % PRINT_EVERY == 0) {
	//		log_msg("setting joints 2");
	//	}
	arm->getJointByType(Joint::Type::SHOULDER_)->setPosition(ths_act);
	arm->getJointByType(Joint::Type::ELBOW_)->setPosition(the_act);
	//arm->getJointByType(Joint::Type::TOOL_ROT_)->setPosition(fix_angle(thr_act + M_PI); //TOOL_ROT_HOME_ANGLE; int WARNING_ROT_NOT_SET;
	arm->getJointByType(Joint::Type::ROTATION_)->setPosition(thr_act); //TOOL_ROT_HOME_ANGLE; int WARNING_ROT_NOT_SET;

	int limits = 0;
	if (arm->getJointByType(Joint::Type::SHOULDER_)->position() < SHOULDER_MIN_LIMIT) {
		limits++;
		log_msg("shoulder min limit");
		arm->getJointByType(Joint::Type::SHOULDER_)->setPosition(SHOULDER_MIN_LIMIT);
	} else if (arm->getJointByType(Joint::Type::SHOULDER_)->position() > SHOULDER_MAX_LIMIT) {
		limits++;
		log_msg("shoulder max limit");
		arm->getJointByType(Joint::Type::SHOULDER_)->setPosition(SHOULDER_MAX_LIMIT);
	}

	if (arm->getJointByType(Joint::Type::ELBOW_)->position() < ELBOW_MIN_LIMIT) {
		limits++;
		log_msg("elbow min limit");
		arm->getJointByType(Joint::Type::ELBOW_)->setPosition(ELBOW_MIN_LIMIT);
	} else if (arm->getJointByType(Joint::Type::ELBOW_)->position() > ELBOW_MAX_LIMIT) {
		limits++;
		log_msg("elbow max limit");
		arm->getJointByType(Joint::Type::ELBOW_)->setPosition(ELBOW_MAX_LIMIT);
	}

	if (arm->getJointByType(Joint::Type::ROTATION_)->position() < TOOL_ROLL_MIN_LIMIT) {
		limits++;
		log_msg("roll % 3.1fdeg under min limit",arm->getJointByType(Joint::Type::ROTATION_)->position() RAD2DEG);
		arm->getJointByType(Joint::Type::ROTATION_)->setPosition(TOOL_ROLL_MIN_LIMIT);
	} else if (arm->getJointByType(Joint::Type::ROTATION_)->position() > TOOL_ROLL_MAX_LIMIT) {
		limits++;
		log_msg("roll % 3.1fdeg over max limit",arm->getJointByType(Joint::Type::ROTATION_)->position() RAD2DEG);
		arm->getJointByType(Joint::Type::ROTATION_)->setPosition(TOOL_ROLL_MAX_LIMIT);
	}

	return limits;
}

bool
KinematicSolver::checkJointLimits1(float d_act, float thp_act, float g1_act, float g2_act,int validity[4]) const {
	validity[0] = 0;
	validity[1] = 0;
	validity[2] = 0;
	validity[3] = 0;

	bool any_nan = false;
	if (d_act != d_act) { /*if (_ik_counter % PRINT_EVERY == 0) { log_msg("d_act is nan"); }*/ any_nan = true; };
	if (thp_act != thp_act) { /*if (_ik_counter % PRINT_EVERY == 0) { log_msg("thp_act is nan"); }*/ any_nan = true; };
	if (g1_act != g1_act) { /*if (_ik_counter % PRINT_EVERY == 0) { log_msg("g1_act is nan"); }*/ any_nan = true; };
	if (g2_act != g2_act) { /*if (_ik_counter % PRINT_EVERY == 0) { log_msg("g2_act is nan"); }*/ any_nan = true; };
	if (any_nan) {
		return false;
	}

	bool bad = false;
	if (d_act < Z_INS_MIN_LIMIT) {
		validity[0] = -1;
		bad = true;
	}
	if (d_act > Z_INS_MAX_LIMIT) {
		validity[0] = +1;
		bad = true;
	}
	if (thp_act < TOOL_WRIST_MIN_LIMIT) {
		validity[1] = -1;
		bad = true;
	}
	if (thp_act > TOOL_WRIST_MAX_LIMIT) {
		validity[1] = +1;
		bad = true;
	}
	if (g1_act < TOOL_GRASP1_MIN_LIMIT) {
		validity[2] = -1;
		bad = true;
	}
	if (g1_act > TOOL_GRASP1_MAX_LIMIT) {
		validity[2] = +1;
		bad = true;
	}
	if (g2_act < TOOL_GRASP2_MIN_LIMIT) {
		validity[3] = -1;
		bad = true;
	}
	if (g2_act > TOOL_GRASP2_MAX_LIMIT) {
		validity[3] = +1;
		bad = true;
	}
	return !bad;
}

bool
KinematicSolver::checkJointLimits2(float ths_act, float the_act, float thr_act,float validity[3]) const {
	validity[0] = 0;
	validity[1] = 0;
	validity[2] = 0;
	bool any_nan = false;
	if (ths_act != ths_act) { /*if (_ik_counter % PRINT_EVERY == 0) { log_msg("ths_act is nan"); }*/ any_nan = true; };
	if (the_act != the_act) { /*if (_ik_counter % PRINT_EVERY == 0) { log_msg("the_act is nan"); }*/ any_nan = true; };
	if (thr_act != thr_act) { /*if (_ik_counter % PRINT_EVERY == 0) { log_msg("thr_act is nan"); }*/ any_nan = true; };
	if (any_nan) {
		return false;
	}

	bool bad = false;
	if (ths_act < SHOULDER_MIN_LIMIT) {
		validity[0] = ths_act - SHOULDER_MIN_LIMIT;
		bad = true;
	}
	if (ths_act > SHOULDER_MAX_LIMIT) {
		validity[0] = ths_act - SHOULDER_MAX_LIMIT;
		bad = true;
	}
	if (the_act < ELBOW_MIN_LIMIT) {
		validity[1] = the_act - ELBOW_MIN_LIMIT;
		bad = true;
	}
	if (the_act > ELBOW_MAX_LIMIT) {
		validity[1] = the_act - ELBOW_MAX_LIMIT;
		bad = true;
	}
	if (thr_act < TOOL_ROLL_MIN_LIMIT) {
		validity[2] = thr_act - TOOL_ROLL_MIN_LIMIT;
		bad = true;
	}
	if (thr_act > TOOL_ROLL_MAX_LIMIT) {
		validity[2] = thr_act - TOOL_ROLL_MAX_LIMIT;
		bad = true;
	}
	return !bad;
}
