/*******************************
 *
 *  File: inv_kinmatics.c
 *
 *  Calculate the inverse kinematics from a point P (px, py,pz)
 *  expressed in base frame
 *
 **********************************/

/**
*  Old inverse kinematics for Raven_I removed.
*  See versions prior to 24-Aug-2011 for reference.
*/


#include <math.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <LinearMath/btTransform.h>
#include "hmatrix.h"
#include "inv_kinematics.h"
#include "utils.h"
#include "t_to_DAC_val.h"

#define EPS2 0.00001
#define EPS 0.01

#define DISABLE_ALL_PRINTING false

static const int PRINT_EVERY_PEDAL_UP   = 1000000;
static const int PRINT_EVERY_PEDAL_DOWN = 1000;
static int PRINT_EVERY = PRINT_EVERY_PEDAL_UP;

#define PRINT (_ik_counter % PRINT_EVERY == 0 && !(DISABLE_ALL_PRINTING))


extern bool disable_arm_id[2];
extern unsigned long int gTime;
extern int NUM_MECH;
int inv_kin_last_err = 0;

int check_joint_limits1(struct mechanism*);
int check_joint_limits2(struct mechanism*);
bool check_joint_limits1_new(float d_act, float thp_act, float g1_act, float g2_act,int validity[4]);
bool check_joint_limits2_new(float ths_act, float the_act, float thr_act,float validity[3]);
int set_joints_with_limits1(mechanism* mech, float d_act, float thp_act, float g1_act, float g2_act);
int set_joints_with_limits2(mechanism* mech, float ths_act, float the_act, float thr_act);

static int _ik_counter = 0;
static int _curr_rl = -1;
static int _prev_rl = -1;

/**
 * invKin - wrapper function that checks for correct runlevel
 *   and calls invMechKin for each mechanism in device
 *
 * \param device0 pointer to device struct
 * \param runlevel current runlevel
 *
 * Note the following special cases:
 *  If RL = RL_PEDAL_DN, pos_d is set to pos and we want that to propogate and use this function (using this function)
 *  If RL = RL_INIT,     jpos_d is set by Auto init function.  We don't want to overwrite here, so we return.
 */
void invKin(struct device *device0, struct param_pass* currParams)
{
  int ret=0;

  int runlevel = currParams->runlevel;
  int sublevel = currParams->sublevel;

  _prev_rl = _curr_rl;
  _curr_rl = runlevel;

  if (runlevel == RL_PEDAL_DN) {
	  PRINT_EVERY = PRINT_EVERY_PEDAL_DOWN;
  } else {
	  PRINT_EVERY = PRINT_EVERY_PEDAL_UP;
  }

#ifdef AUTO_INIT  // EE449 Return if we are in run level 1.2: auto initialize
  // Note that in RL 1.2 we are running the joint-level auto-init routine, so we don't want invKin to update jpos_d.  So we return here.
  if(runlevel == RL_INIT && sublevel == SL_AUTO_INIT) {
    return;
  }
#endif

  //Run inverse kinematics for each mech
  int mechnum=0; mechanism* _mech=NULL;
  while(loop_over_mechs(device0,_mech,mechnum)) {
	  if (true || _mech->type == GREEN_ARM) {
		  ret = invMechKinNew(_mech);
	  } else {
		  ret=0;//invMechKin( &(device0->mech[i]));
	  }
	  if (ret < 0) {
		  //    	log_msg("inv_kin failed with %d", ret);
		  return;
	  }
  }
}

//fool eclipse
float atan2f(float y,float x);
float fabsf(float val);
float cosf(float val);

//inline float* fromBt(btMatrix3x3 btR) {
//	float R[3][3];
//		for (int i = 0; i < 3; i++) {
//			for (int j = 0; j < 3; j++) {
//				R[i][j] = btR[i][j];
//			}
//		}
//	return R;
//}

struct InvKinematicsResult {
	bool success;
	float shoulder;
	float elbow;
	float rotation;
	float insertion;
	float wrist;
	float grasp1;
	float grasp2;
};

int invMechKinNew(struct mechanism *mech,bool test) {
	_ik_counter++;
	bool print = PRINT || (_prev_rl !=3 && _curr_rl == 3);
	struct position*    pos_d = &(mech->pos_d);
	struct orientation* ori_d = &(mech->ori_d);

	if (print) {
		log_msg("---------------------------");
		log_msg("---------------------------");
		if (test) {
			log_msg("----------TESTING----------");
		}
		//log_msg("time since last call: %1.4f",since_last_call.toSec());
	}

	int armId = armIdFromMechType(mech->type);


	// desired tip position
	btVector3 currentPoint = btVector3(mech->pos.x,mech->pos.y,mech->pos.z) / MICRON_PER_M;
	btVector3 actualPoint = btVector3(pos_d->x,pos_d->y,pos_d->z) / MICRON_PER_M;
	btMatrix3x3 actualOrientation = toBt(ori_d->R);
	btTransform actualPose(actualOrientation,actualPoint);

	btTransform actualPose_fk = actualPose;// = fwdKin(mech);

	tb_angles currentPoseAngles = tb_angles(mech->ori.R);
	tb_angles actualPoseAngles = tb_angles(ori_d->R);

	float grasp = GRASP_TO_IK(armId,mech->ori_d.grasp);

	if (print) {
		log_msg("j s % 2.1f e % 2.1f r % 2.1f i % 1.3f p % 2.1f y % 2.1f g % 2.1f g1 % 2.1f g2 % 2.1f",
				mech->joint[SHOULDER].jpos RAD2DEG,
				mech->joint[ELBOW].jpos RAD2DEG,
				mech->joint[TOOL_ROT].jpos RAD2DEG,
				mech->joint[Z_INS].jpos,
				mech->joint[WRIST].jpos RAD2DEG,
				fix_angle(mech->joint[GRASP1].jpos - mech->joint[GRASP2].jpos) / 2 RAD2DEG,
				(mech->joint[GRASP1].jpos + mech->joint[GRASP2].jpos) RAD2DEG,
				mech->joint[GRASP1].jpos RAD2DEG,mech->joint[GRASP2].jpos RAD2DEG);
		log_msg("v s % 2.1f e % 2.1f r % 2.1f i % 1.3f p % 2.1f y % 2.1f g % 2.1f g1 % 2.1f g2 % 2.1f",
				mech->joint[SHOULDER].jvel RAD2DEG,
				mech->joint[ELBOW].jvel RAD2DEG,
				mech->joint[TOOL_ROT].jvel RAD2DEG,
				mech->joint[Z_INS].jvel,
				mech->joint[WRIST].jvel RAD2DEG,
				(mech->joint[GRASP1].jvel - mech->joint[GRASP2].jvel) / 2 RAD2DEG,
				mech->joint[GRASP1].jvel + mech->joint[GRASP2].jvel RAD2DEG,
				mech->joint[GRASP1].jvel RAD2DEG,mech->joint[GRASP2].jvel RAD2DEG);
		log_msg("t s % 1.3f e % 1.3f r % 1.3f i % 1.3f p % 1.3f g1 % 1.3f g2 % 1.3f",mech->joint[SHOULDER].tau_d,
				mech->joint[ELBOW].tau_d,mech->joint[TOOL_ROT].tau_d,mech->joint[Z_INS].tau_d,mech->joint[WRIST].tau_d,
				mech->joint[GRASP1].tau_d,mech->joint[GRASP2].tau_d);
		log_msg("d s %d e %d r %d i %d p %d g1 %d g2 %d",tToDACVal(&(mech->joint[SHOULDER])),
				tToDACVal(&(mech->joint[ELBOW])),tToDACVal(&(mech->joint[TOOL_ROT])),tToDACVal(&(mech->joint[Z_INS])),tToDACVal(&(mech->joint[WRIST])),
				tToDACVal(&(mech->joint[GRASP1])),tToDACVal(&(mech->joint[GRASP2])));
		log_msg("cp (% 1.3f,% 1.3f,% 1.3f\typr (% 1.3f,% 1.3f,% 1.3f))",
				currentPoint.x(),currentPoint.y(),currentPoint.z(),
				currentPoseAngles.yaw_deg,currentPoseAngles.pitch_deg,currentPoseAngles.roll_deg);
		log_msg("pt (% 1.3f,% 1.3f,% 1.3f)\typr (% 1.3f,% 1.3f,% 1.3f)\tg % 1.3f",
				actualPoint.x(),actualPoint.y(),actualPoint.z(),
				actualPoseAngles.yaw_deg,actualPoseAngles.pitch_deg,actualPoseAngles.roll_deg,
				grasp);

		btVector3 point = actualPose_fk.getOrigin();
		tb_angles angles = tb_angles(actualPose_fk.getBasis());
		log_msg("fp (% 1.3f,% 1.3f,% 1.3f)\typr (% 2.1f,% 2.1f,% 2.1f)\tg % 1.3f",
				point.x(),point.y(),point.z(),
				angles.yaw_deg,angles.pitch_deg,angles.roll_deg,
				grasp);

	}

	/*
	 * Actual pose is in the actual world frame, so we have <actual_world to gripper>
	 * The ik world frame is the base frame.
	 * Therefore, we need <base to actual_world> to get <base to gripper>.
	 * <base to actual_world> is inverse of <actual_world to base>
	 *
	 * Since the ik is based on the yaw frame (to which the gripper is fixed), we
	 * take the pose of the yaw frame, not the gripper frame
	 */
	btTransform ik_pose = ik_world_to_actual_world(armId) * actualPose * Tg.inverse();

	btMatrix3x3 ik_orientation = ik_pose.getBasis();
	btVector3 ik_point = ik_pose.getOrigin();

	tb_angles ikPoseAngles = tb_angles(ik_pose);
	if (print) {
		log_msg("ik (%0.4f,%0.4f,%0.4f)\typr (%0.4f,%0.4f,%0.4f)",
				ik_point.x(),ik_point.y(),ik_point.z(),
				ikPoseAngles.yaw_deg,ikPoseAngles.pitch_deg,ikPoseAngles.roll_deg);
	}

	float ths_offset, thr_offset, thp_offset;
	if (mech->type == GOLD_ARM) {
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

	if (print) {
		log_msg("Tw2g: %d",PRINT_EVERY);
		for (int i=0;i<3;i++) {
			log_msg("   %0.4f\t%0.4f\t%0.4f\t%0.4f",ik_pose.getBasis()[i][0],ik_pose.getBasis()[i][1],ik_pose.getBasis()[i][2],ik_pose.getOrigin()[i]);
		}
	}

	btVector3 origin_in_gripper_frame = Tgripper_to_world.getOrigin();
	float px = origin_in_gripper_frame.x();
	float py = origin_in_gripper_frame.y();
	float pz = origin_in_gripper_frame.z();

	float thy = atan2f(py,-px);

	float thp;
	if (fabs(thy) < 0.001) {
		thp = atan2f(-pz, -px/cos(thy) - dw);
		if (print) { log_msg("zero thy: %0.4f, (%0.4f, %0.4f, %0.4f)",thp,px,py,pz); }
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
	bool valid1 = check_joint_limits1_new(d_act,thp_act,g1_act,g2_act,validity1);
	if (!valid1) {
		if (_curr_rl == 3 && !(DISABLE_ALL_PRINTING)) {
			printf("ik %d invalid --1-- d [%d] % 2.4f \tp [%d] % 3.1f\ty [%d %d] % 3.1f\n",
					armId,
					validity1[0],              d_act,
					validity1[1],              thp_act RAD2DEG,
					validity1[2],validity1[3], thy_act RAD2DEG);
		}
		return 0;
	}

	//set joints
	if (!test) {
		//set_joints_with_limits1(mech,d_act,thp_act,thy_act,grasp);
	}


	btVector3 z_roll_in_world = btTransform(Zi(d) * Xip * Zp(thp) * Xpy * Zy(thy) * Tg * Tgripper_to_world).invXform(btVector3(0,0,1));
	btVector3 x_roll_in_world = btTransform(Zi(d) * Xip * Zp(thp) * Xpy * Zy(thy) * Tg * Tgripper_to_world).invXform(btVector3(1,0,0));

	float zx = z_roll_in_world.x();
	float zy = z_roll_in_world.y();
	float zz = z_roll_in_world.z();

	float xx = x_roll_in_world.x();
	float xy = x_roll_in_world.y();
	float xz = x_roll_in_world.z();

	if (_curr_rl == 3) {
	}

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

		/*
		if (mech->type == GOLD_ARM) {
			ths_act = ths_opt[i] - ths_offset;
			the_act = the_opt[i];
			thr_act = fix_angle(-thr_opt[i] + thr_offset);
		} else {
			//log_msg("GREEN ARM KINEMATICS NOT IMPLEMENTED");
			//TODO: fix
			ths_act = fix_angle(-ths_opt[i] - ths_offset);
			the_act = fix_angle(the_opt[i] + M_PI);
			thr_act = fix_angle(-thr_opt[i] + thr_offset);
		}*/
		ths_act[i] = THS_FROM_IK(armId,ths_opt[i]);
		the_act[i] = THE_FROM_IK(armId,the_opt[i]);
		thr_act[i] = THR_FROM_IK(armId,thr_opt[i]);

		if (print) {
			log_msg("j s % 3.1f e % 3.1f r % 3.1f i % 1.3f p % 3.1f y % 3.1f g % 3.1f g1 % 3.1f g2 % 3.1f",
						mech->joint[SHOULDER].jpos RAD2DEG,
						mech->joint[ELBOW].jpos RAD2DEG,
						mech->joint[TOOL_ROT].jpos RAD2DEG,
						mech->joint[Z_INS].jpos,
						mech->joint[WRIST].jpos RAD2DEG,
						THY_MECH_FROM_FINGERS(armIdFromMechType(mech->type),mech->joint[GRASP1].jpos, mech->joint[GRASP2].jpos) RAD2DEG,//fix_angle(mech->joint[GRASP2].jpos - mech->joint[GRASP1].jpos,0) / 2  RAD2DEG,
						mech->ori.grasp * 1000. RAD2DEG,
						fix_angle(mech->joint[GRASP1].jpos + mech->joint[GRASP2].jpos,0) RAD2DEG,
						mech->joint[GRASP1].jpos RAD2DEG, mech->joint[GRASP2].jpos RAD2DEG);
			log_msg("%d s % 3.1f e % 3.1f r % 3.1f i % 1.3f p % 3.1f y % 3.1f g % 3.1f g1 % 3.1f g2 % 3.1f",i,
					ths_act[i] RAD2DEG,
					the_act[i] RAD2DEG,
					thr_act[i] RAD2DEG,
					d_act,
					thp_act RAD2DEG,
					thy_act RAD2DEG,
					grasp RAD2DEG,
					g1_act RAD2DEG,
					g2_act RAD2DEG);

			if (ths_act != ths_act) {
				log_msg("C1 %0.4f\tC2 %0.4f\tC3 %0.4f\tC4 %0.4f\tC5 %0.4f\tC6 %0.4f\tC7 %0.4f\t",C1,C2,C3,C4,C5,C6,C7);
				log_msg("ks23 %0.4f\tsthe_tmp %0.4f",ks23 , sthe_tmp);
				log_msg("cthe %0.4f\tzy %0.4f\tkc12 %0.4f\tkc23 %0.4f\tks12 %0.4f\tks23 %0.4f",cthe,zy,kc12,kc23,ks12,ks23);
			}
		}

		bool valid2 = check_joint_limits2_new(ths_act[i],the_act[i],thr_act[i],validity2[i]);
		opts_valid[i] = valid2;

		if (valid2) {
			float ths_diff, the_diff, d_diff, thr_diff, thp_diff, thg1_diff, thg2_diff;
				//set joints
				set_joints_with_limits1(mech,d_act,thp_act,g1_act,g2_act);
				set_joints_with_limits2(mech,ths_act[i],the_act[i],thr_act[i]);

				ths_diff = mech->joint[SHOULDER].jpos - ths_act[i];
				the_diff = mech->joint[ELBOW].jpos    - the_act[i];
				d_diff = mech->joint[Z_INS].jpos    - d_act;
				thr_diff = mech->joint[TOOL_ROT].jpos - thr_act[i];
				thp_diff = mech->joint[WRIST].jpos    - thp_act;
				thg1_diff = mech->joint[GRASP1].jpos   - g1_act;
				thg2_diff = mech->joint[GRASP2].jpos   - g2_act;
			/*
				ths_diff = mech->joint[SHOULDER].jpos_d - ths_act;
				the_diff = mech->joint[ELBOW].jpos_d    - the_act;
				d_diff = mech->joint[Z_INS].jpos_d    - d_act;
				thr_diff = mech->joint[TOOL_ROT].jpos_d - thr_act;
				thp_diff = mech->joint[WRIST].jpos_d    - thp_act;
				thg1_diff = mech->joint[GRASP1].jpos_d   - g1_act;
				thg2_diff = mech->joint[GRASP2].jpos_d   - g2_act;
			*/

			if (print) {
				log_msg("%d s % 2.1f e % 2.1f r % 2.1f i % 1.3f p % 2.1f        g % 2.1f g1 % 2.1f g2 % 2.1f",i,
						mech->joint[SHOULDER].jpos_d RAD2DEG,
						mech->joint[ELBOW].jpos_d RAD2DEG,
						mech->joint[TOOL_ROT].jpos_d RAD2DEG,
						mech->joint[Z_INS].jpos_d,
						mech->joint[WRIST].jpos_d RAD2DEG,
						grasp RAD2DEG,
						mech->joint[GRASP1].jpos_d RAD2DEG,
						mech->joint[GRASP2].jpos_d RAD2DEG);
				log_msg("diff:");
				log_msg("R s %0.4f e %0.4f r %0.4f d %0.4f p %0.4f                     g1 %0.4f  g2 %0.4f",
						ths_diff,the_diff,thr_diff,d_diff,thp_diff,thg1_diff,thg2_diff);
				log_msg("D s %0.4f e %0.4f r %0.4f d %0.4f p %0.4f                     g1 %0.4f  g2 %0.4f",
						ths_diff*180/M_PI,the_diff*180/M_PI,thr_diff*180/M_PI,d_diff,
						thp_diff*180/M_PI,thg1_diff*180/M_PI,thg2_diff*180/M_PI);

			}

			if (i==1 && _curr_rl == 3) {
				//printf("ik ok! %d\n",_ik_counter);
			}
		}
	}

	if (opts_valid[0]) {
		return 1;
	} else if (opts_valid[1]) {
		bool ENABLE_PARTIAL_INVALID_IK_PRINTING = false;
		if (ENABLE_PARTIAL_INVALID_IK_PRINTING && (_curr_rl == 3 || print) && !(DISABLE_ALL_PRINTING)) {
			printf("ik %d    ok   **2** s %1.4f %1.4f\te %1.4f %1.4f\tr %1.4f %1.4f\n",
					armId,
					ths_act[0] RAD2DEG,validity2[0][0],
					the_act[0] RAD2DEG,validity2[0][1],
					thr_act[0] RAD2DEG,validity2[0][2]);
			/*
			printf("%7d          d %0.4f  \tp %0.4f  \ty %0.4f\n",_ik_counter,
								d_act,thp_act RAD2DEG,thy_act RAD2DEG);
			printf("x (%f, %f, %f)  z (%f, %f, %f) %f\n",xx,xy,xz,zx,zy,zz,cthe);
			printf("norms %f %f\n",x_roll_in_world.length(),z_roll_in_world.length());
			printf("(zy + kc12*kc23): %f (%f, %f)\n",(zy + kc12*kc23),zy, kc12*kc23);
			printf("(ks12*ks23): %f\n",(ks12*ks23));
			 */
		}
		return 1;
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
			set_joints_with_limits1(mech,d_act,thp_act,g1_act,g2_act);
			set_joints_with_limits2(mech,ths_act[0],the_act[0],thr_act[0]);
		} else if (valid_dist[1] < maxValidDist) {
			printf("setting joints to ik soln 2\n");
			set_joints_with_limits1(mech,d_act,thp_act,g1_act,g2_act);
			set_joints_with_limits2(mech,ths_act[1],the_act[1],thr_act[1]);
		}

		if ((_curr_rl == 3 || print) && !(DISABLE_ALL_PRINTING)) {
			printf("ik %d invalid **2** s %1.4f % 2.1f\te %1.4f % 2.1f\tr %1.4f % 2.1f\n",
					armId,
					ths_act[0] RAD2DEG,validity2[0][0] RAD2DEG,
					the_act[0] RAD2DEG,validity2[0][1] RAD2DEG,
					thr_act[0] RAD2DEG,validity2[0][2] RAD2DEG);
			printf("                   s %1.4f % 2.1f\te %1.4f % 2.1f\tr %1.4f % 2.1f\n",
					ths_act[1] RAD2DEG,validity2[1][0] RAD2DEG,
					the_act[1] RAD2DEG,validity2[1][1] RAD2DEG,
					thr_act[1] RAD2DEG,validity2[1][2] RAD2DEG);
		}
		return 0;
	}
}

int set_joints_with_limits1(mechanism* mech, float d_act, float thp_act, float g1_act, float g2_act) {
	if (_ik_counter % PRINT_EVERY == 0) {
		log_msg("setting joints 1");
	}
	mech->joint[Z_INS].jpos_d    = d_act;
	mech->joint[WRIST].jpos_d    = thp_act; //WRIST_HOME_ANGLE; int WARNING_WRIST_NOT_SET;
	mech->joint[GRASP1].jpos_d   = g1_act; //GRASP1_HOME_ANGLE; int WARNING_GRASP1_NOT_SET;
	mech->joint[GRASP2].jpos_d   = g2_act; //GRASP2_HOME_ANGLE; int WARNING_GRASP2_NOT_SET;

	int limits=0;

	if (mech->joint[Z_INS].jpos_d  < Z_INS_MIN_LIMIT) {
		limits++;
		log_msg("insertion min limit");
		mech->joint[Z_INS].jpos_d = Z_INS_MIN_LIMIT;
	} else if (mech->joint[Z_INS].jpos_d  > Z_INS_MAX_LIMIT) {
		limits++;
		log_msg("insertion max limit");
		mech->joint[Z_INS].jpos_d = Z_INS_MAX_LIMIT;
	}

	if (mech->joint[WRIST].jpos_d  < TOOL_WRIST_MIN_LIMIT) {
		log_msg("wrist min limit");
		limits++;
		mech->joint[WRIST].jpos_d = TOOL_WRIST_MIN_LIMIT;
	} else if (mech->joint[WRIST].jpos_d  > TOOL_WRIST_MAX_LIMIT) {
		limits++;
		log_msg("wrist max limit");
		mech->joint[WRIST].jpos_d = TOOL_WRIST_MAX_LIMIT;
	}
	//TODO: limit this elsewhere
	if (fabs(mech->joint[WRIST].jpos_d - mech->joint[WRIST].jpos) > 10 DEG2RAD) {
		if (mech->joint[WRIST].jpos_d > mech->joint[WRIST].jpos) {
			mech->joint[WRIST].jpos_d = mech->joint[WRIST].jpos + 10 DEG2RAD;
		} else {
			mech->joint[WRIST].jpos_d = mech->joint[WRIST].jpos - 10 DEG2RAD;
		}
	}

	if (mech->joint[GRASP1].jpos_d  < TOOL_GRASP1_MIN_LIMIT) {
		limits++;
		log_msg("grasp1 min limit");
		mech->joint[GRASP1].jpos_d = TOOL_GRASP1_MIN_LIMIT;
	} else if (mech->joint[GRASP1].jpos_d  > TOOL_GRASP1_MAX_LIMIT) {
		limits++;
		log_msg("grasp1 max limit");
		mech->joint[GRASP1].jpos_d = TOOL_GRASP1_MAX_LIMIT;
	}

	if (mech->joint[GRASP2].jpos_d  < TOOL_GRASP2_MIN_LIMIT) {
		log_msg("grasp2 min limit");
		limits++;
		mech->joint[GRASP2].jpos_d = TOOL_GRASP2_MIN_LIMIT;
	} else if (mech->joint[GRASP2].jpos_d  > TOOL_GRASP2_MAX_LIMIT) {
		limits++;
		log_msg("grasp2 max limit");
		mech->joint[GRASP2].jpos_d = TOOL_GRASP2_MAX_LIMIT;
	}

	return limits;
}

int set_joints_with_limits2(mechanism* mech, float ths_act, float the_act, float thr_act) {
	if (_ik_counter % PRINT_EVERY == 0) {
		log_msg("setting joints 2");
	}
	mech->joint[SHOULDER].jpos_d = ths_act;
	mech->joint[ELBOW].jpos_d    = the_act;
	//mech->joint[TOOL_ROT].jpos_d = fix_angle(thr_act + M_PI); //TOOL_ROT_HOME_ANGLE; int WARNING_ROT_NOT_SET;
	mech->joint[TOOL_ROT].jpos_d = thr_act; //TOOL_ROT_HOME_ANGLE; int WARNING_ROT_NOT_SET;

	int limits = 0;
	if (mech->joint[SHOULDER].jpos_d < SHOULDER_MIN_LIMIT) {
		limits++;
		log_msg("shoulder min limit");
		mech->joint[SHOULDER].jpos_d = SHOULDER_MIN_LIMIT;
	} else if (mech->joint[SHOULDER].jpos_d > SHOULDER_MAX_LIMIT) {
		limits++;
		log_msg("shoulder max limit");
		mech->joint[SHOULDER].jpos_d = SHOULDER_MAX_LIMIT;
	}

	if (mech->joint[ELBOW].jpos_d < ELBOW_MIN_LIMIT) {
		limits++;
		log_msg("elbow min limit");
		mech->joint[ELBOW].jpos_d = ELBOW_MIN_LIMIT;
	} else if (mech->joint[ELBOW].jpos_d > ELBOW_MAX_LIMIT) {
		limits++;
		log_msg("elbow max limit");
		mech->joint[ELBOW].jpos_d = ELBOW_MAX_LIMIT;
	}

	if (mech->joint[TOOL_ROT].jpos_d < TOOL_ROLL_MIN_LIMIT) {
		limits++;
		log_msg("roll % 3.1fdeg under min limit",mech->joint[TOOL_ROT].jpos_d RAD2DEG);
		mech->joint[TOOL_ROT].jpos_d = TOOL_ROLL_MIN_LIMIT;
	} else if (mech->joint[TOOL_ROT].jpos_d > TOOL_ROLL_MAX_LIMIT) {
		limits++;
		log_msg("roll % 3.1fdeg over max limit",mech->joint[TOOL_ROT].jpos_d RAD2DEG);
		mech->joint[TOOL_ROT].jpos_d = TOOL_ROLL_MAX_LIMIT;
	}

	//TODO: limit this elsewhere
	if (fabs(mech->joint[TOOL_ROT].jpos_d - mech->joint[TOOL_ROT].jpos) > 10 DEG2RAD) {
		if (mech->joint[TOOL_ROT].jpos_d > mech->joint[TOOL_ROT].jpos) {
			mech->joint[TOOL_ROT].jpos_d = mech->joint[TOOL_ROT].jpos + 10 DEG2RAD;
		} else {
			mech->joint[TOOL_ROT].jpos_d = mech->joint[TOOL_ROT].jpos - 10 DEG2RAD;
		}
	}

	return limits;
}

bool check_joint_limits1_new(float d_act, float thp_act, float g1_act, float g2_act,int validity[4]) {
	validity[0] = 0;
	validity[1] = 0;
	validity[2] = 0;
	validity[3] = 0;

	bool any_nan = false;
	if (d_act != d_act) { if (_ik_counter % PRINT_EVERY == 0) { log_msg("d_act is nan"); } any_nan = true; };
	if (thp_act != thp_act) { if (_ik_counter % PRINT_EVERY == 0) { log_msg("thp_act is nan"); } any_nan = true; };
	if (g1_act != g1_act) { if (_ik_counter % PRINT_EVERY == 0) { log_msg("g1_act is nan"); } any_nan = true; };
	if (g2_act != g2_act) { if (_ik_counter % PRINT_EVERY == 0) { log_msg("g2_act is nan"); } any_nan = true; };
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

bool check_joint_limits2_new(float ths_act, float the_act, float thr_act,float validity[3]) {
	validity[0] = 0;
	validity[1] = 0;
	validity[2] = 0;
	bool any_nan = false;
	if (ths_act != ths_act) { if (_ik_counter % PRINT_EVERY == 0) { log_msg("ths_act is nan"); } any_nan = true; };
	if (the_act != the_act) { if (_ik_counter % PRINT_EVERY == 0) { log_msg("the_act is nan"); } any_nan = true; };
	if (thr_act != thr_act) { if (_ik_counter % PRINT_EVERY == 0) { log_msg("thr_act is nan"); } any_nan = true; };
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



/**

    Inverse kinematics have been reformulated for UW Raven II Build
    Lee White
    Hawkeye King

    1/8/2012

* Inverse kinematics equations for Raven_II robot
*  Preconditions:
*      pos_d stores the desired end effector position
*      ori_d stores the desired end effector orientation
*  Postcondition
*      jpos_d stores the desired joint angles
*
*  Previously the function was defined like this, but not anymore:
*   Input pos_d, ori_d
*   Output jpos_d joint angles
*/

int invMechKin_pos(struct mechanism *mech);
int invMechKin_ori(struct mechanism *mech);

int invMechKin(struct mechanism *mech)
{
    if (invMechKin_pos(mech) > 0)
    {
        // enforce joint limits on position DoF
        check_joint_limits1(mech);
        invMechKin_ori(mech);
        // enforce joint limits on tool DoF
        check_joint_limits2(mech);
        return 1;
    }
    else
    {
        log_msg("no_ori");
        return 0;
    }
}

int invMechKin_pos(struct mechanism *mech)
{
    struct position    *pos_d = &(mech->pos_d);

    // output angles for robot
    float j1; //Shoulder
    float j2; //Elbow
    float d3; //Insertion

	// desired tip position
	float Px = (pos_d->x) / MICRON_PER_M;
	float Py = (pos_d->y) / MICRON_PER_M;
	float Pz = (pos_d->z) / MICRON_PER_M;

	/// Solve for d3 (insertion) - always negative
	d3 = - sqrt(SQR(Px) + SQR(Py) + SQR(Pz));

    // Avoid mechanism singularity at d3=0
    const float min_insertion=0.001;
	if (fabs(d3) < min_insertion)
	{
	    return 0;
	}

	/// ** Solve for j2 (Elbow) **
	float cbe=1, sbe=0, cal=1, sal=0;
	float c2,s2;
    if (mech->type == GOLD_ARM)
    {
        cbe = cos(M_PI - A23); 	// cos(Pi - beta)
        sbe = sin(M_PI - A23);  // sin(Pi - beta)
        cal = cos(-A12);        // cos(-alpha)
        sal = sin (-A12);       // sin(-alpha)
        c2 = (Pz - cbe * cal * d3)/( - sbe *  d3 * sal);
    }
    else
    {
        cbe = cos(A23);  // cos(beta)
        sbe = sin(A23);  // sin(beta)
        cal = cos(A12);  // cos(alpha)
        sal = sin(A12);  // sin(alpha)
        c2 = (Pz + cbe * cal * d3)/( sbe *  d3 * sal);
    }

    if (c2>1) c2=1;
    else if (c2<-1) c2=-1;

	// s2 is always positive, since 0<j2<180
	// This resolves multiple solutions
	s2 = sqrt(1-(c2*c2));
	j2 = atan2f(s2,c2);


	/// ** Solve for j1 (Shoulder) **
	float s1,c1;
    float a,b;
    if (mech->type == GOLD_ARM)
    {
        a = sbe*d3*s2;
        b = sbe*d3*c2*cal + cbe*d3*sal;
        c1 = (Px*a - Py*b) / (a*a + b*b);
        s1 = (Px*b + Py*a) / (a*a + b*b);
        s1 = fabs(s1); // s1 always positive, since 0<j1<90; resolves multiple sol'n
        c1 = fabs(c1); // c1 always positive, since 0<j1<90;
        j1 = atan2f ( s1, c1 ) - base_tilt;
    }
    else
    {
        s2=-s2;
        a = sbe*d3*s2;
        b = sbe*d3*c2*cal + cbe*d3*sal;

        c1 = (Px*a - Py*b) / (a*a + b*b);
        s1 = (Py*a + Px*b) / (-(a*a) - (b*b));
        s1 = fabs(s1); // s1 always positive, since 0<j1<90;
        c1 = fabs(c1); // c1 always positive, since 0<j1<90;
        j1 = atan2f ( s1, c1 ) - base_tilt;
    }

	// Now have solved for th1, th2, d3 ...
    mech->joint[SHOULDER].jpos_d = j1;
    mech->joint[ELBOW].jpos_d    = j2;
    mech->joint[Z_INS].jpos_d    = d3;

	return 1;   // actually has a solution
}

/**
*   Calculate the tool DOF angles from the desired orientation
*
*   Assumes that the shoulder, elbow, z have been calculated from desired position.
*
*   The rotation from base frame to tool frame is premultiplied to the
*   desired transform.  The resulting rotation is used to calculate the desired
*   tool angles.
*
*   Desired transform TD_0_6:  TD_3_0 * TD_0_6 = TD_3_6
*   TD_3_6 is then the desired transform from tool frame to tip.
*/
int invMechKin_ori(struct mechanism *mech)
{
    struct orientation   *ori_d = &(mech->ori_d);

    double j1 = mech->joint[SHOULDER].jpos_d;
	double j2 = mech->joint[ELBOW].jpos_d;
	double j3 = mech->joint[Z_INS].jpos_d;

    btMatrix3x3 xf;
//    (ori_d->R[0][0], ori_d->R[0][1], ori_d->R[0][2],
//                       ori_d->R[1][0], ori_d->R[1][1], ori_d->R[1][2],
//                       ori_d->R[2][0], ori_d->R[2][1], ori_d->R[2][2]);
	// copy R matrix
	// TODO: use above commented code.
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			xf[i][j] = ori_d->R[i][j];

    // variable DH Parameters for gold and green arms
    double go_dh_d[3]  = {0,              0,      j3};
    double go_dh_th[3] = {j1 + base_tilt, j2,     0};

    double gr_dh_d[3]  = {0,              0,      j3};
    double gr_dh_th[3] = {j1 + base_tilt, j2,     0};

    double dh_al[6];
    double dh_a[6];
    double dh_d[3];
    double dh_th[3];

    if (mech->type == GOLD_ARM_SERIAL)
    {
        memcpy(dh_al, go_dh_al, 6*sizeof(double));
        memcpy(dh_a,  go_dh_a,  6*sizeof(double));
        memcpy(dh_d,  go_dh_d,  3*sizeof(double));
        memcpy(dh_th, go_dh_th,  3*sizeof(double));
    }
    else if (mech->type == GREEN_ARM_SERIAL)
    {
        memcpy(dh_al, gr_dh_al, 6*sizeof(double));
        memcpy(dh_a,  gr_dh_a,  6*sizeof(double));
        memcpy(dh_d,  gr_dh_d,  3*sizeof(double));
        memcpy(dh_th, gr_dh_th, 3*sizeof(double));
    }

    // Generate transformation matrices from DH parameters
    btMatrix3x3 temp;
    btMatrix3x3 Rot;
    Rot.setIdentity();

    btScalar cti, sti;  // cos(thetai), sin(thetai)
    btScalar cai, sai;  // cos(alphai), sin(alphai)

    /// Calculate the transform from base frame to tool frame
    for (int i=0;i<3;i++)
    {
        cti = cos(dh_th[i]);
        sti = sin(dh_th[i]);
        cai = cos(dh_al[i]);
        sai = sin(dh_al[i]);

        temp[0][0] = cti;
        temp[0][1] = -sti;
        temp[0][2] = 0;

        temp[1][0] = sti * cai;
        temp[1][1] = cti * cai;
        temp[1][2] = -sai;

        temp[2][0] = sti * sai;
        temp[2][1] = cti * sai;
        temp[2][2] = cai;

        Rot = Rot * temp;
    }

    /// Get the desired transform in tool frame
    /// Premultiply by inverse transform from base frame to tool frame
    xf = Rot.inverse() * xf;

//	for (int i = 0; i < 3; i++)
//		for (int j = 0; j < 3; j++)
//			mech->ori_d.R[i][j] = xf[i][j];

    float j4[2];
    float j5[2];
    float j6[2];
    float j4_check, min_j4;;

    /// ** Solve j4 (tool roll) **
    if (mech->type == GOLD_ARM)
    {
        j4[0] = atan2f( -xf[1][2], -xf[0][2] );
        j4[1] = atan2f( xf[1][2], xf[0][2] );
        j4_check = mech->joint[TOOL_ROT].jpos - M_PI/2;
    }
    else
    {
        j4[0] = atan2f( -xf[1][2], xf[0][2] );
        j4[1] = atan2f( xf[1][2], -xf[0][2] );
        j4_check = mech->joint[TOOL_ROT].jpos - M_PI/2;
    }

    // select between multiple solutions by the smallest difference from current angle.
    min_j4 = j4[0];
    int whichcase=0;
    if ( fabsf(j4_check - j4[1]) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[1];
        whichcase=1;
    }
    //check rollover conditions
    if ( fabsf(j4_check - (j4[1]+2*M_PI)) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[1]+2*M_PI;
        whichcase=2;
    }
    if ( fabsf(j4_check - (j4[0]+2*M_PI)) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[0]+2*M_PI;
        whichcase=3;
    }
    if ( fabsf(j4_check - (j4[1]-2*M_PI)) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[1]-2*M_PI;
        whichcase=4;
    }
    if ( fabsf(j4_check - (j4[0]-2*M_PI)) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[0]-2*M_PI;
        whichcase=5;
    }
//
//    if (mech->type == GREEN_ARM)
//    {
//        log_msg("cs:%d j4c:%0.3f\t 0:%0.3f\t 1:%0.3f\t 2:%0.3f\t 3:%0.3f\t 4:%0.3f\t 5:%0.3f",
//            whichcase, j4_check,
//            j4[0],j4[1],
//            j4[1]+2*M_PI,
//            j4[0]+2*M_PI,
//            j4[1]-2*M_PI,
//            j4[0]-2*M_PI);
//    }

    if (mech->type == GOLD_ARM)
        mech->joint[TOOL_ROT].jpos_d = min_j4 + M_PI/2;
    else
        mech->joint[TOOL_ROT].jpos_d = -min_j4 - M_PI/2;

    ///  ** Solve j5 (wrist) **
    float c4 = cosf(min_j4);
    float s5 = xf[0][2] / c4;
    float c5 = xf[2][2];
    j5[0] = atan2f(-s5,-c5);

    if (mech->type == GOLD_ARM)
        mech->joint[WRIST].jpos_d  = j5[0] - M_PI/2;
    else
        mech->joint[WRIST].jpos_d  = -j5[0] + M_PI/2;

    /// ** Solve j6 (avg grasp) **
//    float s5_sign = (s5 < 0) ? -1:1;
    float s6 = xf[2][1]/-s5;
    float c6 = xf[2][0]/s5;
    j6[0] = atan2f(s6, c6);

    float grasp = ((float)mech->ori_d.grasp)/1000.0f;
    //log_msg("%d:grasp:%0.3f", mech->type, grasp);


    if (mech->type == GOLD_ARM)
    {
        mech->joint[GRASP1].jpos_d = grasp/2;
        mech->joint[GRASP2].jpos_d = grasp/2;
    }
    else
    {
        mech->joint[GRASP1].jpos_d = grasp/2;
        mech->joint[GRASP2].jpos_d = grasp/2;
    }





//    float openangle;
//    if (mech->type == GOLD_ARM)
//        openangle = 45 DEG2RAD; //ori_d->grasp/1000;
//    else
//        openangle = -45 DEG2RAD;//ori_d->grasp/1000;
//
////    log_msg("g(%d):%0.3f, \t (%0.3f)", mech->type, openangle, (float)mech->ori_d.grasp/1000);
//    if (mech->type == GOLD_ARM)
//    {
//        mech->joint[GRASP1].jpos_d = j6[0]+openangle/2;
//        mech->joint[GRASP2].jpos_d = j6[0]-openangle/2;
//    }
//    else
//    {
//        mech->joint[GRASP1].jpos_d = j6[0]-openangle/2;
//        mech->joint[GRASP2].jpos_d = j6[0]+openangle/2;
//    }

    mech->joint[TOOL_ROT].jpos_d = 0;
    mech->joint[WRIST].jpos_d = 0;

    return 1;
}







/**
* Check for joint.angle_d exceeding joint angles.
*     --> position
*/
int check_joint_limits1(struct mechanism *_mech)
{


    int limits = 0;
	if (_mech->joint[SHOULDER].jpos_d < SHOULDER_MIN_LIMIT)
	{
	    limits++;
	    log_msg("shoulder limit");
		_mech->joint[SHOULDER].jpos_d = SHOULDER_MIN_LIMIT;
	}
	else if (_mech->joint[SHOULDER].jpos_d > SHOULDER_MAX_LIMIT)
	{
	    limits++;
	    log_msg("shoulder limit");
		_mech->joint[SHOULDER].jpos_d = SHOULDER_MAX_LIMIT;
	}
	if (_mech->joint[ELBOW].jpos_d < ELBOW_MIN_LIMIT)
	{
	    limits++;
        log_msg("elbow limit");
		_mech->joint[ELBOW].jpos_d = ELBOW_MIN_LIMIT;
	}
	else if (_mech->joint[ELBOW].jpos_d > ELBOW_MAX_LIMIT)
	{
 	    limits++;
        log_msg("elbow limit");
		_mech->joint[ELBOW].jpos_d = ELBOW_MAX_LIMIT;
	}
	if (_mech->joint[Z_INS].jpos_d < Z_INS_MIN_LIMIT)
	{
	    limits++;
        log_msg("Zins limit");
		_mech->joint[Z_INS].jpos_d = Z_INS_MIN_LIMIT;
	}
	else if (_mech->joint[Z_INS].jpos_d > Z_INS_MAX_LIMIT)
	{
	    limits++;
        log_msg("Zins limit");
		_mech->joint[Z_INS].jpos_d = Z_INS_MAX_LIMIT;
	}

	return limits;
}
/**
* Check for joint.angle_d exceeding joint angles.
*     --> orientation
*/
int check_joint_limits2(struct mechanism *_mech)
{
    int limits=0;

	if (_mech->joint[TOOL_ROT].jpos_d  < TOOL_ROLL_MIN_LIMIT)
	{
	    limits++;
        log_msg("roll limit");
		_mech->joint[TOOL_ROT].jpos_d = TOOL_ROLL_MIN_LIMIT;
	}
	else if (_mech->joint[TOOL_ROT].jpos_d  > TOOL_ROLL_MAX_LIMIT)
	{
	    limits++;
        log_msg("roll limit");
		_mech->joint[TOOL_ROT].jpos_d = TOOL_ROLL_MAX_LIMIT;
	}
	if (_mech->joint[WRIST].jpos_d  < TOOL_WRIST_MIN_LIMIT)
	{
        log_msg("wrist limit");
	    limits++;
		_mech->joint[WRIST].jpos_d = TOOL_WRIST_MIN_LIMIT;
	}
	else if (_mech->joint[WRIST].jpos_d  > TOOL_WRIST_MAX_LIMIT)
	{
	    limits++;
        log_msg("wrist limit");
		_mech->joint[WRIST].jpos_d = TOOL_WRIST_MAX_LIMIT;
	}

	if (_mech->joint[GRASP1].jpos_d  < TOOL_GRASP1_MIN_LIMIT)
	{
        log_msg("grasp1 limit");
	    limits++;
		_mech->joint[GRASP1].jpos_d = TOOL_GRASP1_MIN_LIMIT;
	}
	else if (_mech->joint[GRASP1].jpos_d  > TOOL_GRASP1_MAX_LIMIT)
	{
	    limits++;
        log_msg("grasp1 limit");
		_mech->joint[GRASP1].jpos_d = TOOL_GRASP1_MAX_LIMIT;
	}

	if (_mech->joint[GRASP2].jpos_d  < TOOL_GRASP2_MIN_LIMIT)
	{
        log_msg("grasp2 limit");
	    limits++;
		_mech->joint[GRASP2].jpos_d = TOOL_GRASP2_MIN_LIMIT;
	}
	else if (_mech->joint[GRASP2].jpos_d  > TOOL_GRASP2_MAX_LIMIT)
	{
	    limits++;
        log_msg("grasp2 limit");
		_mech->joint[GRASP2].jpos_d = TOOL_GRASP2_MAX_LIMIT;
	}

	return limits;
}
