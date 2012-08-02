/*
 * kinematics_defines.h
 *
 *  Created on: Jun 19, 2012
 *      Author: biorobotics
 */

#ifndef KINEMATICS_DEFINES_H_
#define KINEMATICS_DEFINES_H_

#include "defines.h"
#include "utils.h"

#define _A5 0.0087  // 8.7mm
const double base_tilt = 0 DEG2RAD;
#define BASE_TILT 0 DEG2RAD

#define SHOULDER_OFFSET_GOLD atan(0.3471/0.9014) //from original URDF
#define TOOL_ROT_OFFSET_GOLD -M_PI_2 //M_PI_4
#define WRIST_OFFSET_GOLD M_PI/6.

//TODO: these are probably incorrect
#define SHOULDER_OFFSET_GREEN atan(0.3471/0.9014)//from original URDF
#define TOOL_ROT_OFFSET_GREEN M_PI_4
#define WRIST_OFFSET_GREEN 0

#define THETA_12 -A12
#define THETA_23 -A23
#define DW 0.012

#define Z(theta,d) btTransform(btMatrix3x3(cos(theta),-sin(theta),0, sin(theta),cos(theta),0, 0,0,1),btVector3(0,0,d))
#define X(alpha,a) btTransform(btMatrix3x3(1,0,0, 0,cos(alpha),-sin(alpha), 0,sin(alpha),cos(alpha)),btVector3(a,0,0))

const btTransform Tw2b(btMatrix3x3(0,-1,0, 0,0,-1, 1,0,0));
#define Zs(ths) Z(ths,0)
const btTransform Xu = X(THETA_12,0);
#define Ze(the) Z(the,0)
const btTransform Xf = X(THETA_23,0);
#define Zr(thr) Z(thr,0)
#define Zi(d) Z(0,d)
const btTransform Xip(btMatrix3x3(0,-1,0, 0,0,-1, 1,0,0));
#define Zp(thp) Z(thp,0)
const btTransform Xpy(btMatrix3x3(1,0,0, 0,0,-1, 0,1,0),btVector3(DW,0,0));
#define Zy(thy) Z(thy,0)
const btTransform Tg(btMatrix3x3::getIdentity());

#define Tikw2g(ths,the,thr,d,thp,thy) btTransform(Tw2b * Zs(ths) * Xu * Ze(the) * Xf * Zr(thr) * Zi(d) * Xip * Zp(thp) * Xpy * Zy(thy) * Tg)
#define Tw2g(armId,ths,the,thr,d,thp,thy) btTransform(actual_world_to_ik_world(armId) * Tikw2g(ths,the,thr,d,thp,thy))

#define Tw2s(armId,ths) btTransform(actual_world_to_ik_world(armId) * Tw2b * Zs(ths))
#define Tw2e(armId,ths,the) btTransform(actual_world_to_ik_world(armId) * Tw2b * Zs(ths) * Xu * Ze(the))
#define Tw2r(armId,ths,the,thr) btTransform(actual_world_to_ik_world(armId) * Tw2b * Zs(ths) * Xu * Ze(the) * Xf * Zr(thr))
#define Tw2i(armId,ths,the,thr,d) btTransform(actual_world_to_ik_world(armId) * Tw2b * Zs(ths) * Xu * Ze(the) * Xf * Zr(thr) * Zi(d))
#define Tw2p(armId,ths,the,thr,d,thp) btTransform(actual_world_to_ik_world(armId) * Tw2b * Zs(ths) * Xu * Ze(the) * Xf * Zr(thr) * Zi(d) * Xip * Zp(thp))
#define Tw2y(armId,ths,the,thr,d,thp,thy) btTransform(actual_world_to_ik_world(armId) * Tw2b * Zs(ths) * Xu * Ze(the) * Xf * Zr(thr) * Zi(d) * Xip * Zp(thp) * Xpy * Zy(thy))

inline btTransform actual_world_to_ik_world(int armId) {
	if (armId == GOLD_ARM_ID) {
		return btTransform(btMatrix3x3(0,1,0, -1,0,0, 0,0,1));
	} else {
		printf("GREEN ARM KINEMATICS NOT IMPLEMENTED\n");
		return btTransform(btMatrix3x3(0,-1,0, 1,0,0, 0,0,1),btVector3(-.2, 0, 0));
	}
}

inline btTransform ik_world_to_actual_world(int armId) {
	return actual_world_to_ik_world(armId).inverse();
}





#endif /* KINEMATICS_DEFINES_H_ */
