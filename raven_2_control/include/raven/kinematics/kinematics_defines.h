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

const btTransform TOOL_POSE_AXES_TRANSFORM(btMatrix3x3(1,0,0,  0,-1,0,  0,0,-1));

#define SHOULDER_OFFSET_GOLD atan(0.3471/0.9014) //from original URDF
#define TOOL_ROT_OFFSET_GOLD -M_PI_2 //-M_PI_2 //M_PI_4
#define WRIST_OFFSET_GOLD 0 //M_PI/6. //Fudge factor

//TODO: these are probably incorrect
#define SHOULDER_OFFSET_GREEN atan(0.3471/0.9014)//from original URDF
#define TOOL_ROT_OFFSET_GREEN -M_PI_2
#define WRIST_OFFSET_GREEN 0

#define THETA_12 -A12
#define THETA_23 -A23
#define DW 0.012

#define Z(theta,d) btTransform(btMatrix3x3(cos(double(theta)),-sin(double(theta)),0, sin(theta),cos(double(theta)),0, 0,0,1),btVector3(0,0,d))
#define X(alpha,a) btTransform(btMatrix3x3(1,0,0, 0,cos(double(alpha)),-sin(double(alpha)), 0,sin(double(alpha)),cos(double(alpha))),btVector3(a,0,0))

const btTransform Tw2b(btMatrix3x3(0,-1,0, 0,0,-1, 1,0,0));

inline float THS_TO_IK(  int armId,float ths) { return (armId == GOLD_ARM_ID ? fix_angle(ths + SHOULDER_OFFSET_GOLD) : fix_angle(M_PI - ths - SHOULDER_OFFSET_GREEN)); }
inline float THS_FROM_IK(int armId,float ths) { return (armId == GOLD_ARM_ID ? fix_angle(ths - SHOULDER_OFFSET_GOLD) : fix_angle(M_PI - ths - SHOULDER_OFFSET_GREEN)); }
#define Zs(ths) Z(ths,0)

const btTransform Xu = X(THETA_12,0);

inline float THE_TO_IK(  int armId,float the) { return (armId == GOLD_ARM_ID ? the : -the); }
inline float THE_FROM_IK(int armId,float the) { return (armId == GOLD_ARM_ID ? the : -the); }
#define Ze(the) Z(the,0)

const btTransform Xf = X(THETA_23,0);

inline float THR_TO_IK(  int armId,float thr) { return (armId == GOLD_ARM_ID ? fix_angle(-thr + TOOL_ROT_OFFSET_GOLD) : fix_angle(thr + TOOL_ROT_OFFSET_GREEN /*- 0.37732519171346124*/)); }
inline float THR_FROM_IK(int armId,float thr) { return (armId == GOLD_ARM_ID ? fix_angle(-thr + TOOL_ROT_OFFSET_GOLD) : fix_angle(thr - TOOL_ROT_OFFSET_GREEN /*+ 0.37732519171346124*/)); }
#define Zr(thr) Z(thr,0)

inline float D_TO_IK(  int armId,float d) { return -d; }
inline float D_FROM_IK(int armId,float d) { return -d; }
#define Zi(d) Z(0,d)

const btTransform Xip(btMatrix3x3(0,-1,0, 0,0,-1, 1,0,0));

inline float THP_TO_IK(  int armId,float thp) { return (armId == GOLD_ARM_ID ? fix_angle(-thp + WRIST_OFFSET_GOLD) : thp); }
inline float THP_FROM_IK(int armId,float thp) { return (armId == GOLD_ARM_ID ? fix_angle(-thp - WRIST_OFFSET_GOLD) : thp); }
#define Zp(thp) Z(thp,0)

const btTransform Xpy(btMatrix3x3(1,0,0, 0,0,-1, 0,1,0),btVector3(DW,0,0));

inline int MECH_GRASP_FROM_MECH_FINGERS(int armId,float g1, float g2) { return 1000.*(armId == GOLD_ARM_ID ? (g2 + g1) : (g2 + g1)); }
inline float GRASP_TO_IK(int armId,int grasp) { return (armId == GOLD_ARM_ID ? grasp : -grasp) / 1000.; }

inline float FINGER1_FROM_IK(int armId,float thy, float grasp) { return (armId == GOLD_ARM_ID ?  thy + grasp/2 : -( thy + grasp/2)); }
inline float FINGER2_FROM_IK(int armId,float thy, float grasp) { return (armId == GOLD_ARM_ID ? -thy + grasp/2 : -(-thy + grasp/2)); }

inline float THY_MECH_FROM_FINGERS(  int armId,float g1, float g2) { return (armId == GOLD_ARM_ID ? (g2 - g1) / 2 : -(g2 - g1) / 2); }
inline float FINGER1_FROM_THY_AND_GRASP_MECH(int armId,float thy, int grasp) { return (armId == GOLD_ARM_ID ? -thy + (grasp * 1000. / 2) :  thy + (-grasp * 1000. / 2)); }
inline float FINGER2_FROM_THY_AND_GRASP_MECH(int armId,float thy, int grasp) { return (armId == GOLD_ARM_ID ?  thy + (grasp * 1000. / 2) : -thy + (-grasp * 1000. / 2)); }


inline float THY_TO_IK_FROM_FINGERS( int armId,float g1, float g2) { return (armId == GOLD_ARM_ID ? -1 : -1) * THY_MECH_FROM_FINGERS(armId,g1,g2); }
inline float THY_FROM_IK(int armId, float thy, float grasp) { return (armId == GOLD_ARM_ID ? 1 : 1) * thy; }

#define Zy(thy) Z(thy,0)

const btTransform Tg(btMatrix3x3::getIdentity(),btVector3(0.01,0,0));

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
		//printf("GREEN ARM KINEMATICS NOT IMPLEMENTED\n");
		//return btTransform(btMatrix3x3(0,-1,0,1,0,0, 0,0,1),btVector3(-.2, 0, 0));-0.14858,0.002,0
		return btTransform(btMatrix3x3(0,-1,0,1,0,0, 0,0,1),btVector3(-0.14858,0.002,0));
		//return btTransform(btMatrix3x3(0,1,0, -1,0,0, 0,0,1)) * GREEN_ARM_BASE_POSE;
	}
}

inline btTransform ik_world_to_actual_world(int armId) {
	return actual_world_to_ik_world(armId).inverse();
}


#define __cable_coupling_inv__(val) (1./(val))

//#define FINAL_RATIO_4 __cable_coupling_inv__(120.0 / 180.0)
//#define FINAL_RATIO_5 __cable_coupling_inv__(200.0 / 180.0)
//#define FINAL_RATIO_6 __cable_coupling_inv__(100.0 /  90.0)
//#define FINAL_RATIO_7 __cable_coupling_inv__(100.0 /  90.0)

#define FINAL_RATIO_4 ( CAPSTAN_RADIUS_ROLL / TOOL_PULLEY_RADIUS_ROLL )
#define FINAL_RATIO_5 ( CAPSTAN_RADIUS_PITCH / TOOL_PULLEY_RADIUS_PITCH )

//#define GRASP_FINAL_RATIO_ADJUSTMENT ( 1. )
//#define GRASP_FINAL_RATIO_ADJUSTMENT ( 200. / 260. )
#define GRASP_FINAL_RATIO_ADJUSTMENT ( (99. * 2)/(99. + 120.) )
#define FINAL_RATIO_6 ( CAPSTAN_RADIUS_YAW / TOOL_PULLEY_RADIUS_YAW * GRASP_FINAL_RATIO_ADJUSTMENT )
#define FINAL_RATIO_7 ( CAPSTAN_RADIUS_YAW / TOOL_PULLEY_RADIUS_YAW * GRASP_FINAL_RATIO_ADJUSTMENT )

//#define CABLE_COUPLING_FINAL_RATIO ( 1. )
#define CABLE_COUPLING_FINAL_RATIO ( 0.97165644414807584 ) //based on calculated

#define CABLE_COUPLING_11 ( __cable_coupling_inv__(GEAR_BOX_GP42_TR) * CAPSTAN_RADIUS_GP42_1050 / PARTIAL_PULLEY_LINK1_RADIUS )
#define CABLE_COUPLING_22 ( __cable_coupling_inv__(GEAR_BOX_GP42_TR) * CAPSTAN_RADIUS_GP42_1050 / CAPSTAN_LINK2_LARGE_RADIUS * CAPSTAN_LINK2_SMALL_RADIUS / PARTIAL_PULLEY_LINK2_RADIUS )
#define CABLE_COUPLING_33 ( __cable_coupling_inv__(GEAR_BOX_GP42_TR) * CAPSTAN_RADIUS_GP42_1024 / 1000.0 * CABLE_COUPLING_FINAL_RATIO )
#define CABLE_COUPLING_44 ( __cable_coupling_inv__(GEAR_BOX_GP32_TR) * CAPSTAN_RADIUS_GP32 / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_4)
#define CABLE_COUPLING_55 ( __cable_coupling_inv__(GEAR_BOX_GP32_TR) * CAPSTAN_RADIUS_GP32 / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_5)
#define CABLE_COUPLING_66 ( __cable_coupling_inv__(GEAR_BOX_GP32_TR) * CAPSTAN_RADIUS_GP32 / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_6)
#define CABLE_COUPLING_77 ( __cable_coupling_inv__(GEAR_BOX_GP32_TR) * CAPSTAN_RADIUS_GP32 / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_7)

#define CABLE_COUPLING_INSERTION_FACTOR (1.)
//#define CABLE_COUPLING_INSERTION_FACTOR (1./1.08) // = 0.9259259259259258
//#define CABLE_COUPLING_INSERTION_FACTOR ( 0.97086794935503984 ) //based on measured
//#define CABLE_COUPLING_INSERTION_FACTOR ( 0.97165644414807584 ) //based on calculated

#define CABLE_COUPLING_21 ( TRANSMISSION_PULLEY_RADIUS_LARGE / CAPSTAN_LINK2_LARGE_RADIUS * CAPSTAN_LINK2_SMALL_RADIUS / PARTIAL_PULLEY_LINK2_RADIUS )

#define CABLE_COUPLING_32 ( TRANSMISSION_PULLEY_RADIUS_SMALL / 1000.0 )
#define CABLE_COUPLING_31 ( TRANSMISSION_PULLEY_RADIUS_SMALL / 1000.0 )

#define CABLE_COUPLING_43 ( 1000. / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_4 * CABLE_COUPLING_INSERTION_FACTOR)
#define CABLE_COUPLING_42 ( TRANSMISSION_PULLEY_RADIUS_SMALL  / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_4 )
#define CABLE_COUPLING_41 ( TRANSMISSION_PULLEY_RADIUS_SMALL  / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_4 )

#define CABLE_COUPLING_53 ( 1000. / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_5 * CABLE_COUPLING_INSERTION_FACTOR)
#define CABLE_COUPLING_52 ( TRANSMISSION_PULLEY_RADIUS_SMALL  / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_5 )
#define CABLE_COUPLING_51 ( TRANSMISSION_PULLEY_RADIUS_SMALL  / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_5 )

#define CABLE_COUPLING_63 ( 1000. / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_6 * CABLE_COUPLING_INSERTION_FACTOR)
#define CABLE_COUPLING_62 ( TRANSMISSION_PULLEY_RADIUS_SMALL  / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_6 )
#define CABLE_COUPLING_61 ( TRANSMISSION_PULLEY_RADIUS_SMALL  / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_6 )

#define CABLE_COUPLING_73 ( 1000. / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_7 * CABLE_COUPLING_INSERTION_FACTOR)
#define CABLE_COUPLING_72 ( TRANSMISSION_PULLEY_RADIUS_SMALL  / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_7 )
#define CABLE_COUPLING_71 ( TRANSMISSION_PULLEY_RADIUS_SMALL  / CAPSTAN_TOOL_RADIUS * FINAL_RATIO_7 )




#endif /* KINEMATICS_DEFINES_H_ */
