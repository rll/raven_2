/*******************************
 *
 *  File: inv_cable_coupling.c
 *
 *  Calculate the inverse cable coupling from a Joint Space Pose,
 *  (th1, th2,d3) express the desired motor pose (m1, m2, m3)
 *
 **********************************/

#include "inv_cable_coupling.h"
#include "log.h"
#include <raven/kinematics/kinematics_defines.h>
#include <raven/util/config.h>

extern struct DOF_type DOF_types[];
extern int NUM_MECH;
extern unsigned long int gTime;

typedef void(*invCableCouplingFn)(struct mechanism *mech,bool use_desired_ins);

/**
 * invCableCoupling - wrapper function that checks for correct runlevel
 *   and calls invMechCableCoupling for each mechanism in device

 *  Function should be called in all runlevels to ensure that mpos_d = jpos_d.
 *
 * \param device0 pointer to device struct
 * \param runlevel current runlevel
 *
 */
void invCableCoupling(struct device *device0, int runlevel) {
	//Run inverse cable coupling for each mechanism
	for (int i = 0; i < NUM_MECH; i++) {
		invMechCableCoupling(&(device0->mech[i]),false);
	}
}



void invMechCableCoupling(struct mechanism *mech, bool use_desired_ins) {
	if (RavenConfig.use_new_cable_coupling) {
		invMechCableCoupling_new(mech,use_desired_ins);
		return;
	}
	if (mech->type != GOLD_ARM && mech->type != GREEN_ARM) {
		log_msg("bad mech type!");
		return;
	}

	float th1, th2, th3, th5, th6, th7;
	float d4;
	float m1, m2, m3, m4, m5, m6, m7;
	float tr1=0, tr2=0, tr3=0, tr4=0, tr5=0, tr6=0, tr7=0;

	th1 = mech->joint[SHOULDER].jpos_d;
	th2 = mech->joint[ELBOW].jpos_d;
	th3 = mech->joint[TOOL_ROT].jpos_d;
	d4 = 	mech->joint[Z_INS].jpos_d;
	th5 = mech->joint[WRIST].jpos_d;
	th6 = mech->joint[GRASP1].jpos_d;
	th7 = mech->joint[GRASP2].jpos_d;

	if (mech->type == GOLD_ARM){
		tr1 = DOF_types[SHOULDER_GOLD ].TR;
		tr2 = DOF_types[ELBOW_GOLD    ].TR;
		tr3 = DOF_types[TOOL_ROT_GOLD ].TR;
		tr4 = DOF_types[Z_INS_GOLD    ].TR;
		tr5 = DOF_types[WRIST_GOLD    ].TR;
		tr6 = DOF_types[GRASP1_GOLD   ].TR;
		tr7 = DOF_types[GRASP2_GOLD   ].TR;

	} else if (mech->type == GREEN_ARM){
		tr1 = DOF_types[SHOULDER_GREEN ].TR;
		tr2 = DOF_types[ELBOW_GREEN    ].TR;
		tr3 = DOF_types[TOOL_ROT_GREEN ].TR;
		tr4 = DOF_types[Z_INS_GREEN    ].TR;
		tr5 = DOF_types[WRIST_GREEN    ].TR;
		tr6 = DOF_types[GRASP1_GREEN   ].TR;
		tr7 = DOF_types[GRASP2_GREEN   ].TR;
	}

	m1 = tr1 * th1;
	m2 = tr2 * th2;
	m4 = tr4 * d4;

	// Use the current joint position for cable coupling
	float m4_actual = mech->joint[Z_INS].mpos;

	if (use_desired_ins)
		m4_actual = m4;

	m3 = tr3 * th3 + m4_actual/GB_RATIO;
	m5 = tr5 * th5 + m4_actual/GB_RATIO;
	//  m3 = tr3 * th3 + m4/GB_RATIO;
	//  m5 = tr5 * th5 + m4/GB_RATIO;

	if (mech->tool_type == TOOL_GRASPER_10MM)
	{
		m6 = tr6 * th6 + m4_actual/GB_RATIO;
		m7 = tr7 * th7 + m4_actual/GB_RATIO;
	}
	else if (mech->tool_type == TOOL_GRASPER_8MM)
	{
		// Note: sign of the last term changes for GOLD vs GREEN arm
		int sgn = (mech->type == GOLD_ARM) ? 1 : -1;
		m6 = tr6 * th6 + m4/GB_RATIO + sgn * (tr5 * th5) * (tr5/tr6);
		m7 = tr7 * th7 + m4/GB_RATIO - sgn * (tr5 * th5) * (tr5/tr6);
	}
	else  // (mech->tool_type == TOOL_NONE)
	{
		// coupling goes until the tool adapter pulleys
		m6 = tr6 * th6 + m4/GB_RATIO;
		m7 = tr7 * th7 + m4/GB_RATIO;
	}

	/*Now have solved for desired motor positions mpos_d*/
	mech->joint[SHOULDER].mpos_d 	= m1;
	mech->joint[ELBOW].mpos_d 	= m2;
	mech->joint[TOOL_ROT].mpos_d 	= m3;
	mech->joint[Z_INS].mpos_d 	= m4;
	mech->joint[WRIST].mpos_d 	= m5;
	mech->joint[GRASP1].mpos_d 	= m6;
	mech->joint[GRASP2].mpos_d 	= m7;

	return;
}


#define T11 CABLE_COUPLING_11
#define T22 CABLE_COUPLING_22
#define T33 CABLE_COUPLING_33
#define T44 CABLE_COUPLING_44
#define T55 CABLE_COUPLING_55
#define T66 CABLE_COUPLING_66
#define T77 CABLE_COUPLING_77

#define T21 CABLE_COUPLING_21

#define T32 CABLE_COUPLING_32
#define T31 CABLE_COUPLING_31

#define T43 CABLE_COUPLING_43
#define T42 CABLE_COUPLING_42
#define T41 CABLE_COUPLING_41

#define T53 CABLE_COUPLING_53
#define T52 CABLE_COUPLING_52
#define T51 CABLE_COUPLING_51

#define T63 CABLE_COUPLING_63
#define T62 CABLE_COUPLING_62
#define T61 CABLE_COUPLING_61

#define T73 CABLE_COUPLING_73
#define T72 CABLE_COUPLING_72
#define T71 CABLE_COUPLING_71

#define inv(val) (1./(val))

void invMechCableCoupling_new(struct mechanism *mech, bool use_desired_ins) {
	if (mech->type != GOLD_ARM && mech->type != GREEN_ARM) {
		log_msg("bad mech type!");
		return;
	}

	float th1, th2, th4, th5, th6, th7;
	float d3;
	float m1, m2, m3, m4, m5, m6, m7;

	th1 = mech->joint[SHOULDER].jpos_d;
	th2 = mech->joint[ELBOW].jpos_d;
	d3  = mech->joint[Z_INS].jpos_d;
	th4 = mech->joint[TOOL_ROT].jpos_d;
	th5 = mech->joint[WRIST].jpos_d;
	th6 = mech->joint[GRASP1].jpos_d;
	th7 = mech->joint[GRASP2].jpos_d;

	float factor = 1.;///1.08;

	m1 = inv(T11) * th1;
	m2 = inv(T22) * (th2 + T21 * th1);
	m3 = inv(T33) * (d3 + T32 * th2 + T31 * th1);
	if (!use_desired_ins) {
		float th1_act = T11 * mech->joint[SHOULDER].mpos;
		float th2_act = T22 * mech->joint[ELBOW].mpos - T21 * th1_act;
		d3  = T33 * mech->joint[Z_INS].mpos - T32 * th2_act - T31 * th1_act;
	}
	m4 = inv(T44) * (th4 + T43 * factor * d3 + T42 * th2 + T41 * th1);
	m5 = inv(T55) * (th5 + T53 * factor * d3 + T52 * th2 + T51 * th1);
	m6 = inv(T66) * (th6 + T63 * factor * d3 + T62 * th2 + T61 * th1);
	m7 = inv(T77) * (th7 + T73 * factor * d3 + T72 * th2 + T71 * th1);

	/*Now have solved for desired motor positions mpos_d*/
	mech->joint[SHOULDER].mpos_d = m1;
	mech->joint[ELBOW].mpos_d    = m2;
	mech->joint[Z_INS].mpos_d    = m3;
	mech->joint[TOOL_ROT].mpos_d = m4;
	mech->joint[WRIST].mpos_d    = m5;
	mech->joint[GRASP1].mpos_d   = m6;
	mech->joint[GRASP2].mpos_d   = m7;
}
