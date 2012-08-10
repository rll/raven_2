/*******************************
 *
 *  File: fwd_cable_coupling.c
 *
 *  Calculate the forward cable coupling from a motor space Pose,
 *  (m1, m2,m3) express the desired joint pose (th1, th2, d3)
 *
 **********************************/

#include "fwd_cable_coupling.h"
#include "log.h"

extern bool disable_arm_id[2];
extern struct DOF_type DOF_types[];
extern int NUM_MECH;

/**
 * fwdCableCoupling - Calls fwdMechCableCoupling for each mechanism in device
 *
 * \param device0 pointer to device struct
 * \param runlevel current runlevel
 *
 */
 // TODO: Remove runlevel from args.
void fwdCableCoupling(struct device *device0, int runlevel)
{
	//Run fwd cable coupling for each mechanism.
	// This should be run in all runlevels.
	int mechnum=0; mechanism* _mech=NULL;
	while(loop_over_mechs(device0,_mech,mechnum)) {
		fwdMechCableCoupling(_mech);
	}
}

void fwdMechCableCoupling(struct mechanism *mech)
{
	float th1, th2, th3, th5, th6, th7;
	float th1_dot, th2_dot;
	float d4;
	float d4_dot;
	float m1, m2, m3, m4, m5, m6, m7;
	float m1_dot, m2_dot, m4_dot;
	float tr1=0, tr2=0, tr3=0, tr4=0, tr5=0, tr6=0, tr7=0;

	m1 = mech->joint[SHOULDER].mpos;
	m2 = mech->joint[ELBOW].mpos;
	m3 = mech->joint[TOOL_ROT].mpos;
	m4 = mech->joint[Z_INS].mpos;
	m5 = mech->joint[WRIST].mpos;
	m6 = mech->joint[GRASP1].mpos;
	m7 = mech->joint[GRASP2].mpos;

	m1_dot = mech->joint[SHOULDER].mvel;
	m2_dot = mech->joint[ELBOW].mvel;
	m4_dot = mech->joint[Z_INS].mvel;

	if (mech->type == GOLD_ARM){
		tr1=DOF_types[SHOULDER_GOLD].TR;
		tr2=DOF_types[ELBOW_GOLD].TR;
		tr3=DOF_types[TOOL_ROT_GOLD].TR;
		tr4=DOF_types[Z_INS_GOLD].TR;
		tr5=DOF_types[WRIST_GOLD].TR;
		tr6=DOF_types[GRASP1_GOLD].TR;
		tr7=DOF_types[GRASP2_GOLD].TR;

	} else if (mech->type == GREEN_ARM){
		tr1=DOF_types[SHOULDER_GREEN].TR;
		tr2=DOF_types[ELBOW_GREEN].TR;
		tr3=DOF_types[TOOL_ROT_GREEN].TR;
		tr4=DOF_types[Z_INS_GREEN].TR;
		tr5=DOF_types[WRIST_GREEN].TR;
		tr6=DOF_types[GRASP1_GREEN].TR;
		tr7=DOF_types[GRASP2_GREEN].TR;
	}
	else {
		log_msg("ERROR: incorrect device type in fwdMechCableCoupling");
		return;
	}

	// Forward Cable Coupling equations
	//   Originally based on 11/7/2005, Mitch notebook pg. 169
	//   Updated from UCSC code.  Code simplified by HK 8/11
	th1 = (1.0/tr1) * m1;
	th2 = (1.0/tr2) * m2;
	d4  = (1.0/tr4) * m4;

	th1_dot = (1.0/tr1) * m1_dot;
	th2_dot = (1.0/tr2) * m2_dot;
	d4_dot  = (1.0/tr4) * m4_dot;


	// Tool degrees of freedom ===========================================
	th3 = (1.0/tr3) * (m3 - m4/GB_RATIO);
	th5 = (1.0/tr5) * (m5 - m4/GB_RATIO);

	if (mech->tool_type == TOOL_GRASPER_10MM)
	{
		th6 = (1.0/tr6) * (m6 - m4/GB_RATIO);
		th7 = (1.0/tr7) * (m7 - m4/GB_RATIO);
	}
	else if (mech->tool_type == TOOL_GRASPER_8MM)
	{
		log_msg("8mm");
		// Note: sign of the last term changes for GOLD vs GREEN arm
		int sgn = (mech->type == GOLD_ARM) ? 1 : -1;
		th6 = (1.0/tr6) * (m6 - m4/GB_RATIO - sgn * (tr5*th5) * (tr5/tr6));
		th7 = (1.0/tr7) * (m7 - m4/GB_RATIO + sgn *(tr5*th5) * (tr5/tr6));
	}
	else // (mech->tool_type == TOOL_NONE) // there's tool in the robot
	{
		// coupling goes until the tool adapter pulleys
		th6 = (1.0/tr6) * (m6 - m4/GB_RATIO);
		th7 = (1.0/tr7) * (m7 - m4/GB_RATIO);
	}
	// Now have solved for th1, th2, d3, th4, th5, th6
	if (disable_arm_id[armIdFromMechType(mech->type)]) {
		mech->joint[SHOULDER].jpos 	= SHOULDER_HOME_ANGLE;// - mech->joint[SHOULDER].jpos_off;
		mech->joint[ELBOW].jpos 	= ELBOW_HOME_ANGLE;// - mech->joint[ELBOW].jpos_off;
		mech->joint[TOOL_ROT].jpos 	= TOOL_ROT_HOME_ANGLE;// - mech->joint[TOOL_ROT].jpos_off;
		mech->joint[Z_INS].jpos 	= Z_INS_HOME_ANGLE;//  - mech->joint[Z_INS].jpos_off;
		mech->joint[WRIST].jpos 	= WRIST_HOME_ANGLE;// - mech->joint[WRIST].jpos_off;
		mech->joint[GRASP1].jpos 	= GRASP1_HOME_ANGLE;// - mech->joint[GRASP1].jpos_off;
		mech->joint[GRASP2].jpos 	= GRASP2_HOME_ANGLE;// - mech->joint[GRASP2].jpos_off;

		mech->joint[SHOULDER].jvel 	= 0;// - mech->joint[SHOULDER].jpos_off;
		mech->joint[ELBOW].jvel 	= 0;// - mech->joint[ELBOW].jpos_off;
		mech->joint[Z_INS].jvel 	= 0;//  - mech->joint[Z_INS].jpos_off;
	} else {
		mech->joint[SHOULDER].jpos 	= th1;// - mech->joint[SHOULDER].jpos_off;
		mech->joint[ELBOW].jpos 		= th2;// - mech->joint[ELBOW].jpos_off;
		mech->joint[TOOL_ROT].jpos 	= th3;// - mech->joint[TOOL_ROT].jpos_off;
		mech->joint[Z_INS].jpos 		= d4;//  - mech->joint[Z_INS].jpos_off;
		mech->joint[WRIST].jpos 		= th5;// - mech->joint[WRIST].jpos_off;
		mech->joint[GRASP1].jpos 		= th6;// - mech->joint[GRASP1].jpos_off;
		mech->joint[GRASP2].jpos 		= th7;// - mech->joint[GRASP2].jpos_off;

		mech->joint[SHOULDER].jvel 	= th1_dot;// - mech->joint[SHOULDER].jpos_off;
		mech->joint[ELBOW].jvel 		= th2_dot;// - mech->joint[ELBOW].jpos_off;
		mech->joint[Z_INS].jvel 		= d4_dot;//  - mech->joint[Z_INS].jpos_off;
	}

	return;
}



