/*
 * pid_control.c - control law
 *     pdControl - PD controller
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 * Modified 8/28/06 by Hawkeye
 *   Rewrote jointLimits function
 * Modified 8/26/11 by Hawkeye
 *   Modified for Raven_II
 */

#include <stdlib.h>
#include "defines.h"
#include "pid_control.h"
#include "log.h"
#include "utils.h"
#include "t_to_DAC_val.h"
#include "homing.h"

#include <iostream>
using namespace std;

extern struct DOF_type DOF_types[];
extern unsigned long int gTime;

static float friction_comp_torque[16] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  };


float cappedVelocity(DOF* joint);

#define HIST_SIZE 75 //200
void mpos_PD_control(struct DOF *joint, int reset_I)
{
	static int lastDACs[16][HIST_SIZE];
	static float lastTau[16][HIST_SIZE];
	static float lastErr[16][HIST_SIZE];
	static float lastP[16][HIST_SIZE];
	static float lastErrVel[16][HIST_SIZE];
	static float lastD[16][HIST_SIZE];
	static float lastErrInt[16][HIST_SIZE];
	static float lastI[16][HIST_SIZE];
	static float lastDes[16][HIST_SIZE];
	static float lastAct[16][HIST_SIZE];
	static float lastDesVel[16][HIST_SIZE];
	static float lastActVel[16][HIST_SIZE];
	static float lastJposD[16][HIST_SIZE];
	static float lastJpos[16][HIST_SIZE];

    static float eps = 10 DEG2RAD;
    float err=0.0;
    float errVel=0.0;
    float errSign=1;
    static float errInt[MAX_MECH*MAX_DOF_PER_MECH] = {0};
    float pTerm=0.0, vTerm=0.0, iTerm = 0.0;
    float friction_feedforward = 0.0;

    float kp = DOF_types[joint->type].KP;
    float kd = DOF_types[joint->type].KD;
    float ki = DOF_types[joint->type].KI;

    /* PD CONTROL LAW */

    //Calcualte error
    err    = joint->mpos_d - joint->mpos;


    float mvel = cappedVelocity(joint);
    errVel = joint->mvel_d - mvel;

    //Calculate position term
    pTerm = err * kp;

    //Calculate velocity term
    vTerm = errVel * kd;

    //Calculate integral
    if (reset_I)
        errInt[joint->type]= 0;
    else
        errInt[joint->type] += err * ONE_MS;

    //Calculate integral term
    iTerm = errInt[joint->type] * ki;

    //Calculate feedforward friction term
//    errSign = err < 0 ? -1 : 1;
//    if (fabs(err) >= eps) {
//        friction_feedforward = errSign * friction_comp_torque[joint->type];
//    }
//    else {
//        friction_feedforward = err * friction_comp_torque[joint->type] / eps;
//    }

    float tau_d = pTerm + vTerm +iTerm + friction_feedforward;

    //Finally place torque
    joint->tau_d = tau_d;

    short int DACVal = tToDACVal(joint);


    if (joint->type == GRASP1_GOLD) {
    	//log_msg("jpos d %1.4f err %1.4f tau_d %1.4f",joint->jpos_d,err,joint->tau_d);
    }

    if (joint->type == 4) {
    	//log_msg("joint 4 vel %1.4f\n",joint->mvel);
    }

    /***** update history ********/

    //DAC history
    for (int i=HIST_SIZE-1;i>0;i--) {
    	lastDACs[joint->type][i] = lastDACs[joint->type][i-1];
    }
    lastDACs[joint->type][0] = DACVal;

    //Tau history
    for (int i=HIST_SIZE-1;i>0;i--) {
    	lastTau[joint->type][i] = lastTau[joint->type][i-1];
    }
    lastTau[joint->type][0] = tau_d;

    //err history
    for (int i=HIST_SIZE-1;i>0;i--) {
    	lastErr[joint->type][i] = lastErr[joint->type][i-1];
    }
    lastErr[joint->type][0] = err;

    //pTerm history
    for (int i=HIST_SIZE-1;i>0;i--) {
    	lastP[joint->type][i] = lastP[joint->type][i-1];
    }
    lastP[joint->type][0] = pTerm;

    //errVel history
    for (int i=HIST_SIZE-1;i>0;i--) {
    	lastErrVel[joint->type][i] = lastErrVel[joint->type][i-1];
    }
    lastErrVel[joint->type][0] = errVel;

    //pTerm history
    for (int i=HIST_SIZE-1;i>0;i--) {
    	lastD[joint->type][i] = lastD[joint->type][i-1];
    }
    lastD[joint->type][0] = vTerm;

    //errInt history
    for (int i=HIST_SIZE-1;i>0;i--) {
    	lastErrInt[joint->type][i] = lastErrInt[joint->type][i-1];
    }
    lastErrInt[joint->type][0] = errInt[joint->type];

    //pTerm history
    for (int i=HIST_SIZE-1;i>0;i--) {
    	lastI[joint->type][i] = lastI[joint->type][i-1];
    }
    lastI[joint->type][0] = iTerm;

    //mpos_d history
    for (int i=HIST_SIZE-1;i>0;i--) {
    	lastDes[joint->type][i] = lastDes[joint->type][i-1];
    }
    lastDes[joint->type][0] = joint->mpos_d;

    //mpos history
    for (int i=HIST_SIZE-1;i>0;i--) {
    	lastAct[joint->type][i] = lastAct[joint->type][i-1];
    }
    lastAct[joint->type][0] = joint->mpos;

    //mvel_d history
    for (int i=HIST_SIZE-1;i>0;i--) {
    	lastDesVel[joint->type][i] = lastDesVel[joint->type][i-1];
    }
    lastDesVel[joint->type][0] = joint->mvel_d;

    //mvel history
    for (int i=HIST_SIZE-1;i>0;i--) {
    	lastActVel[joint->type][i] = lastActVel[joint->type][i-1];
    }
    lastActVel[joint->type][0] = joint->mvel;

    //jpos_d history
    for (int i=HIST_SIZE-1;i>0;i--) {
    	lastJposD[joint->type][i] = lastJposD[joint->type][i-1];
    }
    lastJposD[joint->type][0] = joint->jpos_d;

    //jpos history
    for (int i=HIST_SIZE-1;i>0;i--) {
    	lastJpos[joint->type][i] = lastJpos[joint->type][i-1];
    }
    lastJpos[joint->type][0] = joint->jpos;

    if (abs(DACVal) > MAX_INST_DAC) {
    	cerr << "****** DAC error on " << jointIndexAndArmName(joint->type) << " DACVal " << DACVal << " over " << MAX_INST_DAC << " with tau " << tau_d << " ******" << endl;
    	cerr << "tau " << tau_d << " = " << endl;
    	cerr << "  p " << pTerm << endl;
    	cerr << " +d " << vTerm << endl;
    	cerr << " +i " << iTerm << endl;
    	cerr << "err " << err << " errV " << errVel << " errInt " << errInt[joint->type] << endl;
    	cerr << "mpos " << joint->mpos << " mpos_d " << joint->mpos_d << " mvel " << joint->mvel << " mvel_d " << joint->mvel_d << endl;
    	cerr << "jpos " << joint->jpos << " jpos_d " << joint->jpos_d << endl;
    	cerr << "kp " << kp << " kd " << kd << " ki " << ki << endl;
    	cerr << "DAC hist:" << endl;
    	for (int i=0;i<HIST_SIZE;i++) {
    		cerr << lastDACs[joint->type][i] << " ";
    	}
    	cerr << endl;
    	cerr << "Tau hist:" << endl;
    	for (int i=0;i<HIST_SIZE;i++) {
    		cerr << lastTau[joint->type][i] << " ";
    	}
    	cerr << endl;
    	cerr << "err hist:" << endl;
    	for (int i=0;i<HIST_SIZE;i++) {
    		cerr << lastErr[joint->type][i] << " ";
    	}
    	cerr << endl;
    	cerr << "p term hist:" << endl;
    	for (int i=0;i<HIST_SIZE;i++) {
    		cerr << lastP[joint->type][i] << " ";
    	}
    	cerr << endl;
    	cerr << "errVel hist:" << endl;
    	for (int i=0;i<HIST_SIZE;i++) {
    		cerr << lastErrVel[joint->type][i] << " ";
    	}
    	cerr << endl;
    	cerr << "d term hist:" << endl;
    	for (int i=0;i<HIST_SIZE;i++) {
    		cerr << lastP[joint->type][i] << " ";
    	}
    	cerr << endl;
    	cerr << "errInt hist:" << endl;
    	for (int i=0;i<HIST_SIZE;i++) {
    		cerr << lastErrInt[joint->type][i] << " ";
    	}
    	cerr << endl;
    	cerr << "i term hist:" << endl;
    	for (int i=0;i<HIST_SIZE;i++) {
    		cerr << lastP[joint->type][i] << " ";
    	}
    	cerr << endl;
    	cerr << "mpos hist:" << endl;
    	for (int i=0;i<HIST_SIZE;i++) {
    		cerr << lastAct[joint->type][i] << " ";
    	}
    	cerr << endl;
    	cerr << "mpos_d hist:" << endl;
    	for (int i=0;i<HIST_SIZE;i++) {
    		cerr << lastDes[joint->type][i] << " ";
    	}
    	cerr << endl;
    	cerr << "mvel hist:" << endl;
    	for (int i=0;i<HIST_SIZE;i++) {
    		cerr << lastActVel[joint->type][i] << " ";
    	}
    	cerr << endl;
    	cerr << "mvel_d hist:" << endl;
    	for (int i=0;i<HIST_SIZE;i++) {
    		cerr << lastDesVel[joint->type][i] << " ";
    	}
    	cerr << endl;
    	cerr << "jpos hist:" << endl;
    	for (int i=0;i<HIST_SIZE;i++) {
    		cerr << lastJpos[joint->type][i] << " ";
    	}
    	cerr << endl;
    	cerr << "jpos_d hist:" << endl;
    	for (int i=0;i<HIST_SIZE;i++) {
    		cerr << lastJposD[joint->type][i] << " ";
    	}
    	cerr << endl;
    	cerr << "^^^^^^ DAC error on " << jointIndexAndArmName(joint->type) << " DACVal " << DACVal << " over " << MAX_INST_DAC << " with tau " << tau_d << " ^^^^^^" << endl;
    }

}

/**
*    jointVelControl()
*       Move joints at constant rate.
*/
float jvel_PI_control(struct DOF *_joint, int resetI){
    // Set gains.  Gains have been "empirically" tuned.
    // TODO: move this to a permanent place.
    float ki;
    float kv[MAX_MECH * MAX_DOF_PER_MECH]  = {0};
    kv[SHOULDER_GOLD] = kv[SHOULDER_GREEN] = (0.528/(15 DEG2RAD));
    kv[ELBOW_GOLD]    = kv[ELBOW_GREEN]    = (0.528/(15 DEG2RAD));
    kv[Z_INS_GOLD]    = kv[Z_INS_GREEN]    = (0.400/0.1);
    kv[TOOL_ROT_GOLD] = kv[TOOL_ROT_GREEN] = (0.005/(15 DEG2RAD));
    kv[WRIST_GOLD]    = kv[WRIST_GREEN]    = 0;
    kv[GRASP1_GOLD]   = kv[GRASP1_GREEN]   = 0;
    kv[GRASP2_GREEN]  = kv[GRASP2_GREEN]   = 0;

    static float jVelIntErr[MAX_MECH * MAX_DOF_PER_MECH];  // Integral of velocity error

    // Reset integral term
    if (resetI){
        jVelIntErr[_joint->type] = 0;
        return 0;
    }
    float jVelErr;

    // Calculate joint velocity error
    if ( is_toolDOF(_joint) )
        jVelErr = _joint->mvel_d - _joint->mvel;
    else
        jVelErr = _joint->jvel_d - _joint->jvel;

    // Integrate error over 1ms
    jVelIntErr[_joint->type] += jVelErr * ONE_MS;
    ki = kv[_joint->type] * 0.1 * 0;

    // Calculate PI velocity control
    _joint->tau_d = ( kv[_joint->type]*jVelErr + ki*jVelIntErr[_joint->type] );

    return jVelIntErr[_joint->type];
}

inline float cappedVelocity(DOF* joint) {
	return joint->mvel;

	float mvel = joint->mvel;
	float maxVel = 100000;
	switch (joint->type) {
		case TOOL_ROT_GOLD:
		case TOOL_ROT_GREEN:
			maxVel = 100;
			break;
    }
	if (mvel > maxVel) {
		err_msg("Capping joint %s velocity %1.4f at +%f\n",jointIndexAndArmName(joint->type).c_str(),mvel,maxVel);
		mvel = maxVel;
	} else if (mvel < -maxVel) {
		err_msg("Capping joint %s velocity %1.4f at -%f\n",jointIndexAndArmName(joint->type).c_str(),mvel,maxVel);
		mvel = -maxVel;
	}
	return mvel;
}
