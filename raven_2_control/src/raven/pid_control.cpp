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

#include <raven/state/runlevel.h>

#include <iostream>
#include <fstream>
using namespace std;

extern struct DOF_type DOF_types[];
extern unsigned long int gTime;

static float friction_comp_torque[16] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,
                                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0  };


float cappedVelocity(DOF* joint);
float cappedVelocityDesired(DOF* joint);
float cappedPositionError(DOF* joint);

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
    float pTerm=0.0, dTerm=0.0, iTerm = 0.0;
    float friction_feedforward = 0.0;

    float kp = DOF_types[joint->type].KP;
    float kd = DOF_types[joint->type].KD;
    float ki = DOF_types[joint->type].KI;

    std::string jointName = jointIndexAndArmName(joint->type);

    /* PD CONTROL LAW */

    //Calcualte error
    //err    = joint->mpos_d - joint->mpos;
    err = cappedPositionError(joint);


    float mvel = cappedVelocity(joint);
    float mvel_d = cappedVelocityDesired(joint);
    errVel = mvel_d - mvel;

    //Calculate position term
    pTerm = err * kp;

    //Calculate velocity term
    dTerm = errVel * kd;

    //Calculate integral
    if (reset_I) {
    	//log_warn("PID %-16s: Resetting integral [%i]",jointName.c_str(),LoopNumber::get());
        errInt[joint->type]= 0;
    } else {
        errInt[joint->type] += err * ONE_MS;
    }

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

    float tau_d = pTerm + iTerm + dTerm + friction_feedforward;

    /*
    if (RunLevel::get().isPedalDown() && LoopNumber::every(1)) {
    	//printf("PID: %-16s P: % 7.3f  I: % 7.3f  D: % 7.3f\n",jointIndexAndArmName(joint->type).c_str(),pTerm,iTerm,dTerm);
    	if (fabs(err)>=0.0005 || fabs(pTerm) >=0.0005 || fabs(iTerm) >= 0.0005) {
    		//printf("PID: %-16s ERR: % 7.2f [% 7.2f % 7.2f] [% 7.3f % 7.3f]  P: % 7.3f  I: % 7.3f\n",jointIndexAndArmName(joint->type).c_str(),err,joint->mpos_d,joint->mpos,joint->jpos_d,joint->jpos,pTerm,iTerm);
    	}
    }
    */

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

    /*DAC    history */ for (int i=HIST_SIZE-1;i>0;i--) { lastDACs[joint->type][i]   = lastDACs[joint->type][i-1]; }   lastDACs[joint->type][0] = DACVal;
    /*Tau    history */ for (int i=HIST_SIZE-1;i>0;i--) { lastTau[joint->type][i]    = lastTau[joint->type][i-1]; }    lastTau[joint->type][0] = tau_d;
    /*err    history */ for (int i=HIST_SIZE-1;i>0;i--) { lastErr[joint->type][i]    = lastErr[joint->type][i-1]; }    lastErr[joint->type][0] = err;
    /*pTerm  history */ for (int i=HIST_SIZE-1;i>0;i--) { lastP[joint->type][i]      = lastP[joint->type][i-1]; }      lastP[joint->type][0] = pTerm;
    /*errInt history */ for (int i=HIST_SIZE-1;i>0;i--) { lastErrInt[joint->type][i] = lastErrInt[joint->type][i-1]; } lastErrInt[joint->type][0] = errInt[joint->type];
    /*iTerm  history */ for (int i=HIST_SIZE-1;i>0;i--) { lastI[joint->type][i]      = lastI[joint->type][i-1]; }      lastI[joint->type][0] = iTerm;
    /*errVel history */ for (int i=HIST_SIZE-1;i>0;i--) { lastErrVel[joint->type][i] = lastErrVel[joint->type][i-1]; } lastErrVel[joint->type][0] = errVel;
    /*dTerm  history */ for (int i=HIST_SIZE-1;i>0;i--) { lastD[joint->type][i]      = lastD[joint->type][i-1]; }      lastD[joint->type][0] = dTerm;
    /*mpos_d history */ for (int i=HIST_SIZE-1;i>0;i--) { lastDes[joint->type][i]    = lastDes[joint->type][i-1]; }    lastDes[joint->type][0] = joint->mpos_d;
    /*mpos   history */ for (int i=HIST_SIZE-1;i>0;i--) { lastAct[joint->type][i]    = lastAct[joint->type][i-1]; }    lastAct[joint->type][0] = joint->mpos;
    /*mvel_d history */ for (int i=HIST_SIZE-1;i>0;i--) { lastDesVel[joint->type][i] = lastDesVel[joint->type][i-1]; } lastDesVel[joint->type][0] = joint->mvel_d;
    /*mvel   history */ for (int i=HIST_SIZE-1;i>0;i--) { lastActVel[joint->type][i] = lastActVel[joint->type][i-1]; } lastActVel[joint->type][0] = joint->mvel;
    /*jpos_d history */ for (int i=HIST_SIZE-1;i>0;i--) { lastJposD[joint->type][i]  = lastJposD[joint->type][i-1]; }  lastJposD[joint->type][0] = joint->jpos_d;
    /*jpos   history */ for (int i=HIST_SIZE-1;i>0;i--) { lastJpos[joint->type][i]   = lastJpos[joint->type][i-1]; }   lastJpos[joint->type][0] = joint->jpos;

    if (abs(DACVal) > MAX_INST_DAC) {
    	cerr << "****** DAC error on " << jointIndexAndArmName(joint->type) << " DACVal " << DACVal << " over " << MAX_INST_DAC << " with tau " << tau_d << " ******" << endl;
    	cerr << "tau " << tau_d << " = " << endl;
    	cerr << "  p " << pTerm << endl;
    	cerr << " +d " << dTerm << endl;
    	cerr << " +i " << iTerm << endl;
    	cerr << "err " << err << " errV " << errVel << " errInt " << errInt[joint->type] << endl;
    	cerr << "mpos " << joint->mpos << " mpos_d " << joint->mpos_d << " mvel " << joint->mvel << " mvel_d " << joint->mvel_d << endl;
    	cerr << "jpos " << joint->jpos << " jpos_d " << joint->jpos_d << endl;
    	cerr << "kp " << kp << " kd " << kd << " ki " << ki << endl;
    	cerr << "^^^^^^ DAC error on " << jointIndexAndArmName(joint->type) << " DACVal " << DACVal << " over " << MAX_INST_DAC << " with tau " << tau_d << " ^^^^^^" << endl;

    	static bool opened = false;
    	ios_base::openmode mode = fstream::out;
    	if (opened) {
    		mode |= fstream::app;
    	}
    	std::string filename = "overcurrent.yaml";
    	std::string path = "/home/biorobotics/.ros/log/" + filename;
    	//std::string path = filename;
    	fstream ferr(path.c_str(), mode);
    	opened = true;
    	ferr << "---\n";
    	ferr << "#****** DAC error on " << jointIndexAndArmName(joint->type) << " DACVal " << DACVal << " over " << MAX_INST_DAC << " with tau " << tau_d << " ******" << endl;
    	ferr << "joint: " << jointIndexAndArmName(joint->type) << endl;
    	ferr << "dac: " << DACVal << endl;
    	ferr << "max_dac: " << MAX_INST_DAC << endl;
    	ferr << "loop number: " << LoopNumber::getMain() << endl;
    	timespec tm = LoopNumber::getMainTime();
    	double t = ((double)tm.tv_sec) + ((double)tm.tv_nsec) / 1e9;
    	char t_str[30];
    	sprintf(t_str,"%.9f",t);
    	ferr << "time: " << t_str << endl;
		ferr << "tau: " << endl;
		ferr << "  val: " << tau_d << endl;
		ferr << "  p " << pTerm << endl;
		ferr << "  d " << dTerm << endl;
		ferr << "  i " << iTerm << endl;
		ferr << "err: " << endl;
		ferr << "  p: " << err << endl;
		ferr << "  d: " << errVel << endl;
		ferr << "  i: " << errInt[joint->type] << endl;
		ferr << "mpos:   " << joint->mpos << endl;
		ferr << "mpos_d: " << joint->mpos_d << endl;
		ferr << "mvel:   " << joint->mvel << endl;
		ferr << "mvel_d: " << joint->mvel_d << endl;
		ferr << "jpos:   " << joint->jpos << endl;
		ferr << "jpos_d: " << joint->jpos_d << endl;
		ferr << "gains:" << endl;
		ferr << "  kp: " << kp << endl;
		ferr << "  kd: " << kd << endl;
		ferr << "  ki: " << ki << endl;


		ferr << "DAC_hist:" << "\t";
		ferr << "  [";
		for (int i=0;i<HIST_SIZE;i++) {
			ferr << lastDACs[joint->type][i] << ", ";
		}
		ferr << "]" << endl;
		ferr << "tau_hist:" << "\t";
		ferr << "  [";
		for (int i=0;i<HIST_SIZE;i++) {
			ferr << lastTau[joint->type][i] << ", ";
		}
		ferr << "]" << endl;
		ferr << "err_hist:" << "\t";
		ferr << "  [";
		for (int i=0;i<HIST_SIZE;i++) {
			ferr << lastErr[joint->type][i] << ", ";
		}
		ferr << "]" << endl;
		ferr << "p_term_hist:" << "\t";
		ferr << "  [";
		for (int i=0;i<HIST_SIZE;i++) {
			ferr << lastP[joint->type][i] << ", ";
		}
		ferr << "]" << endl;
		ferr << "errVel_hist:" << "\t";
		ferr << "  [";
		for (int i=0;i<HIST_SIZE;i++) {
			ferr << lastErrVel[joint->type][i] << ", ";
		}
		ferr << "]" << endl;
		ferr << "d_term_hist:" << "\t";
		ferr << "  [";
		for (int i=0;i<HIST_SIZE;i++) {
			ferr << lastP[joint->type][i] << ", ";
		}
		ferr << "]" << endl;
		ferr << "errInt_hist:" << "\t";
		ferr << "  [";
		for (int i=0;i<HIST_SIZE;i++) {
			ferr << lastErrInt[joint->type][i] << ", ";
		}
		ferr << "]" << endl;
		ferr << "i_term_hist:" << "\t";
		ferr << "  [";
		for (int i=0;i<HIST_SIZE;i++) {
			ferr << lastP[joint->type][i] << ", ";
		}
		ferr << "]" << endl;
		ferr << "mpos_hist:" << "\t";
		ferr << "  [";
		for (int i=0;i<HIST_SIZE;i++) {
			ferr << lastAct[joint->type][i] << ", ";
		}
		ferr << "]" << endl;
		ferr << "mpos_d_hist:" << "\t";
		ferr << "  [";
		for (int i=0;i<HIST_SIZE;i++) {
			ferr << lastDes[joint->type][i] << ", ";
		}
		ferr << "]" << endl;
		ferr << "mvel hist:" << "\t";
		ferr << "  [";
		for (int i=0;i<HIST_SIZE;i++) {
			ferr << lastActVel[joint->type][i] << ", ";
		}
		ferr << "]" << endl;
		ferr << "mvel_d_hist:" << "\t";
		ferr << "  [";
		for (int i=0;i<HIST_SIZE;i++) {
			ferr << lastDesVel[joint->type][i] << ", ";
		}
		ferr << "]" << endl;
		ferr << "jpos_hist:" << "\t";
		ferr << "  [";
		for (int i=0;i<HIST_SIZE;i++) {
			ferr << lastJpos[joint->type][i] << ", ";
		}
		ferr << "]" << endl;
		ferr << "jpos_d_hist:" << "\t";
		ferr << "  [";
		for (int i=0;i<HIST_SIZE;i++) {
			ferr << lastJposD[joint->type][i] << ", ";
		}
		ferr << "]" << endl;

    	ferr << "# tau " << tau_d << " = " << endl;
    	ferr << "#   p " << pTerm << endl;
    	ferr << "#  +d " << dTerm << endl;
    	ferr << "#  +i " << iTerm << endl;
    	ferr << "# err " << err << " errV " << errVel << " errInt " << errInt[joint->type] << endl;
    	ferr << "# mpos " << joint->mpos << " mpos_d " << joint->mpos_d << " mvel " << joint->mvel << " mvel_d " << joint->mvel_d << endl;
    	ferr << "# jpos " << joint->jpos << " jpos_d " << joint->jpos_d << endl;
    	ferr << "# kp " << kp << " kd " << kd << " ki " << ki << endl;

    	ferr << "#^^^^^^ DAC error on " << jointIndexAndArmName(joint->type) << " DACVal " << DACVal << " over " << MAX_INST_DAC << " with tau " << tau_d << " ^^^^^^" << endl;

    	ferr.close();
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

inline float cappedPositionError(DOF* joint) {
	float err = joint->mpos_d - joint->mpos;
	float maxErr = 100000;
	switch (jointTypeFromCombinedType(joint->type)) {
		case GRASP1:
		case GRASP2:
			maxErr = DOF_types[joint->type].TR * 40 DEG2RAD;
			break;
		case Z_INS:
			maxErr = 40;
			break;
		case ELBOW:
			maxErr = DOF_types[joint->type].TR * 10 DEG2RAD;
			break;
		case SHOULDER:
			maxErr = DOF_types[joint->type].TR * 10 DEG2RAD;
			break;
    }
	if (err > maxErr) {
		err_msg("Capping joint %s pos error %1.4f at +%f\n",jointIndexAndArmName(joint->type).c_str(),err,maxErr);
		err = maxErr;
	} else if (err < -maxErr) {
		err_msg("Capping joint %s pos error %1.4f at -%f\n",jointIndexAndArmName(joint->type).c_str(),err,maxErr);
		err = -maxErr;
	}
	return err;
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

inline float cappedVelocityDesired(DOF* joint) {
	float mvel_d = joint->mvel_d;
	float speed_limit = DOF_types[joint->type].speed_limit;
	if (mvel_d > speed_limit) {
		mvel_d = speed_limit;
	} else if (mvel_d < -speed_limit) {
		mvel_d = -speed_limit;
	}

	return mvel_d;
}
