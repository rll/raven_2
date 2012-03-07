/***********************************************
 **
 ** File: DS1.h
 ** Authors: Hawkeye King, Arash Aminpour
 **
 ** This file implements a data structure to pass
 ** control parameters to the device-control module
 ** from user-space.
 **
 ***********************************************/

#ifndef DS1_H
#define DS1_H

#ifndef DS0_H
#include "DS0.h"
#endif

#define STOP 0    // runlevel 0 is STOP state

// TODO: Delete stuff from OLD R_I code!
struct param_pass {
  u_08   runlevel;				  // device runlevel
  u_08   sublevel;				  // device runlevel
  int    enc_d[MAX_MECH_PER_DEV * MAX_DOF_PER_MECH];       // desired encoder position
  int    dac_d[MAX_MECH_PER_DEV * MAX_DOF_PER_MECH];       // desired dac level
  float  jpos_d[MAX_MECH_PER_DEV * MAX_DOF_PER_MECH];    // desired joint coordinates
  float  jvel_d[MAX_MECH_PER_DEV * MAX_DOF_PER_MECH];    // desired joint velocity
  float  kp[MAX_MECH_PER_DEV * MAX_DOF_PER_MECH];        // position gain
  float  kd[MAX_MECH_PER_DEV * MAX_DOF_PER_MECH];        // derivative gain
  struct position xd[MAX_MECH_PER_DEV];		  // desired end-point position
  struct orientation rd[MAX_MECH_PER_DEV];	          // desired end-point orientation
  int    torque_vals[MAX_MECH_PER_DEV * MAX_DOF_PER_MECH]; // desired force/torque
  char   cmdStr[200];
  int    surgeon_mode;
  int    robotControlMode;
};

#endif
