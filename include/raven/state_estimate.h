/***************************
 *
 * FILE: stateEstimate.h
 * Created May 2010 by H. Hawkeye King
 *
 *    I apply filters or whatever to get an estimate of the state
 * (position and velocity) of the system.
 *
 */

#include "struct.h"
#include "defines.h"
#include "dof.h"

void stateEstimate(struct robot_device *device0);
void getStateLPF(struct DOF* joint);
void resetFilter(struct DOF* _joint);

