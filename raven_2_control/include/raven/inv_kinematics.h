/*************************************************
 *
 * File: inv_kinematics.h
 *
 * History:
 *  Created 26 July 005 by Mitch
 *
 ************************************************/
#ifndef _INV_KIN_H
#define _INV_KIN_H

#include "struct.h"
#include <raven/kinematics/kinematics_defines.h>
#include "log.h"

// Constant DH parameters
const double go_dh_al[6] = {0,              -A12,   M_PI - A23,  0, M_PI/2, -M_PI/2};
const double go_dh_a[6]  = {0,              0,      0,         0, 0, 0 };
const double gr_dh_al[6] = {M_PI,           A12,   A23,        M_PI, M_PI/2, M_PI/2};
const double gr_dh_a[6]  = {0,              0,      0,         0, 0, 0 };

void invKin(struct device *device0, struct param_pass * currParam);
int invMechKin(struct mechanism *mech);
int invMechKinNew(struct mechanism *mech,bool test=false);

#endif
