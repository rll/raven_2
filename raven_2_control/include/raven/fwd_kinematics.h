/*************************************************
 *
 * File: fwd_kinematics.h
 *
 * History:
 *  Created 3 August, 2005 by Mitch
 *
 ************************************************/

#include "struct.h"
#include <raven/kinematics/kinematics_defines.h>

void fwdKin(struct device *device0, int runlevel);
void fwdMechKinNew(struct mechanism *mech);
void fwdMechKin(struct mechanism *mech);
