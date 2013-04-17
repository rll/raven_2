/*************************************************
 *
 * File: inv_cable_coupling.h
 *
 * History:
 *  Created 28 July 005 by Mitch
 *
 ************************************************/

//#include <linux/kernel.h>
//#include <linux/module.h>
//#include <rtai.h>

#include "struct.h"
#include "utils.h"
#include "defines.h"

#include "fwd_cable_coupling.h"

void invCableCoupling(struct device *device0, int runlevel);
void invMechCableCoupling(struct mechanism *mech, bool use_desired_ins = false);
void invMechCableCoupling_new(struct mechanism *mech, bool use_desired_ins = false);
