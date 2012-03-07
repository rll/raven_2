/*************************************************
 *
 * File: fwd_cable_coupling.h
 *
 * History:
 *  Created 3 August 2005 by Mitch
 *
 ************************************************/

//#include <linux/kernel.h>
//#include <linux/module.h>
//#include <rtai.h>

#include "struct.h"
#include "utils.h"
#include "defines.h"

#define GB_RATIO (GEAR_BOX_GP42_TR/GEAR_BOX_GP32_TR * (1.08 * CAPSTAN_RADIUS_GP32/CAPSTAN_RADIUS_GP42))

void fwdCableCoupling(struct device *device0, int runlevel);
void fwdMechCableCoupling(struct mechanism *mech);
