#ifndef GRAV_COMP_H
#define GRAV_COMP_H

#include <math.h>

#include "struct.h"
#include "defines.h"


#define G 9.8

#define ALPHA_12 (75.0*M_PI/180.0)
#define ALPHA_23 (52.0*M_PI/180.0)

#define MASS_1 (float)(0.2395*2.7)
#define MASS_2 (float)(0.3568*2.7)
#define MASS_3 (float)(0.15*2.7)

#define cm1x  (float)(-6.67/1000.0)
#define cm1y (float)(-108.36/1000.0)
#define cm1z (float)(132.29/1000.0)

#define cm2x (float)(-1.5/1000.0)
#define cm2y (float)(-63.46/1000.0)
#define cm2z (float)(311.64/1000.0)


#define cm3x (float)(0.0/1000.0)
#define cm3y (float)(10.0/1000.0)
#define cm3z (float)(450.0/1000.0)

//motor controller output/gravity torque
#define coeff_tau0 (float)(-1.0/27.86)
#define coeff_tau1 (float)(-1.0/25.83)

#define BASE_ROTATION_ANGLE  15.0 //deg

int gravComp(struct device *device0);

#endif
