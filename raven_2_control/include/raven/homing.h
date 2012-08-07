/**
*
*   File: homing.cpp
*
*   Created 3-Nov-2011 by Hawkeye King
*
*      Based on concept by UCSC, I implement a procedure for joint position discovery from relative encoders.
*
*/
#include "DS0.h"

void homing(struct DOF*);
int check_homing_condition(struct DOF*);
