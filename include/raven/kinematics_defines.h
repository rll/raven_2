/*
 * kinematics_defines.h
 *
 *  Created on: Jun 19, 2012
 *      Author: biorobotics
 */

#ifndef KINEMATICS_DEFINES_H_
#define KINEMATICS_DEFINES_H_

#include "defines.h"

#define _A5 0.0087  // 8.7mm
const double base_tilt = 0 DEG2RAD;
#define BASE_TILT 0 DEG2RAD

#define SHOULDER_OFFSET_GOLD atan(0.3471/0.9014) //from original URDF
#define TOOL_ROT_OFFSET_GOLD M_PI_4

//TODO: these are probably incorrect
#define SHOULDER_OFFSET_GREEN atan(0.3471/0.9014)//from original URDF
#define TOOL_ROT_OFFSET_GREEN M_PI_4

#define THETA_12 -A12
#define THETA_23 -A23
#define DW 0.012


#endif /* KINEMATICS_DEFINES_H_ */
