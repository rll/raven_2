/*
 * pid_control.h - control law
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 * $Id: pd_control.h,v 1.1 2007/03/22 20:17:59 hawkeye1 Exp $
 */

#ifndef PD_CONTROL_H
#define PD_CONTROL_H

//Local include files
#include "struct.h" /*Includes DS0, DS1, DOF_type*/
#include "dof.h"

//PD Controller type defines
#define JOINT_PD_CTRL  1
#define MOTOR_PD_CTRL  2
#define MOTOR_VEL_CTRL  2

//Function Prototypes
void mpos_PD_control(struct DOF *joint, int reset_I=0);
float jvel_PI_control(struct DOF*, int);

#endif // PD_CONTROL_H
