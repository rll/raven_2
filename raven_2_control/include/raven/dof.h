
/*
 * dof.h - Degree of Freedom related functions
 *
 */

//Include Files
//#include <linux/kernel.h>
//#include <linux/module.h>
//#include <rtai.h>

#include "struct.h" //DS0, DS1, DOF_type
#include "defines.h"

//Motor related defines
#include "motor.h"

#define MOTOR_ANGLE    0
#define JOINT_ANGLE    1


//Function prototypes
int processEncVal(unsigned char buffer[], int channel);

void encToJPos(struct DOF *joint);
void encToMPos(struct DOF *joint);
float encToMPos2(struct DOF *joint);
int normalizeEncCnt(struct DOF *joint);

