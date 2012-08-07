/*
 * globals.c - Global variables
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

#include "struct.h" /* DS0, DS1, DOF_types defines */
#include "USB_init.h"

//unsigned long int gTime = 0;

struct DOF_type DOF_types[MAX_MECH*MAX_DOF_PER_MECH];
//struct traj trajectory[MAX_MECH*MAX_DOF_PER_MECH];
USBStruct USBBoards;
