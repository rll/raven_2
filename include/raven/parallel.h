/*
 * parallel.h
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */


//Data Structures
#include "struct.h" /*Includes DS0, DS1, DOF_type*/
#include "defines.h"

//Parallel Port points
//#include <asm/io.h>
#include <sys/io.h>

//Pin Information
#define PARALLEL_PORT     0x378 // 0xec00//
#define DUTY_CYCLE_BIT     0x04

void parallelUpdate(int runlevel, int endOfLoop);
