/*
 * RT_PROCESS Header file
 *
 * written by Ken Fodero
 * BioRobotics Lab, University of Washington
 * ken@ee.washington.edu
 *
 */

#ifndef __RTPROCESS_H__
#define __RTPROCESS_H__

//RTAI + LINUX include files
//#include <linux/kernel.h>
//#include <linux/module.h>

// Module descriptors
//module_param(deviceType, int, SURGICAL_ROBOT);
//MODULE_PARM_DESC(deviceType, "The Device to run (1 for Surgical Robot)");
//MODULE_LICENSE("GPL");

//#include <math.h>
//#include <linux/delay.h>
//#include <rtai.h>
//#include <rtai_sched.h>
//#include <rtai_fifos.h>

#include "log.h"

//Local include files
#include "defines.h"
#include "USB_init.h"
#include "init.h"
#include "get_USB_packet.h"
#include "put_USB_packet.h"
#include "local_io.h"
#include "dof.h"
#include "update_device_state.h"
#include "update_atmel_io.h"
#include "overdrive_detect.h"
#include "inv_kinematics.h"
#include "inv_cable_coupling.h"
#include "fwd_kinematics.h"
#include "fwd_cable_coupling.h"
#include "parallel.h"
#include "state_machine.h"
#include "state_estimate.h"
#include "grav_comp.h"
#include "t_to_DAC_val.h"

#ifdef USE_RTNET
#include "network.h"
#endif
#ifdef USE_NETWORK
#include "network.h"
#endif

//Data Structures
#include "struct.h" /*Includes DS0, DS1, DOF_type*/


//Other Defines
#define CYPRESS_ENABLED 1
#define STARTUP_ERROR   -11

/* RTAI parameters */
#define RT_TIME_ONE_MS 1000000
#define TICK_PERIOD    1000000
#define TASK_PRIORITY  1
#define STACK_SIZE     50000
#define USE_FPU        1

int init_module(void);
void cleanup_module(void);
//static void rt_process(long t);

void displayVals(struct device device0, int period);
int handler (int vec, int signo, struct pt_regs *regs, void *dummy);

#endif
