/*
 * fifo.h - FIFO handling routines
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

//Rtai Include Files
//#include <rtai.h>
//#include <rtai_sched.h>
//#include <rtai_fifos.h>

//Include Files
#include "struct.h" //Includes DS0, DS1, DOF_type
#include "defines.h"

#include "stddef.h" //For size_t

#define NO_PACKET_FOUND   0
#define CMD_PACKET_RCVD   1
#define BAD_PACKET_FND    2

int putFIFOData(int FIFO, struct device *device0);
int putFIFODataSized(int FIFO, void* data, size_t size);
int getFIFOData(int FIFO, struct param_pass *rcv_params);



