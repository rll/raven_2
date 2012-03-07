/*
 * state_machine.h
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */


//Include Files
#include "struct.h"
#include "defines.h"
#include "overdrive_detect.h"
#include "update_atmel_io.h"

//Function prototypes
void stateMachine(struct device *device0, struct param_pass *currParams, struct param_pass *rcvdParams);
