/*
 * overdrive_detect.c - Functions related to checking for motor over heating
 *
 */

//#include <rtai.h>
#include "struct.h"
#include "defines.h"
#include "utils.h"
#include "log.h"
#include <stdlib.h>

#define TIME_WINDOW  10000
#define MAX_OVERDRIVE_TIME 50000

//Function prototypes
int overdriveDetect(struct device *device0);
