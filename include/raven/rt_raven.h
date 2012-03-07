/**
* File: rt_raven.h
* Created 13-oct-2011 by Hawkeye
*
*   Runs all raven control functions.
*   Code split out from rt_process_preempt.cpp, in order to provide more flexibility.
*
*/

#include "DS0.h"
#include "DS1.h"

int controlRaven(struct robot_device*, struct param_pass*);
