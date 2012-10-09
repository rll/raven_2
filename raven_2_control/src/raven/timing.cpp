/*
 * timing.cpp
 *
 *  Created on: Oct 9, 2012
 *      Author: benk
 */

#include "timing.h"

#undef TIMING_INFO_FIELD
#define TIMING_INFO_FIELD(name) \
	ros::Duration TimingInfo::name##_avg; \
	ros::Duration TimingInfo::name##_max; \
	ros::Duration TimingInfo::name##_min; \
	ros::Duration TimingInfo::name##_avg_all; \
	ros::Duration TimingInfo::name##_max_all; \
	ros::Duration TimingInfo::name##_min_all;


int TimingInfo::NUM_LOOPS = 0;
int TimingInfo::NUM_LOOPS_ALL = 0;
bool TimingInfo::RESET = false;

TIMING_INFO_FIELD(overall)

TIMING_INFO_FIELD(usb_read)
TIMING_INFO_FIELD(state_machine)
TIMING_INFO_FIELD(update_state)
TIMING_INFO_FIELD(control)
TIMING_INFO_FIELD(usb_write)
TIMING_INFO_FIELD(ros)
