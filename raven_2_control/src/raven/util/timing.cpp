/*
 * timing.cpp
 *
 *  Created on: Oct 9, 2012
 *      Author: benk
 */

#include <raven/util/timing.h>

#undef TIMING_STRUCT_FIELD
#define TIMING_STRUCT_FIELD(name) TIMING_STRUCT_FIELD_SOURCE(TimingInfo,name)
#undef TIMING_STRUCT_SETUP
#define TIMING_STRUCT_SETUP(StructName) TIMING_STRUCT_SETUP_SOURCE(StructName)

TIMING_STRUCT_SETUP(TimingInfo)

TIMING_STRUCT_FIELD(overall)

TIMING_STRUCT_FIELD(usb_read)
TIMING_STRUCT_FIELD(state_machine)
TIMING_STRUCT_FIELD(update_state)
TIMING_STRUCT_FIELD(control)
TIMING_STRUCT_FIELD(usb_write)
TIMING_STRUCT_FIELD(ros)

TIMING_STRUCT_FIELD(cn_overall)
TIMING_STRUCT_FIELD(cn_get_input)
TIMING_STRUCT_FIELD(cn_set_input)
TIMING_STRUCT_FIELD(cn_copy_device)
TIMING_STRUCT_FIELD(cn_ctrl_overall)
TIMING_STRUCT_FIELD(cn_ctrl_begin)
TIMING_STRUCT_FIELD(cn_apply_ctrl)
TIMING_STRUCT_FIELD(cn_ctrl_finish)
TIMING_STRUCT_FIELD(cn_set_output)

int TimingInfo::NUM_OVER_TIME = 0;
float TimingInfo::PCT_OVER_TIME = 0;

#undef TIMING_STRUCT_FIELD
#define TIMING_STRUCT_FIELD(name) TIMING_STRUCT_FIELD_SOURCE(USBTimingInfo,name)
#undef TIMING_STRUCT_SETUP
#define TIMING_STRUCT_SETUP(StructName) TIMING_STRUCT_SETUP_SOURCE(StructName)

TIMING_STRUCT_SETUP(USBTimingInfo)

TIMING_STRUCT_FIELD(get_packet)
TIMING_STRUCT_FIELD(process_packet)

#undef TIMING_STRUCT_NAME
#define TIMING_STRUCT_NAME ControlTiming
#undef TIMING_STRUCT_SETUP
#define TIMING_STRUCT_SETUP(sn) TIMING_STRUCT_SETUP_SOURCE(TIMING_STRUCT_NAME)
#undef TIMING_STRUCT_FIELD
#define TIMING_STRUCT_FIELD(name) TIMING_STRUCT_FIELD_SOURCE(TIMING_STRUCT_NAME,name)

TIMING_STRUCT_SETUP(ControlTiming)

TIMING_STRUCT_FIELD(overall)

ros::Duration TempTiming::arm_gold;
ros::Duration TempTiming::arm_smf_gold;
ros::Duration TempTiming::arm_cmf_gold;
ros::Duration TempTiming::arm_hue_gold;
ros::Duration TempTiming::arm_green;
ros::Duration TempTiming::arm_smf_green;
ros::Duration TempTiming::arm_cmf_green;
ros::Duration TempTiming::arm_hue_green;

ros::Duration TempTiming::dev_ifu;

ros::Duration TempTiming::nmf_iau;
ros::Duration TempTiming::mf_iau;
ros::Duration TempTiming::mf_mu;
ros::Duration TempTiming::mf_mu_avg;
ros::Duration TempTiming::mf_au;

ros::Duration TempTiming::s_nmf_iau_gold;
ros::Duration TempTiming::s_mf_iau_gold;
ros::Duration TempTiming::s_mf_mu_gold;
ros::Duration TempTiming::s_mf_mu_avg_gold;
ros::Duration TempTiming::s_mf_au_gold;

ros::Duration TempTiming::c_nmf_iau_gold;
ros::Duration TempTiming::c_mf_iau_gold;
ros::Duration TempTiming::c_mf_mu_gold;
ros::Duration TempTiming::c_mf_mu_avg_gold;
ros::Duration TempTiming::c_mf_au_gold;

ros::Duration TempTiming::s_nmf_iau_green;
ros::Duration TempTiming::s_mf_iau_green;
ros::Duration TempTiming::s_mf_mu_green;
ros::Duration TempTiming::s_mf_mu_avg_green;
ros::Duration TempTiming::s_mf_au_green;

ros::Duration TempTiming::c_nmf_iau_green;
ros::Duration TempTiming::c_mf_iau_green;
ros::Duration TempTiming::c_mf_mu_green;
ros::Duration TempTiming::c_mf_mu_avg_green;
ros::Duration TempTiming::c_mf_au_green;
