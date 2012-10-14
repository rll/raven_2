/*
 * struct.h - file to simplify the including of the global data structures
 *   of the project. This is in case the relative path to DS0, DS1, etc.
 *   changes.
 *   Only this file will need to be changed.
 *
 */

#ifndef STRUCT_H
#define STRUCT_H

#include "DS0.h"
#include "DS1.h"
#include "DOF_type.h"
#include "USB_packets.h"
//#include "teleoperation.h"
#include "defines.h"

//Error codes
#include <errno.h>
#include <vector>

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif


#ifndef NULL
#define NULL 0
#endif

enum t_controlmode{
    no_control       = 0,
    end_effector_control   = 1,
    joint_velocity_control = 2,
    apply_arbitrary_torque = 3,
    homing_mode            = 4,
    motor_pd_control       = 5,
    cartesian_space_control =6,
    multi_dof_sinusoid     = 7,
    joint_torque_control = 8,
    trajectory_control = 9,
    LAST_TYPE
    } ;


struct param_pass_trajectory_pt {
	param_pass param;
	double time_from_start;
};

struct param_pass_trajectory {
	double begin_time;
	double total_duration;
	t_controlmode control_mode;
	std::vector<param_pass_trajectory_pt> pts;
};

#endif // STRUCT_H
