/**
*
*   File: homing.cpp
*
*   Created 3-Nov-2011 by Hawkeye King
*
*      Based on concept by UCSC, I implement a procedure for joint position discovery from relative encoders.
*
*/
#include "DS0.h"
#include <raven/util/config.h>

struct HomingConfig : public rosx::ConfigGroup {
	bool homing_stop_at_tool_max;
	bool homing_stop_at_tool_ready;
	bool homing_stop_at_arm_max;

	HomingConfig() : rosx::ConfigGroup() {
		ConfigGroup_flag(homing_stop_at_tool_max);
		ConfigGroup_flag(homing_stop_at_tool_ready);
		ConfigGroup_flag(homing_stop_at_arm_max);
//		ConfigGroup_option(param1,float);
//		ConfigGroup_option(param2_has_default,std::string,"thedefault");
//		ConfigGroup_options(param3,"v,param-number-three",int);
	}


};

class Homing {
public:
static void homing(struct DOF*);
static int check_homing_condition(struct DOF*);

static HomingConfig Config;
};
