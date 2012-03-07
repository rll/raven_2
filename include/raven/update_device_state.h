
/*
 * update_device_state.h
 */

//Include files
//#include <rtai.h>
#include "defines.h"
#include "struct.h" /*Includes DS0, DS1, DOF_type*/

int updateDeviceState(struct param_pass * params_current, struct param_pass * params_update, struct device *device0);

void setRobotControlMode(t_controlmode);
void setDofTorque(unsigned int, unsigned int, int);
