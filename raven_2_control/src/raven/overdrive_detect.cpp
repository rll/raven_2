/*
 * overdrive_detect.c - Functions related to checking for motor over heating
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 * 5/06 Modified by Hawkeye King
 */

#include "overdrive_detect.h"
#include "defines.h"

#include <raven/state/runlevel.h>

extern struct DOF_type DOF_types[];
extern int NUM_MECH;
extern int soft_estopped;
extern unsigned long int gTime;

/*
 * overdriveDetect - Functions to loop through all active joints to detect
 *   current situations that could cause overheating.
 *
 * input - device0 - pointer to device to check
 *
 */
int overdriveDetect(struct device *device0,u_08 runlevel)
{
    int i, j;
    struct DOF* _joint;
    int ret = FALSE;


    for (i = 0; i < NUM_MECH; i++)
        for (j = 0; j < (MAX_DOF_PER_MECH-1); j++)
        {
            _joint = &(device0->mech[i].joint[j]);
            int _dac_max = DOF_types[_joint->type].DAC_max;

            // Kill current if greater than MAX_INST_DAC.  Probably indicates a problem.
#ifdef USE_NEW_RUNLEVEL
            if (!RunLevel::get().isInit() && abs(_joint->current_cmd) > MAX_INST_DAC)
#else
            if (runlevel != RL_INIT && abs(_joint->current_cmd) > MAX_INST_DAC)
#endif
            {
                log_msg("Joint %s instant current command too high. DAC:%d \t tau:%0.3f \t jpos:%0.3f jpos_d:%0.3f\n", jointIndexAndArmName(_joint->type).c_str(), _joint->current_cmd, _joint->tau_d,_joint->jpos,_joint->jpos_d);
                _joint->current_cmd = 0;
                ret = TRUE;
            }

            else if ( _joint->current_cmd > _dac_max )
            {
                //Clip current to max_torque
                if (gTime %100 == 0 || abs(_joint->current_cmd) > MAX_INST_DAC)
                    log_warn("Joint %s is current clipped high (%d) at DAC:%d\n", jointIndexAndArmName(_joint->type).c_str(), _dac_max, _joint->current_cmd);
                _joint->current_cmd = _dac_max;
            }

            else if ( _joint->current_cmd < _dac_max*-1 )
            {
                //Clip current to -1*max_torque
                if (gTime %100 == 0 || abs(_joint->current_cmd) > MAX_INST_DAC)
                    log_warn("Joint %s is current clipped low (%d) at DAC:%d\n", jointIndexAndArmName(_joint->type).c_str(), _dac_max*-1,  _joint->current_cmd);
                _joint->current_cmd = _dac_max*-1;
            }
        }

    return ret;
}
