/*
 * t_to_DAC_val.c
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 * Modified by Hawkeye King
 */

#include "t_to_DAC_val.h"
#include "motor.h"
#include "utils.h"
#include "log.h"

extern struct DOF_type DOF_types[];
extern int NUM_MECH;

extern unsigned int soft_estopped;


/**
 * TorqueToDAC() - Converts desired torque on each joint to desired DAC level
 *
 * Precondition - tau_d has been set for each joint
 *
 * Postcondition - current_cmd is set for each joint
 * \param device0 pointer to device structure
 *
 */
int TorqueToDAC(struct device *device0)
{
	struct DOF *_joint = NULL;
	struct mechanism* _mech = NULL;
    int i=0, j=0;

    // for each arm
    while (loop_over_joints(device0, _mech, _joint, i,j) ) {
    	if (_joint->type == NO_CONNECTION_GOLD || _joint->type == NO_CONNECTION_GREEN)
    		continue;
    	_joint->current_cmd = tToDACVal( _joint );  // Convert torque to DAC value

    	if ( soft_estopped )
    		_joint->current_cmd = 0;

    }
    return 0;
}

/**
    * t_to_DAC_val() - Takes a torque value and DOF and returns the appropriate
 *   encoder value.  This function could be reduced to one line, but that would be un-readable.
 *
 * inputs - torque - the desired torque
 *          dof - the degree of freedom we are using
 *
 * output - encoder value
 * \param joint pointer to DOF structure
 */
short int tToDACVal(struct DOF *joint)
{
    int        DACVal;
    short int  result;
    float      TFamplifier,
    TFmotor;

    int j_index = joint->type;

    TFmotor     = 1 / DOF_types[j_index].tau_per_amp;    // Determine the motor TF  = 1/(tau per amp)
    TFamplifier =     DOF_types[j_index].DAC_per_amp;    // Determine the amplifier TF = (DAC_per_amp)

    DACVal = (int)(joint->tau_d * TFmotor * TFamplifier);  //compute DAC value: DAC=[tau*(amp/torque)*(DACs/amp)]

    //Perform range checking and convert to short int
    //Note: toShort saturates at max value for short int.
    toShort(DACVal, &result);

    return result;
}

/**
 * clearDACs() - sets DACs to 0V
 *
 * input: buffer_out
 * \param device0 pointer to device structure
 */
void clearDACs(struct device *device0)
{
	struct DOF *_joint = NULL;
	struct mechanism* _mech = NULL;
	int i=0, j=0;

    //Set all encoder values to no movement
	while (loop_over_joints(device0, _mech, _joint, i,j) ) {
		_joint->current_cmd = 0;
	}
}

int TorqueToDACTest(struct device *device0)
{
    static int count;
    struct DOF *_joint = NULL;
    struct mechanism* _mech = NULL;
    int i=0, j=0;
    static unsigned int output=0x4000;
    if (output < 0x8000)
        output = 0xa000;
    else
        output = 0x6000;
    // for each arm
    count++;

    while (loop_over_joints(device0, _mech, _joint, i,j) ) {
    	_joint->current_cmd = output;
    }

    return 0;
}
