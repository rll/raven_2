/*
 * state_machine.c
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

#include "state_machine.h"
#include "log.h"

#include <raven/state/runlevel.h>

extern int initialized;
extern int NUM_MECH;
extern int soft_estopped;
extern int globalTime;
#include <sys/times.h>
struct tms dummy_times;

/**
 * stateMachine() - Function to put data in a state machine
 *
 * In SOFTWARE_RUNLEVEL mode, get desired runlevel from the rcvdParams.
 * In PLC_RUNLEVELS mode, get desired runlevel from the PLC via atmel inputs.
 *    If the two PLC's give different runlevels, select  the lowest of the two.
 *
 * inputs - state, stateD
 *
 */
void stateMachine(struct device *device0, struct param_pass *currParams, struct param_pass *rcvdParams) {
    static int rlDelayCounter = 0; // This is a software workaround to a PLC switching transient.  Wait two cycles for the delay.

    u_08 rlDesired;
    int i;
    u_08 tmp;
    rlDesired = 9; // arbitrary large number

    // Checks runlevel of all mechanisms. Lowest runlevel is chosen.
    for (i=0;i<NUM_MECH;i++)
    {
        tmp = ( device0->mech[i].inputs & (PIN_PS0 | PIN_PS1)) >> 6;
        if ( tmp < rlDesired )
        {
            rlDesired = tmp;
        }
    }

    // already in desired runlevel.  Exit.
    u_08 curr_rl;
    u_08 curr_sl;
    RunLevel::get().getNumbers<u_08>(curr_rl,curr_sl);
    //RunLevel::updateRunlevel(rlDesired);
    if (curr_rl == rlDesired) {
        return;
    } else if (rlDelayCounter < 3) {
        rlDelayCounter++;
        return;
    }

    rlDelayCounter = 0;
    RunLevel::updateRunlevel(rlDesired);
    //log_msg("Entered runlevel %d", *rl);

}
