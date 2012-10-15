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
#ifndef USE_NEW_RUNLEVEL
    u_08 *rl = &(currParams->runlevel);
#endif
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
#ifdef USE_NEW_RUNLEVEL
    u_08 curr_rl;
    u_08 curr_sl;
    RunLevel::get().getNumbers<u_08>(curr_rl,curr_sl);
    RunLevel::updateRunlevel(rlDesired);
    if (curr_rl == rlDesired) {
#else
	if ( *rl == rlDesired) {
#endif
        return;
    } else if (rlDelayCounter < 3) {
        rlDelayCounter++;
        return;
    }

    rlDelayCounter = 0;
#ifdef USE_NEW_RUNLEVEL
    //RunLevel::updateRunlevel(rlDesired);
#else
    *rl = rlDesired;            // Update Run Level
    device0->runlevel = *rl;    // Log runlevels in DS0.
    RunLevel::updateRunlevel(*rl);
#endif
    //log_msg("Entered runlevel %d", *rl);

#ifdef USE_NEW_RUNLEVEL
    RunLevel runlevel = RunLevel::get();
    if (runlevel.isEstop()) {
#else
    if (*rl == RL_E_STOP) {
#endif
#ifndef USE_NEW_RUNLEVEL
        if (soft_estopped) {
        	err_msg("Software e-stop.\n");
            soft_estopped = FALSE;
        } else {
        	for (i=0;i<NUM_MECH;i++) {
        		tmp = ( device0->mech[i].inputs & (PIN_PS0 | PIN_PS1)) >> 6;
        		if (tmp == RL_E_STOP) {
        			err_msg("E-stop signal from mech %d\n",i);
        		}
        	}
        }

        err_msg("*** ENTERED E-STOP STATE ***\n");
        initialized = FALSE;
        currParams->sublevel = 0;
#endif
    }

}
