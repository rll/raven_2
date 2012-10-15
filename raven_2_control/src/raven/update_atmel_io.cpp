/*
 * update_atmel_io.c
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 * Modified by Hawkeye King 5/2006
 */

#include "update_atmel_io.h"
#include "log.h"

#include <raven/state/runlevel.h>

extern int initialized;
extern int soft_estopped;
extern int NUM_MECH;
extern unsigned long int gTime;

void updateAtmelOutputs(struct device *device0, int runlevel)
{
    static int counter;
    unsigned char i, outputs = 0x00;

    //Update Foot Pedal
#ifdef USE_NEW_RUNLEVEL
    RunLevel rl = RunLevel::get();
    bool fp = RunLevel::getPedal();
    if (!rl.isEstop() && fp) {
#else
    if ( (runlevel>1) && (device0->surgeon_mode) ) {
#endif
        outputs |= PIN_FP;
    }

    //Update Ready
#ifdef USE_NEW_RUNLEVEL
    if (RunLevel::isInitialized()) {
#else
    if (initialized) {
    	if (runlevel == 1) {
    		printf("rl 1 & init %s\n",RunLevel::get().str().c_str());
    	}
#endif
    	outputs |= PIN_READY;
    }


    //Update Linux State
#ifdef USE_NEW_RUNLEVEL
    int rl_num;
    int sl_num;
	rl.getNumbers<int>(rl_num,sl_num);
	outputs |= (rl_num & (PIN_LS0 | PIN_LS1));
#else
    outputs |= (runlevel & (PIN_LS0 | PIN_LS1));
#endif

    //Update WD Timer - if not software triggered
#ifdef USE_NEW_RUNLEVEL
    if (!rl.isSoftwareEstop()) {
#else
    if ( !soft_estopped ) {
#endif
        if ( counter <= (WD_PERIOD / 2) ) {
            outputs |= PIN_WD;
        } else if (counter >= WD_PERIOD) {
            counter = 0;
        }
    } else {
    	//printf("SES uaio\n");
    }

/*
#ifdef USE_NEW_RUNLEVEL
    if (rl.isSoftwareEstop()) {
    	log_msg("xx%i FP %hhu R %hhu RL %i %i %s %hhu",RunLevel::isInitialized(),outputs & PIN_FP, outputs & PIN_READY, (int) outputs & (PIN_LS0 | PIN_LS1),rl_num,rl.str().c_str(),outputs);
    } else {
    	log_msg_throttle(0.1,"%i FP %hhu R %hhu RL %i %i %s",RunLevel::isInitialized(),outputs & PIN_FP, outputs & PIN_READY, (int) outputs & (PIN_LS0 | PIN_LS1),rl_num,rl.str().c_str());
    }
#else
    if (soft_estopped) {
    	log_msg("xx%i FP %hhu R %hhu RL %i %i %hhu",initialized, outputs & PIN_FP, outputs & PIN_READY, (int) outputs & (PIN_LS0 | PIN_LS1),runlevel,outputs);
    } else {
    	log_msg_throttle(0.25,"%i FP %hhu R %hhu RL %i %i",initialized, outputs & PIN_FP, outputs & PIN_READY, (int) outputs & (PIN_LS0 | PIN_LS1),runlevel);
    }
#endif
*/

    //Write Changes
    for (i = 0; i < NUM_MECH; i++) {
    	device0->mech[i].outputs = outputs;
    }

    counter++;
}

void updateAtmelInputs(struct device device0, int runlevel)
{
    //unsigned char inputs;
    int PLCState;

    //Update PLC State
    PLCState = (device0.mech[0].inputs & (PIN_PS0 | PIN_PS1)) >> 6;

    //  printk("Mech0.inputs: %#x\n",device0.mech[0].inputs);
    //  static int j;
    //if (j++ % 1000 == 0)
    //  printk("PLC State is %d\n",PLCState);
    //if (PLCState != runlevel)
    // printk("RunLevels on Linux Box and PLC do not match!!!\n");
}
