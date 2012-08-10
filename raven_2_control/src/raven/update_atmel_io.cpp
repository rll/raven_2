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

extern int initialized;
extern int soft_estopped;
extern int NUM_MECH;
extern unsigned long int gTime;

void updateAtmelOutputs(struct device *device0, int runlevel)
{
    static int counter;
    unsigned char i, outputs = 0x00;

    //Update Foot Pedal
    if ( (runlevel>1) && (device0->surgeon_mode) )
        outputs |= PIN_FP;

    //Update Ready
    if (initialized)
        outputs |= PIN_READY;

    //Update Linux State
    outputs |= (runlevel & (PIN_LS0 | PIN_LS1));

    //Update WD Timer - if not software triggered
    if ( !soft_estopped )
    {
        if ( counter <= (WD_PERIOD / 2) )
        {
            outputs |= PIN_WD;
        }
        else if (counter >= WD_PERIOD)
        {
            counter = 0;
        }
    }

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
