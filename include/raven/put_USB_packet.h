/*
 * put_USB_packet.h
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

//Include files
//#include <rtai.h>

#include "struct.h"
#include "defines.h"
#include "USB_init.h"
#include "t_to_DAC_val.h"

//Function prototypes
void putUSBPackets(struct device *device0);
int putUSBPacket(int id, struct mechanism *mech);
