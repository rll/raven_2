/*
 * put_USB_packet.h
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

//RTAI + Linux include files
//#include <linux/module.h> //used for jiffies
//#include <rtai.h>

//Include files
#include "struct.h" /*Includes DS0, DS1, DOF_type*/
#include "dof.h"
#include "USB_init.h"

/* USB packet lengths */
#define IN_LENGTH          27 /* 27 with input pins */

//Function prototypes
void getUSBPackets(struct device *device0);
int getUSBPacket(int id, struct mechanism *mech);
void processEncoderPacket(struct mechanism *mech, unsigned char buffer[]);
