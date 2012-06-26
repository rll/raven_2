/*
 * put_USB_packet.c
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

#include "put_USB_packet.h"
#include "USB_init.h"
#include "update_atmel_io.h"

extern unsigned long int gTime;
extern USBStruct USBBoards;

/**
 * putUSBPackets() - Takes data from robot to send to USB board(s)
 *
 * inputs - device - the device data structure
 *
 */
void putUSBPackets(struct device *device0)
{
    //Loop through all USB Boards
    for (int i = 0; i < USBBoards.activeAtStart; i++)
    {
        if (putUSBPacket(USBBoards.boards[i], &(device0->mech[i])) == -USB_WRITE_ERROR)
            log_msg("Error writing to USB Board %d (%s)!\n", USBBoards.boards[i],getArmNameFromSerial(USBBoards.boards[i]).c_str());
    }
}

/**
 * putUSBPacket() - Takes data from mech struct and uses it to fill a USB
 *   packet on specified board
 *
 * inputs - id - the usb board id number (serial#)
 *          mech - the data structure to get data from
 *
 * output - success of the operation
 *
 */
int putUSBPacket(int id, struct mechanism *mech)
{
    int i = 0;
    unsigned char buffer_out[MAX_OUT_LENGTH];

    buffer_out[0]= DAC;        //Type of USB packet
    buffer_out[1]= MAX_DOF_PER_MECH; //Number of DAC channels

    for (i = 0; i < MAX_DOF_PER_MECH; i++)
    {
        //Factor in offset since we are in midrange operation
        mech->joint[i].current_cmd += DAC_OFFSET;

        buffer_out[2*i+2] = (char)(mech->joint[i].current_cmd);
        buffer_out[2*i+3] = (char)(mech->joint[i].current_cmd >> 8);

        //Remove offset
        mech->joint[i].current_cmd -= DAC_OFFSET;
    }

    // Set PortF outputs
    buffer_out[OUT_LENGTH-1] = mech->outputs;

    //Write the packet to the USB Driver
    if (usb_write(id, &buffer_out, OUT_LENGTH )!= OUT_LENGTH)
        return -USB_WRITE_ERROR;

    return 0;
}
