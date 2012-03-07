/*
 * get_USB_packet.c
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

#include "get_USB_packet.h"

extern unsigned long int gTime;

extern USBStruct USBBoards;

/**
 * getUSBPackets() - Takes data from USB packet(s) and uses it to fill the
 *   DS0 data structure.
 *
 * inputs - device - the data structure to fill
 *
 */
void getUSBPackets(struct device *device0)
{
    int i;
    int err=0;

    //Loop through all USB Boards
    for (i = 0; i < USBBoards.activeAtStart; i++)
    {
        err = getUSBPacket( USBBoards.boards[i], &(device0->mech[i] ) );
        if (  err == -USB_WRITE_ERROR)
        {
            log_msg("Error (%d) reading from USB Board %d on loop %d!\n", err, USBBoards.boards[i], gTime);
        }
    }
}

/**
 * getUSBPacket() - Takes data from a USB packet and uses it to fill the
 *   DS0 data structure.
 *
 * inputs - mechanism - the data structure to fill
 *          id - the USB board to read from
 *
 * output - returns success of procedure
 *
 */
int getUSBPacket(int id, struct mechanism *mech)
{
    int result, type;
    unsigned char buffer[MAX_IN_LENGTH];

    //Read USB Packet
    result = usb_read(id,buffer,IN_LENGTH);

    // -- Check for read errors --
    ///TODO: Fix error codes and error handling
    //No Packet found
    if (result == 0)
        return result;

    //Device Busy error
    else if (result == -EBUSY)
        return result;

    //Packet incorrect length
    else if ((result > 0) && (result != IN_LENGTH))
        return result;

    //Unknown error
    else if (result < 0)
        return result;

    // -- Good packet so process it --

    type = buffer[0];

    //Load in the data from the USB packet
    switch (type)
    {
        //Handle and Encoder USB packet
    case ENC:
        processEncoderPacket(mech, buffer);
        break;
    }

    return 0;
}

void processEncoderPacket(struct mechanism* mech, unsigned char buffer[])
{
    int i, numChannels;
    int encVal;

    //Determine channels of data received
    numChannels = buffer[1];

    //Get the input pin status
#ifdef RAVEN_I
    mech->inputs = ~buffer[2];
#else
    mech->inputs = buffer[2];
#endif

    //Loop through and read data for each channel
    for (i = 0; i < numChannels; i++)
    {
        //Load encoder values
        encVal = processEncVal(buffer, i);
        mech->joint[i].enc_val = encVal;
    }
}
