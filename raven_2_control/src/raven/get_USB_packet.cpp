/*
 * get_USB_packet.c
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

#include "get_USB_packet.h"

#include <ros/ros.h>

#include <raven/state/device.h>
#include <sstream>
#include <raven/util/timing.h>

extern unsigned long int gTime;

extern USBStruct USBBoards;

static USBTimingInfo timing;

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

    USBTimingInfo::clear(timing);

#ifdef USE_NEW_DEVICE
    Device::beginCurrentUpdate(ros::Time::now());
#endif


    //Loop through all USB Boards
    for (i = 0; i < USBBoards.activeAtStart; i++)
    {
        err = getUSBPacket( USBBoards.boards[i], &(device0->mech[i] ) );
        if (  err == -USB_WRITE_ERROR)
        {
            log_msg("Error (%d) reading from USB Board %d (%s) on loop %d!\n", err, USBBoards.boards[i], armNameFromSerial(USBBoards.boards[i]).c_str(), gTime);
        }
    }

#ifdef USE_NEW_DEVICE
    Device::finishCurrentUpdate();
#endif

    timing.mark_get_packet_intermediate_final();
    timing.mark_process_packet_intermediate_final();

    USBTimingInfo::mark_loop_end();
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

    timing.mark_get_packet_start();
    //Read USB Packet
    result = usb_read(id,buffer,IN_LENGTH);
    timing.mark_get_packet_intermediate();

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

    timing.mark_process_packet_start();
    //Load in the data from the USB packet
    switch (type)
    {
        //Handle and Encoder USB packet
    case ENC:
        processEncoderPacket(mech, buffer);
        break;
    }
    timing.mark_process_packet_intermediate();

    return 0;
}

void processEncoderPacket(struct mechanism* mech, unsigned char buffer[])
{
#ifdef USE_NEW_DEVICE
	static MotorList motorsForUpdating;
#endif
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

#ifdef USE_NEW_DEVICE
    ArmPtr arm = Device::currentNoClone()->getArmById(mech->type);
    arm->motorFilter()->getMotorsForUpdate(motorsForUpdating);
#endif

    //Loop through and read data for each channel
    for (i = 0; i < numChannels; i++) {
        //Load encoder values
        encVal = processEncVal(buffer, i);
        mech->joint[i].enc_val = encVal;

#ifdef USE_NEW_DEVICE
        if (i == 3) {
        	continue;
        }
        int motor_ind = i<=2 ? i : i-1;

        //arm->motor(motor_ind)->setEncoderValue(encVal);
        motorsForUpdating[motor_ind]->setEncoderValue((arm->isGold()?-1:1)*encVal,true);
#endif
    }

}
