/*
 * USB_init.h
 *
 */

#ifndef __USB_INIT_H__
#define __USB_INIT_H__

//Include files
#include "log.h"
#include <stdio.h>
#include "defines.h"
#include "struct.h"
#include <sys/io.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <vector>

//RTAI + LINUX include files
//#include <linux/kernel.h>
//#include <linux/module.h>
//#include <linux/delay.h>
//#include <rtai.h>

#define MAX_BOARD_COUNT 10 ///Maximum number of usb boards

/* USB packet lengths */
#define OUT_LENGTH      (3+MAX_DOF_PER_MECH*2) /* (3+8*2) w/ output pins */

typedef struct
{
    std::vector <int>    boards;   /// Vector of serial numbers
    int activeAtStart;    /// Number of active boards

} USBStruct;


//Defines

#define MAX_ERROR_COUNT   50
#define USB_WRITE_ERROR   1
#define USB_READ_ERROR    2
#define USB_BUSY_ERROR    3

#define MAX_LOOPS         10
#define USB_INIT_ERROR   -1
#define USB_RESET         1

//Function Prototypes
int USBInit(struct device *device0);
void USBShutdown(void);

void USBShutdown(void);

int usb_read(int id, void *buffer, size_t len);
int usb_write(int id, void *buffer, size_t len);

int usb_reset_encoders(int boardid);

int write_zeros_to_board(int boardid);

#endif
