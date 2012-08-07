/*
 * update_atmel_io.c
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

#include "struct.h"
#include "defines.h"

//Bit Defines
#define BIT0  0x01
#define BIT1  0x02
#define BIT2  0x04
#define BIT3  0x08
#define BIT4  0x10
#define BIT5  0x20
#define BIT6  0x40
#define BIT7  0x80

//Output Pins

#ifdef RAVEN_I
#define PIN_LS0    BIT0
#define PIN_LS1    BIT1
#define PIN_FP     BIT2
#define PIN_READY  BIT3
#define PIN_WD     BIT4

#else
#define PIN_LS0    BIT0
#define PIN_LS1    BIT1
#define PIN_FP     BIT2
#define PIN_READY  BIT3
#define PIN_WD     BIT4
#endif


//Input Pins
#define PIN_PS0    BIT6    // state bit 0  (LSB)
#define PIN_PS1    BIT7    // state bit 1  (MSB)

//Function Prototypes
void updateAtmelOutputs(struct device *device0, int runlevel);
void updateAtmelInputs(struct device device0, int runlevel);
