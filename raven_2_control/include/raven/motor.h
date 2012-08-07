/*
 * motor.h - contains constants involving the EC32 and EC40 Motors
 * Kenneth Fodero
 *
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "struct.h"

// RAVEN_I motors
// EC 40 motor constants
#define T_PER_AMP_EC40  0.0764 /* Nm/A */
#define    I_CONT_EC40  2.5    /* A */
#define     I_MAX_EC40  10     /* A */
// EC 32 motor constants
#define T_PER_AMP_EC32  0.040  /* Nm/A */
#define    I_CONT_EC32  1.667  /* A */
#define     I_MAX_EC32  10     /* A */
#define ENC_CNTS_PER_REV_R_I 2000.0

// RAVEN_II motors
// RE 40 motor constants
#define T_PER_AMP_RE40  0.0603 /* Nm/A - From datasheet*/
#define    I_CONT_RE40  3.12   /* A - From datasheet*/
#define     I_MAX_RE40  10     /* A */
// RE 30 motor constants
#define T_PER_AMP_RE30  0.0538 /* Nm/A From datasheet*/
#define    I_CONT_RE30  1.72   /* A - From  datasheet*/
#define     I_MAX_RE30  10     /* A why? */
#define ENC_CNTS_PER_REV_R_II 4000.0

// Motor constants
#define   T_PER_AMP_BIG_MOTOR  T_PER_AMP_RE40
#define      I_CONT_BIG_MOTOR  I_CONT_RE40
#define       I_MAX_BIG_MOTOR  I_MAX_RE40

#define T_PER_AMP_SMALL_MOTOR  T_PER_AMP_RE30
#define    I_CONT_SMALL_MOTOR  I_CONT_RE30
#define     I_MAX_SMALL_MOTOR  I_MAX_RE30

/// Encoder ticks per motor revolution
#define ENC_CNTS_PER_REV ENC_CNTS_PER_REV_R_II

void getMotorIMax(int joint, float *iMax);
void getMotorTR(int joint, float *tr);
int getMotorType(int joint);
void getMotorTF(int joint, float *tfMotor);



#endif
