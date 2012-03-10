/*********************************************
 **
 **
 **  DOF_type.h
 **
 **	DOF types contains a data structure containing parameters
 **     for a DOF that remain static from power on to power off of
 **     the surgical robot.
 **
 *********************************************/

/********************************************
 *
 * DOF_type - Struct for storing data that remains constant
 *  from power on to power off of the surgical robot.
 *
 */

#ifndef __DOF_type__
#define __DOF_type__

#define MAX_WINDOW_SIZE 1000 /* ms */
#define DAC_STORE_SIZE  10 /* s */
#define HISTORY_SIZE    10

struct Window
{
  int length;
  int isFull;
  int first, last;
  float data[MAX_WINDOW_SIZE];

};

struct DOF_type {

  // joint limit in degrees
  float max_position;

  // starting position in degrees
  float home_position;
  
  float speed_limit; // radian / sec


  // encoder counts per revolution
  int enc_cnts;

  int DAC_max;

  //DOF current variables
  float i_max;
  float i_cont;

  //Motor Transmission Ratio
  float TR;

  //Torque per amp - motor dependant
  float tau_per_amp;

  //DAC counts per amp - different for high / low current amps.
  float DAC_per_amp;

  //Controller Gains
  float KP;
  float KD;
  float KI;

  //Old position data
  int filterRdy;
  float old_mpos[HISTORY_SIZE];
  float old_filtered_mpos[HISTORY_SIZE];
  float old_mpos_d[HISTORY_SIZE];

  //Old velocity data
  float old_mvel[HISTORY_SIZE];
  float old_mvel_d[HISTORY_SIZE];

  //Length of time motor has been overdriven
  int overdrive_time;

};


#endif
