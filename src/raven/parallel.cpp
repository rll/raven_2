/*
 * parallel.c 
 *
 * Kenneth Fodero 
 * Biorobotics Lab
 * 2005
 *
 */

#include "parallel.h"
#include "log.h"

void parallelUpdate(int runlevel, int endOfLoop)
{
  unsigned char data = 0x00;

  //Update RL pins
  data |= ((char)runlevel & 0x03);


  //Update Duty Cycle pins
  if (!endOfLoop)
    data |= DUTY_CYCLE_BIT;

	log_msg("parallel B");

  //Write the data
  outb(data, PARALLEL_PORT);
	log_msg("parallel C");

}
