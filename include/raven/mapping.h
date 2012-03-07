/*
 *
 *mapping.h - contains constants involving mapping from master to slave
 *Mitch Lum, July 25, 2006
 *BioRobotics Lab
 *
*/


//#include <rtai.h>
// Hack to get rid of annoying compiler warnings w/ math.h
/*#ifdef __attribute_used__
#undef __attribute_used__
#endif
#ifdef __attribute_pure__
#undef __attribute_pure__
#endif*/

#include <math.h>

#include "struct.h"
#include "defines.h"
#include <tf/transform_datatypes.h>

//Rotation about Ymaster into Slave Frame
//green arm using +1.5707 was 45 deg off in actual, so we hacked this number
#define Y_ROT_GREEN_ARM -1.5707
#define Y_ROT_GOLD_ARM 1.5707

void masterToSlave(struct position*, int);
void fromITP(struct position*, btQuaternion&, int);
