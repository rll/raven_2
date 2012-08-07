
/*
 *
 *mapping.c - contains functions involving mapping from master to slave
 *Mitch Lum, July 25, 2006
 *BioRobotics Lab
 *
*/

#include "mapping.h"
#include "log.h"
#include <iostream>

/**
 *
 * masterToSlave() - express the incrimental change in position
 *	from master to slave coordinate frame
 * Will eventually use X-Y-Z fixed frame rotation from fixed (Master) frame
 *
**/

const int USE_ITP = 1;
void masterToSlave(struct position *pos, int type) // struct mechanism *mech)
{
  double yrot = 0;
  struct position p_m = {pos->x,pos->y,pos->z};
  const double itpRot[3][3] = {{ 0, 1, 0}, {0, 0, -1}, {-1, 0, 0}};
  // Convert from teleoperation frame to local frame.
  if (USE_ITP) {
    p_m.x = itpRot[0][0]*pos->x + itpRot[0][1]*pos->y + itpRot[0][2]*pos->z;
    p_m.y = itpRot[1][0]*pos->x + itpRot[1][1]*pos->y + itpRot[1][2]*pos->z;
    p_m.z = itpRot[2][0]*pos->x + itpRot[2][1]*pos->y + itpRot[2][2]*pos->z;
  }
//
//  if (type == GOLD_ARM_SERIAL) {
//	yrot = Y_ROT_GOLD_ARM;
//	//printk("yrot_Gold_Arm  = %d\n", (int)(yrot*1000));
//  } else if (type == GREEN_ARM_SERIAL)  {
//	yrot = Y_ROT_GREEN_ARM;
//	//printk("yrot_Green_Arm = %d\n", (int)(yrot*1000));
//  } else return;

  yrot = Y_ROT_GREEN_ARM;
  //This is purely a rotation about Ymaster into Yslave for updating correct position increment
  pos->x = cos(yrot)*p_m.x + sin(yrot)*p_m.z;
  pos->y = p_m.y;
  pos->z = -sin(yrot)*p_m.x + cos(yrot)*p_m.z;
//
//  log_msg("updated d1.xd[%d] from: (%d,%d,%d)/(%d,%d,%d)", type,
//          pos->x, pos->y, pos->z,
//          p_m.x, p_m.x, p_m.z);

  return;
}

///
/// Transform a position increment and an orientation increment from ITP coordinate frame
/// into local robot coordinate frame.
///
void fromITP(btVector3& pos, btQuaternion& q, int armserial)
{
	const btMatrix3x3 ITP2Gold_pos(1,0,0, 0,1,0, 0,0,1);
    const btMatrix3x3 ITP2Gold_rotL(0,0,1, 0,-1,0, 1,0,0);
    const btMatrix3x3 ITP2Gold_rotR(0,-1,0, 0,0,1, -1,0,0);
    const btMatrix3x3 ITP2Green( 0,0,-1,  1,0,0,  0,-1,0);

        btMatrix3x3 rot(q);

    if (armserial == GOLD_ARM_SERIAL) {
        // Multiply the position vector by the rotation matrix to convert from ITP frame to R_II gold/green frame
        pos = btMatrix3x3(-1,0,0, 0,0,1, 0,1,0) * pos;
        rot = ITP2Gold_rotL * rot * ITP2Gold_rotR * btMatrix3x3(1,0,0, 0,-1,0, 0,0,-1);
    } else {
        pos = ITP2Green * pos;
        rot = ITP2Green * rot;
    }

    rot.getRotation(q);

}
