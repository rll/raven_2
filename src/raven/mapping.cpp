/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University of Washington BioRobotics Laboratory
 *
 * This file is part of Raven 2 Control.
 *
 * Raven 2 Control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven 2 Control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */


/**
 * mapping.c - contains functions involving mapping from master to slave
 * Mitch Lum, July 25, 2006
 * BioRobotics Lab
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
///  Do this using inv(R)*C*R : R= transform, C= increment
///
void fromITP_testing(struct position *delpos, btQuaternion &delrot, int armserial)
{
    const btTransform ITP2Gold ( btMatrix3x3 (0,0,-1,  -1,0,0,  0,1,0), btVector3 (0,0,0) );
    const btTransform ITP2Green( btMatrix3x3 (0,0,-1,  1,0,0,  0,-1,0), btVector3 (0,0,0) );
    btTransform incr (delrot, btVector3(delpos->x, delpos->y, delpos->z));

    if (armserial == GOLD_ARM_SERIAL)
    {
        incr = ITP2Gold  * incr * ITP2Gold.inverse();
    }
    else
    {
        incr = ITP2Green * incr * ITP2Green.inverse();
    }

    log_msg("pre: %d,\t %d,\t %d", delpos->x, delpos->y, delpos->z);
    log_msg("post: %0.3f,\t %0.3f,\t %0.3f", incr.getOrigin()[0], incr.getOrigin()[1], incr.getOrigin()[2]);
    delrot = incr.getRotation();
    delpos->x = (int)(incr.getOrigin()[0]);
    delpos->y = (int)(incr.getOrigin()[1]);
    delpos->z = (int)(incr.getOrigin()[2]);
}

void fromITP(struct position *pos, btQuaternion &rot, int armserial)
{
    const btMatrix3x3 ITP2Gold ( 0,0,-1,  -1,0,0,  0,1,0);
    const btMatrix3x3 ITP2Green( 0,0,-1,  1,0,0,  0,-1,0);
    btVector3 v_pos(pos->x, pos->y, pos->z);
    btVector3 v_rotAx(rot.getAxis());
    btScalar  s_rotAn(rot.getAngle());

    if (armserial == GOLD_ARM_SERIAL)
    {
        // Multiply the position vector by the rotation matrix to convert from ITP frame to R_II gold/green frame
        v_pos   = ITP2Gold * v_pos;
        v_rotAx = ITP2Gold * v_rotAx;
    }
    else
    {
        v_pos   = ITP2Green * v_pos;
        v_rotAx = ITP2Green * v_rotAx;
    }
    pos->x=v_pos[0];
    pos->y=v_pos[1];
    pos->z=v_pos[2];

    // check for degenerate case.  Only a problem at zero.
    if (v_rotAx[0] == 0 && v_rotAx[1] == 0 && v_rotAx[2] == 0)
    {
        v_rotAx[0] = 1;
    }

    btQuaternion newrot(v_rotAx, s_rotAn);
    rot=newrot;

}
