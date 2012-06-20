/*********************************************
*
*
*  teleoperation.h
*
*    I define datastructures representing the
*  information passed between master and slave
*  in teleoperation.
*
*  Based on the wave variables naming schema:
*
*  u_struct passes from master to slave
*  v_struct passes from slave to master
*
*********************************************/

#ifndef TELEOPERATION_H
#define TELEOPERATION_H
#define SURGEON_ENGAGED       1
#define SURGEON_DISENGAGED    0

#include <iostream>

/*
u_struct : structure passed from master to slave.
This struct defines an incremental movment packet type.

sequence     Packet's sequence number
pactyp       protocol version in use
version      Protocol version number  (***SRI)

delx[2]	     position increment
dely[2]
delz[2]
delyaw[2]    Orientation increment
delpitch[2]
delroll[2]
buttonstate[2]
grasp[2]        +32767 = 100% closing torque, -32768 = 100% opening
surgeon_mode    SURGEON_ENGAGED or SURGEON_DISENGAGED  (formerly Pedal_Down or Pedal_UP)
checksum
*/

struct u_struct {
	unsigned int sequence;
	unsigned int pactyp;
	unsigned int version;

	int delx[2];
	int dely[2];
	int delz[2];
    double Qx[2];
    double Qy[2];
    double Qz[2];
    double Qw[2];
	int buttonstate[2];
	int grasp[2];
	int surgeon_mode;
	int checksum;
};

inline void print_u_struct(u_struct* u,int i=0) {
    std::cout << i << " (" << u->delx[i] << "," << u->dely[i] << "," << u->delz[i] << ")"
            << " (" << u->Qx[i] << "," << u->Qy[i] << "," << u->Qz[i] << "," << u->Qw[i] << ") "
            << u->buttonstate[i] << " " << u->grasp[i] << std::endl;
}

inline void print_u_struct_pos(u_struct* u,int i=0) {
    std::cout << i << " (" << u->delx[i] << "," << u->dely[i] << "," << u->delz[i] << ")" << std::endl;
}

/*
v_struct: Return DS from slave to master.
sequence
pactyp        protocol version in use
version       Protocol version number  (***SRI)
fx            X force
fy            Y force
fz            Z force
runlevel      Slave operating state
jointflags    bit flags for each joint limit (up to 16 joints).
checksum
*/
struct v_struct {
	unsigned int sequence;
	unsigned int pactyp;
	unsigned int version;
	int fx;
	int fy;
	int fz;
	int runlevel;
	unsigned int  jointflags;
	int checksum;
};

#endif //teleoperation_h


