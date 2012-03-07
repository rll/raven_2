/*
 *  FILE: GravComp.c
 *
 *  Author: Hawkeye King
 *  Equations: Mitch Lum
 *  Additional: Ko Seong-Young
 *
 *  I implement gravity compensation for the RAVEN 7 robot.
 *
 *  Update 26-August-2011 By Hawkeye
 *       Modified to work with Raven_II.  Code taken from UCSC system.
 */

#include "grav_comp.h"

#define c1 cos(pos_j0)
#define s1 sin(pos_j0)
#define c2 cos(pos_j1)
#define s2 sin(pos_j1)
#define d_4 pos_j3

#define a1 ALPHA_12
#define b1 ALPHA_23

#define m1 MASS_1
#define m2 MASS_2
#define m3 MASS_3

extern int NUM_MECH;

int gravComp(struct device *device0)
{
    float tau_d_grav_joint0 = 0.0;//shoulder
    float tau_d_grav_joint1 = 0.0;//elbow

    //	float tau_d_grav_joint3 = 0.0; //insertion is not calculated
    float grav1 = 0;
    float grav2 = 0;
    float pos_j0, pos_j1, pos_j3; //current joint position (rad and mm)

    for (int i=0;i<NUM_MECH;i++)
    {
        pos_j0 = device0->mech[i].joint[0].jpos-(BASE_ROTATION_ANGLE*PI/180.0);
        pos_j1 = device0->mech[i].joint[1].jpos;
        pos_j3 = device0->mech[i].joint[3].jpos;

        grav1 = 0;
        grav2 = 0;

        grav1 = -m1*(G*s1*cm1x-G*c1*cos(a1)*cm1y-G*c1*sin(a1)*cm1z);
        grav1 += -m2*((G*s1*c2+G*c1*cos(a1)*s2)*cm2x+(G*s1*s2*cos(b1)-G*c1*cos(a1)*c2*cos(b1)+G*c1*sin(a1)*sin(b1))*cm2y+(G*s1*s2*sin(b1)-G*c1*cos(a1)*c2*sin(b1)-G*c1*sin(a1)*cos(b1))*cm2z);
        grav1 +=  -m3*((G*s1*c2+G*c1*cos(a1)*s2)*cm3x+(G*s1*s2*cos(b1)-G*c1*cos(a1)*c2*cos(b1)+G*c1*sin(a1)*sin(b1))*cm3y+(G*s1*s2*sin(b1)-G*c1*cos(a1)*c2*sin(b1)-G*c1*sin(a1)*cos(b1))*(d_4+cm3z));

        grav2 = -m2*((G*c1*s2+G*s1*cos(a1)*c2)*cm2x+(-G*c1*c2*cos(b1)+G*s1*cos(a1)*s2*cos(b1))*cm2y+(-G*c1*c2*sin(b1)+G*s1*cos(a1)*s2*sin(b1))*cm2z);
        grav2 +=  -m3*((G*c1*s2+G*s1*cos(a1)*c2)*cm3x+(-G*c1*c2*cos(b1)+G*s1*cos(a1)*s2*cos(b1))*cm3y+(-G*c1*c2*sin(b1)+G*s1*cos(a1)*s2*sin(b1))*(d_4+cm3z));


        tau_d_grav_joint0 = grav1*coeff_tau0;
        tau_d_grav_joint1 = grav2*coeff_tau1;

        if (i ==0 || i ==2)
        {
            device0->mech[i].joint[0].tau_d -= tau_d_grav_joint0;
            device0->mech[i].joint[1].tau_d -= tau_d_grav_joint1;
        }
        else if (i ==1 || i ==3)
        {
            device0->mech[i].joint[0].tau_d += tau_d_grav_joint0;
            device0->mech[i].joint[1].tau_d += tau_d_grav_joint1;
        }
    }

    return 0;
}
