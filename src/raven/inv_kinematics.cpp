/*******************************
 *
 *  File: inv_kinmatics.c
 *
 *  Calculate the inverse kinematics from a point P (px, py,pz)
 *  expressed in base frame
 *
 **********************************/

/**
*  Old inverse kinematics for Raven_I removed.
*  See versions prior to 24-Aug-2011 for reference.
*/


#include <math.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include "hmatrix.h"
#include "inv_kinematics.h"

#define EPS2 0.00001
#define EPS 0.01

extern unsigned long int gTime;
extern int NUM_MECH;
int inv_kin_last_err = 0;

int check_joint_limits1(struct mechanism*);
int check_joint_limits2(struct mechanism*);

/**
 * invKin - wrapper function that checks for correct runlevel
 *   and calls invMechKin for each mechanism in device
 *
 * \param device0 pointer to device struct
 * \param runlevel current runlevel
 *
 * Note the following special cases:
 *  If RL = RL_PEDAL_DN, pos_d is set to pos and we want that to propogate and use this function (using this function)
 *  If RL = RL_INIT,     jpos_d is set by Auto init function.  We don't want to overwrite here, so we return.
 */
void invKin(struct device *device0, struct param_pass* currParams)
{
  int ret=0;

#ifdef AUTO_INIT  // EE449 Return if we are in run level 1.2: auto initialize
  int runlevel = currParams->runlevel;
  int sublevel = currParams->sublevel;
  // Note that in RL 1.2 we are running the joint-level auto-init routine, so we don't want invKin to update jpos_d.  So we return here.
  if(runlevel == RL_INIT && sublevel == SL_AUTO_INIT) {
    return;
  }
#endif

  //Run inverse kinematics for each mech
  for (int i = 0; i < NUM_MECH; i++) {
    if  ( (ret=invMechKin( &(device0->mech[i]) )) < 0)
//    	log_msg("inv_kin failed with %d", ret);
        return;
  }
}



/**

    Inverse kinematics have been reformulated for UW Raven II Build
    Lee White
    Hawkeye King

    1/8/2012

* Inverse kinematics equations for Raven_II robot
*  Preconditions:
*      pos_d stores the desired end effector position
*      ori_d stores the desired end effector orientation
*  Postcondition
*      jpos_d stores the desired joint angles
*
*  Previously the function was defined like this, but not anymore:
*   Input pos_d, ori_d
*   Output jpos_d joint angles
*/

int invMechKin_pos(struct mechanism *mech);
int invMechKin_ori(struct mechanism *mech);

int invMechKin(struct mechanism *mech)
{
    if (invMechKin_pos(mech) > 0)
    {
        // enforce joint limits on position DoF
        check_joint_limits1(mech);
        invMechKin_ori(mech);
        // enforce joint limits on tool DoF
        check_joint_limits2(mech);
        return 1;
    }
    else
    {
        log_msg("no_ori");
        return 0;
    }
}

int invMechKin_pos(struct mechanism *mech)
{
    struct position    *pos_d = &(mech->pos_d);

    // output angles for robot
    float j1; //Shoulder
    float j2; //Elbow
    float d3; //Insertion

	// desired tip position
	float Px = (pos_d->x) / MICRON_PER_M;
	float Py = (pos_d->y) / MICRON_PER_M;
	float Pz = (pos_d->z) / MICRON_PER_M;

	/// Solve for d3 (insertion) - always negative
	d3 = - sqrt(SQR(Px) + SQR(Py) + SQR(Pz));

    // Avoid mechanism singularity at d3=0
    const float min_insertion=0.001;
	if (fabs(d3) < min_insertion)
	{
	    return 0;
	}

	/// ** Solve for j2 (Elbow) **
	float cbe=1, sbe=0, cal=1, sal=0;
	float c2,s2;
    if (mech->type == GOLD_ARM)
    {
        cbe = cos(M_PI - A23); 	// cos(Pi - beta)
        sbe = sin(M_PI - A23);  // sin(Pi - beta)
        cal = cos(-A12);        // cos(-alpha)
        sal = sin (-A12);       // sin(-alpha)
        c2 = (Pz - cbe * cal * d3)/( - sbe *  d3 * sal);
    }
    else
    {
        cbe = cos(A23);  // cos(beta)
        sbe = sin(A23);  // sin(beta)
        cal = cos(A12);  // cos(alpha)
        sal = sin(A12);  // sin(alpha)
        c2 = (Pz + cbe * cal * d3)/( sbe *  d3 * sal);
    }

    if (c2>1) c2=1;
    else if (c2<-1) c2=-1;

	// s2 is always positive, since 0<j2<180
	// This resolves multiple solutions
	s2 = sqrt(1-(c2*c2));
	j2 = atan2f(s2,c2);


	/// ** Solve for j1 (Shoulder) **
	float s1,c1;
    float a,b;
    if (mech->type == GOLD_ARM)
    {
        a = sbe*d3*s2;
        b = sbe*d3*c2*cal + cbe*d3*sal;
        c1 = (Px*a - Py*b) / (a*a + b*b);
        s1 = (Px*b + Py*a) / (a*a + b*b);
        s1 = fabs(s1); // s1 always positive, since 0<j1<90; resolves multiple sol'n
        c1 = fabs(c1); // c1 always positive, since 0<j1<90;
        j1 = atan2f ( s1, c1 ) - base_tilt;
    }
    else
    {
        s2=-s2;
        a = sbe*d3*s2;
        b = sbe*d3*c2*cal + cbe*d3*sal;

        c1 = (Px*a - Py*b) / (a*a + b*b);
        s1 = (Py*a + Px*b) / (-(a*a) - (b*b));
        s1 = fabs(s1); // s1 always positive, since 0<j1<90;
        c1 = fabs(c1); // c1 always positive, since 0<j1<90;
        j1 = atan2f ( s1, c1 ) - base_tilt;
    }

	// Now have solved for th1, th2, d3 ...
    mech->joint[SHOULDER].jpos_d = j1;
    mech->joint[ELBOW].jpos_d    = j2;
    mech->joint[Z_INS].jpos_d    = d3;

	return 1;   // actually has a solution
}

/**
*   Calculate the tool DOF angles from the desired orientation
*
*   Assumes that the shoulder, elbow, z have been calculated from desired position.
*
*   The rotation from base frame to tool frame is premultiplied to the
*   desired transform.  The resulting rotation is used to calculate the desired
*   tool angles.
*
*   Desired transform TD_0_6:  TD_3_0 * TD_0_6 = TD_3_6
*   TD_3_6 is then the desired transform from tool frame to tip.
*/
int invMechKin_ori(struct mechanism *mech)
{
    struct orientation   *ori_d = &(mech->ori_d);

    double j1 = mech->joint[SHOULDER].jpos_d;
	double j2 = mech->joint[ELBOW].jpos_d;
	double j3 = mech->joint[Z_INS].jpos_d;

    btMatrix3x3 xf;
//    (ori_d->R[0][0], ori_d->R[0][1], ori_d->R[0][2],
//                       ori_d->R[1][0], ori_d->R[1][1], ori_d->R[1][2],
//                       ori_d->R[2][0], ori_d->R[2][1], ori_d->R[2][2]);
	// copy R matrix
	// TODO: use above commented code.
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			xf[i][j] = ori_d->R[i][j];

    // variable DH Parameters for gold and green arms
    double go_dh_d[3]  = {0,              0,      j3};
    double go_dh_th[3] = {j1 + base_tilt, j2,     0};

    double gr_dh_d[3]  = {0,              0,      j3};
    double gr_dh_th[3] = {j1 + base_tilt, j2,     0};

    double dh_al[6];
    double dh_a[6];
    double dh_d[3];
    double dh_th[3];

    if (mech->type == GOLD_ARM_SERIAL)
    {
        memcpy(dh_al, go_dh_al, 6*sizeof(double));
        memcpy(dh_a,  go_dh_a,  6*sizeof(double));
        memcpy(dh_d,  go_dh_d,  3*sizeof(double));
        memcpy(dh_th, go_dh_th,  3*sizeof(double));
    }
    else if (mech->type == GREEN_ARM_SERIAL)
    {
        memcpy(dh_al, gr_dh_al, 6*sizeof(double));
        memcpy(dh_a,  gr_dh_a,  6*sizeof(double));
        memcpy(dh_d,  gr_dh_d,  3*sizeof(double));
        memcpy(dh_th, gr_dh_th, 3*sizeof(double));
    }

    // Generate transformation matrices from DH parameters
    btMatrix3x3 temp;
    btMatrix3x3 Rot;
    Rot.setIdentity();

    btScalar cti, sti;  // cos(thetai), sin(thetai)
    btScalar cai, sai;  // cos(alphai), sin(alphai)

    /// Calculate the transform from base frame to tool frame
    for (int i=0;i<3;i++)
    {
        cti = cos(dh_th[i]);
        sti = sin(dh_th[i]);
        cai = cos(dh_al[i]);
        sai = sin(dh_al[i]);

        temp[0][0] = cti;
        temp[0][1] = -sti;
        temp[0][2] = 0;

        temp[1][0] = sti * cai;
        temp[1][1] = cti * cai;
        temp[1][2] = -sai;

        temp[2][0] = sti * sai;
        temp[2][1] = cti * sai;
        temp[2][2] = cai;

        Rot = Rot * temp;
    }

    /// Get the desired transform in tool frame
    /// Premultiply by inverse transform from base frame to tool frame
    xf = Rot.inverse() * xf;

//	for (int i = 0; i < 3; i++)
//		for (int j = 0; j < 3; j++)
//			mech->ori_d.R[i][j] = xf[i][j];

    float j4[2];
    float j5[2];
    float j6[2];
    float j4_check, min_j4;;

    /// ** Solve j4 (tool roll) **
    if (mech->type == GOLD_ARM)
    {
        j4[0] = atan2f( -xf[1][2], -xf[0][2] );
        j4[1] = atan2f( xf[1][2], xf[0][2] );
        j4_check = mech->joint[TOOL_ROT].jpos - M_PI/2;
    }
    else
    {
        j4[0] = atan2f( -xf[1][2], xf[0][2] );
        j4[1] = atan2f( xf[1][2], -xf[0][2] );
        j4_check = mech->joint[TOOL_ROT].jpos - M_PI/2;
    }

    // select between multiple solutions by the smallest difference from current angle.
    min_j4 = j4[0];
    int whichcase=0;
    if ( fabsf(j4_check - j4[1]) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[1];
        whichcase=1;
    }
    //check rollover conditions
    if ( fabsf(j4_check - (j4[1]+2*M_PI)) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[1]+2*M_PI;
        whichcase=2;
    }
    if ( fabsf(j4_check - (j4[0]+2*M_PI)) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[0]+2*M_PI;
        whichcase=3;
    }
    if ( fabsf(j4_check - (j4[1]-2*M_PI)) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[1]-2*M_PI;
        whichcase=4;
    }
    if ( fabsf(j4_check - (j4[0]-2*M_PI)) < fabsf(j4_check - min_j4 ))
    {
        min_j4 = j4[0]-2*M_PI;
        whichcase=5;
    }
//
//    if (mech->type == GREEN_ARM)
//    {
//        log_msg("cs:%d j4c:%0.3f\t 0:%0.3f\t 1:%0.3f\t 2:%0.3f\t 3:%0.3f\t 4:%0.3f\t 5:%0.3f",
//            whichcase, j4_check,
//            j4[0],j4[1],
//            j4[1]+2*M_PI,
//            j4[0]+2*M_PI,
//            j4[1]-2*M_PI,
//            j4[0]-2*M_PI);
//    }

    if (mech->type == GOLD_ARM)
        mech->joint[TOOL_ROT].jpos_d = min_j4 + M_PI/2;
    else
        mech->joint[TOOL_ROT].jpos_d = -min_j4 - M_PI/2;

    ///  ** Solve j5 (wrist) **
    float c4 = cosf(min_j4);
    float s5 = xf[0][2] / c4;
    float c5 = xf[2][2];
    j5[0] = atan2f(-s5,-c5);

    if (mech->type == GOLD_ARM)
        mech->joint[WRIST].jpos_d  = j5[0] - M_PI/2;
    else
        mech->joint[WRIST].jpos_d  = -j5[0] + M_PI/2;

    /// ** Solve j6 (avg grasp) **
//    float s5_sign = (s5 < 0) ? -1:1;
    float s6 = xf[2][1]/-s5;
    float c6 = xf[2][0]/s5;
    j6[0] = atan2f(s6, c6);

    float grasp = ((float)mech->ori_d.grasp)/1000.0f;
    //log_msg("%d:grasp:%0.3f", mech->type, grasp);


    if (mech->type == GOLD_ARM)
    {
        mech->joint[GRASP1].jpos_d = grasp/2;
        mech->joint[GRASP2].jpos_d = grasp/2;
    }
    else
    {
        mech->joint[GRASP1].jpos_d = grasp/2;
        mech->joint[GRASP2].jpos_d = grasp/2;
    }





//    float openangle;
//    if (mech->type == GOLD_ARM)
//        openangle = 45 DEG2RAD; //ori_d->grasp/1000;
//    else
//        openangle = -45 DEG2RAD;//ori_d->grasp/1000;
//
////    log_msg("g(%d):%0.3f, \t (%0.3f)", mech->type, openangle, (float)mech->ori_d.grasp/1000);
//    if (mech->type == GOLD_ARM)
//    {
//        mech->joint[GRASP1].jpos_d = j6[0]+openangle/2;
//        mech->joint[GRASP2].jpos_d = j6[0]-openangle/2;
//    }
//    else
//    {
//        mech->joint[GRASP1].jpos_d = j6[0]-openangle/2;
//        mech->joint[GRASP2].jpos_d = j6[0]+openangle/2;
//    }

    mech->joint[TOOL_ROT].jpos_d = 0;
    mech->joint[WRIST].jpos_d = 0;

    return 1;
}







/**
* Check for joint.angle_d exceeding joint angles.
*     --> position
*/
int check_joint_limits1(struct mechanism *_mech)
{


    int limits = 0;
	if (_mech->joint[SHOULDER].jpos_d < SHOULDER_MIN_LIMIT)
	{
	    limits++;
	    log_msg("shoulder limit");
		_mech->joint[SHOULDER].jpos_d = SHOULDER_MIN_LIMIT;
	}
	else if (_mech->joint[SHOULDER].jpos_d > SHOULDER_MAX_LIMIT)
	{
	    limits++;
	    log_msg("shoulder limit");
		_mech->joint[SHOULDER].jpos_d = SHOULDER_MAX_LIMIT;
	}
	if (_mech->joint[ELBOW].jpos_d < ELBOW_MIN_LIMIT)
	{
	    limits++;
        log_msg("elbow limit");
		_mech->joint[ELBOW].jpos_d = ELBOW_MIN_LIMIT;
	}
	else if (_mech->joint[ELBOW].jpos_d > ELBOW_MAX_LIMIT)
	{
 	    limits++;
        log_msg("elbow limit");
		_mech->joint[ELBOW].jpos_d = ELBOW_MAX_LIMIT;
	}
	if (_mech->joint[Z_INS].jpos_d < Z_INS_MIN_LIMIT)
	{
	    limits++;
        log_msg("Zins limit");
		_mech->joint[Z_INS].jpos_d = Z_INS_MIN_LIMIT;
	}
	else if (_mech->joint[Z_INS].jpos_d > Z_INS_MAX_LIMIT)
	{
	    limits++;
        log_msg("Zins limit");
		_mech->joint[Z_INS].jpos_d = Z_INS_MAX_LIMIT;
	}

	return limits;
}
/**
* Check for joint.angle_d exceeding joint angles.
*     --> orientation
*/
int check_joint_limits2(struct mechanism *_mech)
{
    int limits=0;

	if (_mech->joint[TOOL_ROT].jpos_d  < TOOL_ROLL_MIN_LIMIT)
	{
	    limits++;
        log_msg("roll limit");
		_mech->joint[TOOL_ROT].jpos_d = TOOL_ROLL_MIN_LIMIT;
	}
	else if (_mech->joint[TOOL_ROT].jpos_d  > TOOL_ROLL_MAX_LIMIT)
	{
	    limits++;
        log_msg("roll limit");
		_mech->joint[TOOL_ROT].jpos_d = TOOL_ROLL_MAX_LIMIT;
	}
	if (_mech->joint[WRIST].jpos_d  < TOOL_WRIST_MIN_LIMIT)
	{
        log_msg("wrist limit");
	    limits++;
		_mech->joint[WRIST].jpos_d = TOOL_WRIST_MIN_LIMIT;
	}
	else if (_mech->joint[WRIST].jpos_d  > TOOL_WRIST_MAX_LIMIT)
	{
	    limits++;
        log_msg("wrist limit");
		_mech->joint[WRIST].jpos_d = TOOL_WRIST_MAX_LIMIT;
	}

	if (_mech->joint[GRASP1].jpos_d  < TOOL_GRASP1_MIN_LIMIT)
	{
        log_msg("grasp1 limit");
	    limits++;
		_mech->joint[GRASP1].jpos_d = TOOL_GRASP1_MIN_LIMIT;
	}
	else if (_mech->joint[GRASP1].jpos_d  > TOOL_GRASP1_MAX_LIMIT)
	{
	    limits++;
        log_msg("grasp1 limit");
		_mech->joint[GRASP1].jpos_d = TOOL_GRASP1_MAX_LIMIT;
	}

	if (_mech->joint[GRASP2].jpos_d  < TOOL_GRASP2_MIN_LIMIT)
	{
        log_msg("grasp2 limit");
	    limits++;
		_mech->joint[GRASP2].jpos_d = TOOL_GRASP2_MIN_LIMIT;
	}
	else if (_mech->joint[GRASP2].jpos_d  > TOOL_GRASP2_MAX_LIMIT)
	{
	    limits++;
        log_msg("grasp2 limit");
		_mech->joint[GRASP2].jpos_d = TOOL_GRASP2_MAX_LIMIT;
	}

	return limits;
}


































/**
* Inverse kinematics equations for Raven_II robot
*  Preconditions:
*      pos_d stores the desired end effector position
*      ori_d stores the desired end effector orientation
*  Postcondition
*      jpos_d stores the desired joint angles
*
*  Previously the function was defined like this, but not anymore:
*   Input pos_d, ori_d
*   Output jpos_d joint angles
*/
int invMechKin_old(struct mechanism *mech)
{
    int robot;
    if (mech->type == GOLD_ARM){
        robot=0;
    }
    else if (mech->type == GREEN_ARM) {
        robot=1;
    }
    else {
        log_msg("ERROR: @invMechKin Bad mechanism->type");
        return -1;
    }

    struct position    *pos_d = &(mech->pos_d);
    struct orientation *ori_d = &(mech->ori_d);
    //float jpos_d[MAX_DOF_PER_MECH];

	int has_solution = 0;
	int i, j;
	float tip[HMATRIX_SIZE][HMATRIX_SIZE];
	float tip_inv[HMATRIX_SIZE][HMATRIX_SIZE];

// New Variables introduced my LW and HK for new Inv Kinematics

    // Desired Position
    float Px;
    float Py;
    float Pz;

    // helper variables
//    float a;
//    float b;
//    float c;
    float d;

    float cpb;
	float spb;
	float cna;
	float sna;

    // output angles for robot
    float j1; //Shoulder
    float j2; //Elbow
    float d3; //Insertion
//    float j4; //Tool Roll
//    float j5; //Wrist
//    float j6; //Grasper

// End new variables


	float Px_inv;
	float Py_inv;
	float Pz_inv;
	float Pxyz;
	float d44;
	float dd4;
	float s1, c1;
	float s2, c2;
	float s3, c3;
	float s5, c5;
	float s6 = 0;
	float c6 = 0;
	float a, b, c, dd;
	float th31;
	float th32;
	float th1;
	float th2;
	float th3;
	float th5;
	float th6;

	const float _sin_A12 = sin(A12);
	const float _cos_A12 = cos(A12);
	const float _sin_A23 = sin(A23);
	const float _cos_A23 = cos(A23);

	float a5 = _A5;

	float T46[HMATRIX_SIZE][HMATRIX_SIZE];
	float T46_inv[HMATRIX_SIZE][HMATRIX_SIZE];
	float T13[HMATRIX_SIZE][HMATRIX_SIZE];
	float T23[HMATRIX_SIZE][HMATRIX_SIZE];
	float temp[HMATRIX_SIZE][HMATRIX_SIZE];
	float temp_inv[HMATRIX_SIZE][HMATRIX_SIZE];
	float T1[HMATRIX_SIZE][HMATRIX_SIZE];

	float s31;
	float c31;
	float s32;
	float c32;
	float T13_calc1;
	float T13_calc2;

	static int counter = 0;

#define USE_EXACT_ROTATION
	// create tip matrix ====================================================
	// the invkin is solved from the tip matrix =============================
#ifdef USE_EXACT_ROTATION
	// copy R matrix
	for (i = 0; i < 3; i++)
	  for (j = 0; j < 3; j++)
		  tip[i][j] = ori_d->R[i][j];
#else
	hmatrix_from_orientation(ori_d, tip);
#endif

	tip[0][3] = (pos_d->x) / MICRON_PER_M;
	tip[1][3] = (pos_d->y) / MICRON_PER_M;
	tip[2][3] = (pos_d->z) / MICRON_PER_M;
	tip[3][3] = 1.0;

	tip[3][0] = 0.0;
	tip[3][1] = 0.0;
	tip[3][2] = 0.0;


    /*
    Inverse kinematics have been reformulated for Raven II Build
    New formulation follows

    Lee White
    Hawkeye King

    1/8/2012

    */

    // CURRENTLY ONLY VALID FOR LEFT ARM

	// desired tip position
	Px = tip[0][3];
	Py = tip[1][3];
	Pz = tip[2][3];

	// insertion - always negative
	d3 = - sqrt(SQR(Px) + SQR(Py) + SQR(Pz));

	// j2

	// cos(Pi-beta), sin(Pi - beta)
	cpb = cos(M_PI - A23);
	spb = sin(M_PI - A23);

	// cos(-alpha), sin (-alpha)
	cna = cos(-A12);
	sna = sin (-A12);

	c2 = (Pz - cpb * cna * d3)/( - spb *  d3 * cna);
	// may need to specify an output sign for pose
	s2 = 1 * sqrt(1-SQR(c2));

	// correct atan2?
	j2 = atan2f(s2,c2);

	//j1

	//helper terms
	a = spb * d3 * s2;
	b = ( spb * d3 * c2 * cna + cpb * d3 * sna );
	c = spb * d3 * s2;
	d = ( spb * d3 * c2 * cna - cpb * d3 * sna );

	c1 = Px * c / (a * c + Pz + d );
	// choose sign
	s1 = 1 * sqrt( 1 - SQR(c1));

	// correct atan2?
	j1 = atan2f ( s1, c1 );


// END MODIFICATIONS


	// create tip matrix end ================================================

	invOrthMatrix(tip, tip_inv);
	//printf("the inverse recieved matrix\n");
	//printMatrix(tip_inv);
	Px_inv = tip_inv[0][3];
	Py_inv = tip_inv[1][3];
	Pz_inv = tip_inv[2][3];
	Pxyz = SQR(tip[0][3]) + SQR(tip[1][3]) + SQR(tip[2][3]);

	//There are four solutions for d4
	//float d41 = sqrt((SQR(a5)+Pxyz+2*sqrt((SQR(a5)*Pxyz-SQR(Py_inv)*SQR(a5)))));
	//float d42 = -sqrt((SQR(a5)+Pxyz+2*sqrt((SQR(a5)*Pxyz-SQR(Py_inv)*SQR(a5)))));
	//float d43 = sqrt((SQR(a5)+Pxyz-2*sqrt((SQR(a5)*Pxyz-SQR(Py_inv)*SQR(a5)))));
	d44 = -sqrt( SQR(a5) + Pxyz - 2 * sqrt( SQR(a5) * Pxyz - SQR(Py_inv) * SQR(a5) ) );

	//We choose solution number 4
	dd4 = d44;
	// Solved for d4 --------------------------------------------------------------------

	if (ABS(dd4) < EPS) // division by zero
	{
		s5 = 0.0;
	}
	else
	{
		s5 = (Py_inv/dd4);
	}

	if ( (ABS(s5) > 1.0) && (ABS(s5) < 1.0 + EPS) )
	{
		s5 = ABS(s5)/s5;
	}
	else if (ABS(s5) > 1.0 + EPS)
	{
		log_msg("ik: ABS(s5) > 1.0 + EPS, %f", s5);
		return has_solution;  // has no solution
	}

	th5 = asin(s5);
	//Solved for theta5 -------------------------------------------------------------------

	c5 = sqrt(1.0-SQR(s5));
	if ((robot == 0)||(robot == 2))
	{
		c6 = Pz_inv/(-c5*dd4+a5);
		s6 = Px_inv/(-c5*dd4+a5);
	}
	if ((robot == 1)||(robot == 3))
	{
		c6 = Pz_inv/(-c5*dd4+a5);
		s6 = -Px_inv/(-c5*dd4+a5);
	}

	th6 = atan2(s6, c6);
	//Solved for th6 -------------------------------------------------------------------


	if ((robot == 0) || (robot == 2))
	{
		T46[0][0] = -s5*s6; T46[0][1] = -c5; T46[0][2] = -s5*c6; T46[0][3] = a5*s5;
		T46[1][0] = c6; T46[1][1] = 0; T46[1][2] = -s6; T46[1][3] = 0;
		T46[2][0] = c5*s6; T46[2][1] = -s5; T46[2][2] = c5*c6; T46[2][3] = -a5*c5+dd4;
		T46[3][0] = 0; T46[3][1] = 0; T46[3][2] = 0; T46[3][3] = 1;
	}

	else // if ((robot == 1) || (robot == 3))
	{
		T46[0][0] = -s5*s6; T46[0][1] = c5; T46[0][2] = s5*c6; T46[0][3] = -a5*s5;
		T46[1][0] = -c6; T46[1][1] = 0; T46[1][2] = -s6; T46[1][3] = 0;
		T46[2][0] = -c5*s6; T46[2][1] = -s5; T46[2][2] = c5*c6; T46[2][3] = -a5*c5+dd4;
		T46[3][0] = 0; T46[3][1] = 0; T46[3][2] = 0; T46[3][3] = 1;
	}

	invOrthMatrix(T46, T46_inv);
	mulMatrix(tip, T46_inv, T13);
	//printf("T13 matrix\n");
	//printMatrix(T13);
	c2 = (_cos_A12*_cos_A23+ T13[2][2])/(_sin_A12*_sin_A23);
	s2 = 0;
	//printf("ABS of c2 is %f\n",c2);
	if (ABS(c2) > 1.0 + EPS)
	{
		log_msg("ABS(c2) [%6.3f] > 1.0 + EPS", c2);
		return has_solution;   // has no solution
	}
	else if (ABS(c2) > 1.0)
	{
		log_msg("ABS(c2) [%6.3f] > 1.0 < 1.0 + EPS", c2);
		c2 = ABS(c2)/c2; // make it 1 with the right sign
	}
	s2 = sqrt(1-SQR(c2));
	th2 = atan2(s2,c2);
	//Solved for theta2 -------------------------------------------------------------------

	a = _sin_A12*s2;
	b = _sin_A12*c2*_cos_A23+_cos_A12*_sin_A23;
	c = T13[2][1];
	dd = SQR(b)+SQR(a)-SQR(c);

	if (dd < 0.0)
	{
		if (ABS(dd) < EPS)
		{
			dd = 0;
			log_msg("dd = 0, %f", dd);
		}
		else
		{
			log_msg("ABS(b*b+a*a-c*c)<EPS, %f < %f", ABS(dd), EPS);
			return has_solution;  // has no solution
		}
	}

	if (ABS(a+c) < EPS2)
	{
		log_msg("ABS(a+c) < %d", EPS);
		a = c = EPS2;
		return has_solution;  // has no solution
	}

	//There are two solutions for th3
	th31 = 2.0*atan((b+sqrt(dd))/(a+c));
	th32 = 2.0*atan((b-sqrt(dd))/(a+c));

	//We can choose by selection of other unit in matrix
	s31 = sin(th31);
	c31 = cos(th31);
	s32 = sin(th32);
	c32 = cos(th32);
	T13_calc1 = 0;
	T13_calc2 = 0;

	if ((robot == 0)||(robot == 2))
	{
		T13_calc1 = -_sin_A12*s2*s31+(_sin_A12*c2*_cos_A23+_cos_A12*_sin_A23)*c31;
		T13_calc2 = -_sin_A12*s2*s32+(_sin_A12*c2*_cos_A23+_cos_A12*_sin_A23)*c32;
	}
	else //if ((robot == 1)||(robot == 3))
	{
		T13_calc1 = _sin_A12*s2*s31-(_sin_A12*c2*_cos_A23+_cos_A12*_sin_A23)*c31;
		T13_calc2 = _sin_A12*s2*s32-(_sin_A12*c2*_cos_A23+_cos_A12*_sin_A23)*c32;
	}

	th3 = 0;

	if ( ABS(T13_calc1 - T13[2][0]) < ABS(T13_calc2 - T13[2][0]) )
	{
		th3 = th31;
	}
	else
	{
		th3 = th32;
	}

	// Solved for th3 -----------------------------------------------------------------

	c3 = cos(th3);
	s3 = sin(th3);

	if ((robot == 0)||(robot == 2))
	{
		T23[0][0] = c2*s3+s2*_cos_A23*c3;
		T23[0][1] = -c2*c3+s2*_cos_A23*s3;
		T23[0][2] = s2*_sin_A23;
		T23[0][3] = 0;

		T23[1][0] = -s2*s3+c2*_cos_A23*c3;
		T23[1][1] = s2*c3+c2*_cos_A23*s3;
		T23[1][2] = c2*_sin_A23;
		T23[1][3] = 0;

		T23[2][0] = -_sin_A23*c3;
		T23[2][1] = -_sin_A23*s3;
		T23[2][2] = _cos_A23;
		T23[2][3] = 0;

		T23[3][0] = 0;
		T23[3][1] = 0;
		T23[3][2] = 0;
		T23[3][3] = 1;
	}

	if ((robot == 1)||(robot == 3))
	{
		T23[0][0] = c2*s3+s2*_cos_A23*c3;
		T23[0][1] = c2*c3-s2*_cos_A23*s3;
		T23[0][2] = -s2*_sin_A23;
		T23[0][3] = 0;

		T23[1][0] = s2*s3-c2*_cos_A23*c3;
		T23[1][1] = s2*c3+c2*_cos_A23*s3;
		T23[1][2] = c2*_sin_A23;
		T23[1][3] = 0;

		T23[2][0] = _sin_A23*c3;
		T23[2][1] = -_sin_A23*s3;
		T23[2][2] = _cos_A23;
		T23[2][3] = 0;

		T23[3][0] = 0;
		T23[3][1] = 0;
		T23[3][2] = 0;
		T23[3][3] = 1;
	}

	mulMatrix(T23, T46, temp);
	invOrthMatrix(temp, temp_inv);
	mulMatrix(tip, temp_inv, T1);
	//printMatrix(T1);


	if ((robot == 0) || (robot == 2))
	{
		c1 = T1[0][0];
		s1 = T1[1][0];
	}
	else //if ((robot == 1) || (robot == 3))
	{
		c1 = -T1[0][0];
		s1 = T1[1][0];
	}

	//th1 final
	th1 = atan2(s1,c1);
	//solved for theta1 ----------------------------------------------------

	inv_kin_last_err = 0;
	// revised by JiMa to fix the problem that robot stops working when reaching joint limit

	if (th1 < SHOULDER_MIN_LIMIT)
	{
		th1 = SHOULDER_MIN_LIMIT;
		inv_kin_last_err = 1;
	}
	else if (th1 > SHOULDER_MAX_LIMIT)
	{
		th1 = SHOULDER_MAX_LIMIT;
		inv_kin_last_err = 2;
	}

	if (th2 < ELBOW_MIN_LIMIT)
	{
		th2 = ELBOW_MIN_LIMIT;
		inv_kin_last_err = 3;
	}
	else if (th2 > ELBOW_MAX_LIMIT)
	{
		th2 = ELBOW_MAX_LIMIT;
		inv_kin_last_err = 4;
	}

	if (th3 < TOOL_ROLL_MIN_LIMIT)
	{
		th3 = TOOL_ROLL_MIN_LIMIT;
		inv_kin_last_err = 5;
	}
	else if (th3 > TOOL_ROLL_MAX_LIMIT)
	{
		th3 = TOOL_ROLL_MAX_LIMIT;
		inv_kin_last_err = 6;
	}

	if (dd4 < Z_INS_MIN_LIMIT)
	{
		dd4 = Z_INS_MIN_LIMIT;
		inv_kin_last_err = 7;
	}
	else if (dd4 > Z_INS_MAX_LIMIT)
	{
		dd4 = Z_INS_MAX_LIMIT;
		inv_kin_last_err = 8;
	}

	if (th5 < TOOL_WRIST_MIN_LIMIT)
	{
		th5 = TOOL_WRIST_MIN_LIMIT;
		inv_kin_last_err = 9;
	}
	else if (th5 > TOOL_WRIST_MAX_LIMIT)
	{
		th5 = TOOL_WRIST_MAX_LIMIT;
		inv_kin_last_err = 10;
	}

	// Now have solved for th1, th2, d3 ...
    mech->joint[SHOULDER].jpos_d = th1;
    mech->joint[ELBOW].jpos_d    = th2;
    mech->joint[Z_INS].jpos_d    = dd4;
    mech->joint[TOOL_ROT].jpos_d = th3;
    mech->joint[WRIST].jpos_d    = th5;

	// implementation of simple grasping on and off -------------------------------------
	// it takes several cycles to open or close so do it gradually
	static float delta_grasp[MAX_MECH_PER_DEV] = {0.0, 0.0};
	static int dir[MAX_MECH_PER_DEV] = {1, 1};
	static float delta_target[MAX_MECH_PER_DEV] = {0.0, 0.0};

	if (!ori_d->grasp)
		delta_target[robot] =  45.0 DEG2RAD; // grasp open
	else
		delta_target[robot] = -40.0 DEG2RAD;			 // grasp closed

	if ( ABS(delta_grasp[robot] - delta_target[robot]) > 0.2 DEG2RAD )
	{
		if (delta_grasp[robot] < delta_target[robot])
		{
			dir[robot] = 5;
		}
		else
		{
			dir[robot] = -1;
		}


		if ((dir[robot] == -1) && mech->joint[GRASP1].tau > 0.02)
		{
			log_msg("robot: %d, torque: %5.2f", robot, mech->joint[GRASP1].tau);
			; // do nothing if closing and too much torque
		}
		else
		{
			delta_grasp[robot] += dir[robot] * 0.1 DEG2RAD;
			//delta_target[robot] = delta_grasp[robot]; // stay where you are
		}
	}

	if ((robot == 0) || (robot == 2))
	{
        mech->joint[GRASP1].jpos_d = (th6 + delta_grasp[robot]);
        mech->joint[GRASP2].jpos_d = -(th6 - delta_grasp[robot]);
//      From UCSC R_II. Kept for reference
//		jpos_d[GRASP1] =  (th6 + delta_grasp[robot]);
//		jpos_d[GRASP2] = -(th6 - delta_grasp[robot]);
	}
	else if ((robot == 1) || (robot == 3))
	{
        mech->joint[GRASP1].jpos_d = (th6 - delta_grasp[robot]);
        mech->joint[GRASP2].jpos_d = -(th6 + delta_grasp[robot]);
//      From UCSC R_II. Kept for reference.     Once this code is working properly, this should b3e deleted.

//		jpos_d[GRASP1] =  (th6 - delta_grasp[robot]);
//		jpos_d[GRASP2] = -(th6 + delta_grasp[robot]);
	}
	// ----------------------------------------------------------------------------------


	if (mech->joint[GRASP1].jpos_d < TOOL_GRASP1_MIN_LIMIT)
	{
		mech->joint[GRASP1].jpos_d = TOOL_GRASP1_MIN_LIMIT;
		inv_kin_last_err = 11;
	}
	else if (mech->joint[GRASP1].jpos_d > TOOL_GRASP1_MAX_LIMIT)
	{
		mech->joint[GRASP1].jpos_d = TOOL_GRASP1_MAX_LIMIT;
		inv_kin_last_err = 12;
	}

	if (mech->joint[GRASP2].jpos_d < TOOL_GRASP2_MIN_LIMIT)
	{
		mech->joint[GRASP2].jpos_d = TOOL_GRASP2_MIN_LIMIT;
		inv_kin_last_err = 13;
	}
	else if (mech->joint[GRASP2].jpos_d > TOOL_GRASP2_MAX_LIMIT)
	{
		mech->joint[GRASP2].jpos_d = TOOL_GRASP2_MAX_LIMIT;
		inv_kin_last_err = 14;
	}

	counter++;

	has_solution = 1;
	return has_solution;   // actually has a solution
}

