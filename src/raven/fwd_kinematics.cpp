/*******************************
 *
 *  File: fwd_kinmatics.c
 *
 *  Calculate the forward kinematics point P (px, py,pz) from joint angles
 *  expressed in base frame
 *
 *   Updated 25-Aug-2011 by HK
 *     Changed to work for Raven_II.  Code taken from UCSC implementation
 *
 **********************************/

#include <math.h>
#include <tf/transform_datatypes.h>

#include "local_io.h"
#include "fwd_kinematics.h"
#include "inv_kinematics.h"
#include "hmatrix.h"
#include "utils.h"
#include "defines.h"


extern int NUM_MECH;

/*
 * fwdKin - wrapper function that checks for correct runlevel
 *   and calls fwdMechKin for each mechanism in device
 *
 * inputs: device - pointer to device struct
 *         runlevel - current runlevel
 *
 */
void fwdKin(struct device *device0, int runlevel)
{
    //Always run forward kinematics for each mech
    for (int i = 0; i < NUM_MECH; i++) {
    	if (device0->mech[i].type == GOLD_ARM) {
    		fwdMechKinNew(&(device0->mech[i]));
    	} else {
    		fwdMechKin(&(device0->mech[i]));
    	}
    }

    if ((runlevel != RL_PEDAL_DN) && (runlevel != RL_INIT)) {
        // set cartesian pos_d = pos.
        // That way, if anything wonky happens during state transitions
        // there won't be any discontinuities.
        // Note: in init, this is done in setStartXYZ
        for (int m = 0; m < NUM_MECH; m++) {
          device0->mech[m].pos_d.x     = device0->mech[m].pos.x;
          device0->mech[m].pos_d.y     = device0->mech[m].pos.y;
          device0->mech[m].pos_d.z     = device0->mech[m].pos.z;
          device0->mech[m].ori_d.yaw   = device0->mech[m].ori.yaw;
          device0->mech[m].ori_d.pitch = device0->mech[m].ori.pitch;
          device0->mech[m].ori_d.roll  = device0->mech[m].ori.roll;
          device0->mech[m].ori_d.grasp = device0->mech[m].ori.grasp;

          for (int k = 0; k < 3; k++)
              for (int j = 0; j < 3; j++)
                  device0->mech[m].ori_d.R[k][j] = device0->mech[m].ori.R[k][j];

        }
    updateMasterRelativeOrigin( device0 );   // Update the origin, to which master-side deltas are added.
  }
}

void fwdMechKinNew(struct mechanism* mech) {
	float ths_offset, thr_offset;
	if (mech->type == GOLD_ARM) {
		ths_offset = atan(0.3471/0.9014); //from original URDF
		thr_offset = M_PI_2;
	} else {
		//TODO: fix
		log_msg("GREEN ARM KINEMATICS NOT IMPLEMENTED");
		ths_offset = atan(0.3471/0.9014); //from original URDF
		thr_offset = M_PI / 4.;
	}

	float th12 = -A12;
	float th23 = -A23;

	float dw = 0.012;

	float ths = mech->joint[SHOULDER].jpos;
	float the = mech->joint[ELBOW].jpos;
	float   d = mech->joint[Z_INS].jpos;
	float thr = mech->joint[TOOL_ROT].jpos;
	//float thr = fix_angle(mech->joint[TOOL_ROT].jpos - M_PI);
	float thp = mech->joint[WRIST].jpos;
	float thy = (mech->joint[GRASP2].jpos - mech->joint[GRASP1].jpos) / 2;
	//float thy = (mech->joint[GRASP1].jpos - mech->joint[GRASP2].jpos) / 2;
	float grasp = mech->joint[GRASP1].jpos + mech->joint[GRASP2].jpos;

	btTransform ik_world_to_actual_world;
	if (mech->type == GOLD_ARM) {
		ik_world_to_actual_world = btTransform(btMatrix3x3(
				0,1,0,
				-1,0,0,
				0,0,1)).inverse();
	} else {
		//TODO: check
		log_msg("GREEN ARM KINEMATICS NOT IMPLEMENTED");
		ik_world_to_actual_world = btTransform(btMatrix3x3(
				0,-1,0,
				1,0,0,
				0,0,1),
				btVector3(-.2, 0, 0)).inverse();
	}

	btTransform Tw2b(btMatrix3x3(0,-1,0, 0,0,-1, 1,0,0));
	btTransform Zs = Z(ths + ths_offset,0);
	btTransform Xu = X(th12,0);
	btTransform Ze = Z(the,0);
	btTransform Xf = X(th23,0);
	btTransform Zr = Z(-thr + thr_offset,0);
	btTransform Zi = Z(0,-d);
	btTransform Xip(btMatrix3x3(0,-1,0, 0,0,-1, 1,0,0));
	btTransform Zp = Z(thp,0);
	btTransform Xpy(btMatrix3x3(1,0,0, 0,0,-1, 0,1,0),btVector3(dw,0,0));
	btTransform Zy = Z(thy,0);
	btTransform Tg(btMatrix3x3::getIdentity());

	btTransform tool = ik_world_to_actual_world.inverse() * Tw2b * Zs * Xu * Ze * Xf * Zr * Zi * Xip * Zp * Xpy * Zy * Tg;

	/*
		if (_ik_counter % PRINT_EVERY == 0) {
			btVector3 ins = Tw2b * Zs * Xu * Ze * Xf * btVector3(0,0,-1);
			float yaw = atan2(ins.y(),ins.x());
			float pitch = atan2(sqrt(ins.x()*ins.x()+ins.y()*ins.y()), ins.z());

			log_msg("curr roll axis (%0.4f,%0.4f,%0.4f)",
					yaw * 180 / M_PI,pitch * 180 / M_PI,0);

			btVector3 gpt = (Tw2b * Zs * Xu * Ze * Xf * Zr * Zi * Xip * Xpy * Zy * Tg).getOrigin();
			log_msg("gpt (%0.4f,%0.4f,%0.4f)",gpt.x(),gpt.y(),gpt.z());
		}
	 */

	btMatrix3x3 temp = tool.getBasis();
	btVector3 p    = tool.getOrigin();

	mech->pos.x = (int)(p[0] * MICRON_PER_M);
	mech->pos.y = (int)(p[1] * MICRON_PER_M);
	mech->pos.z = (int)(p[2] * MICRON_PER_M);

	// copy R matrix
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			mech->ori.R[i][j] = temp[i][j];
		}
	}

	mech->ori.grasp = grasp*1000;
}




void fwdMechKin(struct mechanism *mech)
{
	double j1 = mech->joint[SHOULDER].jpos;
	double j2 = mech->joint[ELBOW].jpos;
	double j3 = mech->joint[Z_INS].jpos;
	double j4 = mech->joint[TOOL_ROT].jpos;
	double j5 = mech->joint[WRIST].jpos;
	// TODO :: use average grasp position.
	double j6=0;
    if (mech->type == GOLD_ARM_SERIAL)
        j6 = (mech->joint[GRASP1].jpos - mech->joint[GRASP2].jpos) / 2.0;
    else if (mech->type == GREEN_ARM_SERIAL)
        j6 = (mech->joint[GRASP2].jpos - mech->joint[GRASP1].jpos) / 2.0;
    else
    {
        log_msg("unknown mech type in fwd_kin.");
        return;
    }

    mech->ori.grasp = (mech->joint[GRASP1].jpos + mech->joint[GRASP2].jpos)*1000;

    // DH Parameters for gold and green arms
    double go_dh_d[6]  = {0,              0,      j3,        0, 0, 0};
    double go_dh_th[6] = {j1 + base_tilt, j2,     0,          j4+M_PI/2, j5-M_PI/2, j6};

    double gr_dh_d[6]  = {0,              0,      j3,        0, 0, 0};
    double gr_dh_th[6] = {j1 + base_tilt, j2,     0,         j4+M_PI/2, j5+M_PI/2, j6};

    double dh_al[6];
    double dh_a[6];
    double dh_d[6];
    double dh_th[6];

    if (mech->type == GOLD_ARM_SERIAL)
    {
        memcpy(dh_al, go_dh_al, 6*sizeof(double));
        memcpy(dh_a,  go_dh_a,  6*sizeof(double));
        memcpy(dh_d,  go_dh_d,  6*sizeof(double));
        memcpy(dh_th, go_dh_th,  6*sizeof(double));
    }
    else if (mech->type == GREEN_ARM_SERIAL)
    {
        memcpy(dh_al, gr_dh_al, 6*sizeof(double));
        memcpy(dh_a,  gr_dh_a,  6*sizeof(double));
        memcpy(dh_d,  gr_dh_d,  6*sizeof(double));
        memcpy(dh_th, gr_dh_th, 6*sizeof(double));
    }

    // Generate transformation matrices from DH parameters
    btTransform T[6];
    btMatrix3x3 temp;
    btVector3 p(0,0,0);

    btTransform fkin;
    fkin.setIdentity();

    btScalar cti, sti;  // cos(thetai), sin(thetai)
    btScalar cai, sai;  // cos(alphai), sin(alphai)
    for (int i=0;i<6;i++)
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

        p[0] = dh_a[i];
        p[1] = -sai * dh_d[i];
        p[2] = cai * dh_d[i];

        T[i].setBasis(temp);  // set the rotation component of the transform
        T[i].setOrigin(p);    // set the translation component of the transform
    }

    for (int i=0;i<6;i++)
    {
        fkin   =fkin * T[i]; // * T[i];
    }

    temp = fkin.getBasis();
    p    = fkin.getOrigin();

	mech->pos.x = (int)(p[0] * MICRON_PER_M);
	mech->pos.y = (int)(p[1] * MICRON_PER_M);
	mech->pos.z = (int)(p[2] * MICRON_PER_M);

	// copy R matrix
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			mech->ori.R[i][j] = temp[i][j];
}











































float fk_tip[HMATRIX_SIZE][HMATRIX_SIZE];
/**
*   fwdKinMech() -
*       Do the forward kinematics for a mechanism.
*       Function is taken from code developed at UCSC
*         Precondition:  current joint position is set in mech->joint[].jpos
*         Postcondition: End effector position is set in mech->pos
*
*/
void fwdMechKin_old(struct mechanism *mech)
{
	float s[7];
	float c[7];
	const float a5 = _A5;

	int armsign = (mech->type == GOLD_ARM_SERIAL) ? 1 : -1;

	const float _sin_A12 = sin(A12);
	const float _cos_A12 = cos(A12);
	const float _sin_A23 = sin(A23);
	const float _cos_A23 = cos(A23);


	s[1] = sin(mech->joint[SHOULDER].jpos);
	c[1] = cos(mech->joint[SHOULDER].jpos);

	s[2] = sin(mech->joint[ELBOW].jpos);
	c[2] = cos(mech->joint[ELBOW].jpos);

	s[3] = sin(-1 * mech->joint[TOOL_ROT].jpos);
	c[3] = cos(-1 * mech->joint[TOOL_ROT].jpos);

	float d_4 = mech->joint[Z_INS].jpos;

	s[5] = sin(armsign * mech->joint[WRIST].jpos);
	c[5] = cos(armsign * mech->joint[WRIST].jpos);

    // TODO: replace GRASP1 with avg(GRASP1,GRASP2)
	s[6] = sin(mech->joint[GRASP1].jpos);
	c[6] = cos(mech->joint[GRASP1].jpos);


	if (mech->type == GOLD_ARM) // was robot 0/2
	{
		// From Maple solution ravenforward_3revolute_rotated.mw
		// a := vector([0, 0, 0, 0, a5, 0])
		// d := vector([0, 0, 0, d_4, 0, 0])
		// alpha := vector([Pi-a1, -b1, 0, -1/2*Pi, 1/2*Pi, -1/2*Pi])
		// theta := vector([t[1], -t[2], 1/2*Pi-t[3], 0, 1/2*Pi-t[5], 1/2*Pi+t[6]])
		fk_tip[0][0] = -(((c[1]*c[2]-s[1]*_cos_A12*s[2])*s[3]+(c[1]*s[2]*_cos_A23+s[1]*_cos_A12*c[2]*_cos_A23-s[1]*_sin_A12*_sin_A23)*c[3])*s[5]+(-c[1]*s[2]*_sin_A23-s[1]*_cos_A12*c[2]*_sin_A23-s[1]*_sin_A12*_cos_A23)*c[5])*s[6]+(-(c[1]*c[2]-s[1]*_cos_A12*s[2])*c[3]+(c[1]*s[2]*_cos_A23+s[1]*_cos_A12*c[2]*_cos_A23-s[1]*_sin_A12*_sin_A23)*s[3])*c[6];
		fk_tip[0][1] = -((c[1]*c[2]-s[1]*_cos_A12*s[2])*s[3]+(c[1]*s[2]*_cos_A23+s[1]*_cos_A12*c[2]*_cos_A23-s[1]*_sin_A12*_sin_A23)*c[3])*c[5]+(-c[1]*s[2]*_sin_A23-s[1]*_cos_A12*c[2]*_sin_A23-s[1]*_sin_A12*_cos_A23)*s[5];
		fk_tip[0][2] = -(((c[1]*c[2]-s[1]*_cos_A12*s[2])*s[3]+(c[1]*s[2]*_cos_A23+s[1]*_cos_A12*c[2]*_cos_A23-s[1]*_sin_A12*_sin_A23)*c[3])*s[5]+(-c[1]*s[2]*_sin_A23-s[1]*_cos_A12*c[2]*_sin_A23-s[1]*_sin_A12*_cos_A23)*c[5])*c[6]-(-(c[1]*c[2]-s[1]*_cos_A12*s[2])*c[3]+(c[1]*s[2]*_cos_A23+s[1]*_cos_A12*c[2]*_cos_A23-s[1]*_sin_A12*_sin_A23)*s[3])*s[6];
		fk_tip[0][3] = ((c[1]*c[2]-s[1]*_cos_A12*s[2])*s[3]+(c[1]*s[2]*_cos_A23+s[1]*_cos_A12*c[2]*_cos_A23-s[1]*_sin_A12*_sin_A23)*c[3])*a5*s[5]+(-c[1]*s[2]*_sin_A23-s[1]*_cos_A12*c[2]*_sin_A23-s[1]*_sin_A12*_cos_A23)*a5*c[5]+(c[1]*s[2]*_sin_A23+s[1]*_cos_A12*c[2]*_sin_A23+s[1]*_sin_A12*_cos_A23)*d_4;

		fk_tip[1][0] = 	-(((s[1]*c[2]+c[1]*_cos_A12*s[2])*s[3]+(s[1]*s[2]*_cos_A23-c[1]*_cos_A12*c[2]*_cos_A23+c[1]*_sin_A12*_sin_A23)*c[3])*s[5]+(-s[1]*s[2]*_sin_A23+c[1]*_cos_A12*c[2]*_sin_A23+c[1]*_sin_A12*_cos_A23)*c[5])*s[6]+(-(s[1]*c[2]+c[1]*_cos_A12*s[2])*c[3]+(s[1]*s[2]*_cos_A23-c[1]*_cos_A12*c[2]*_cos_A23+c[1]*_sin_A12*_sin_A23)*s[3])*c[6];
		fk_tip[1][1] = 	-((s[1]*c[2]+c[1]*_cos_A12*s[2])*s[3]+(s[1]*s[2]*_cos_A23-c[1]*_cos_A12*c[2]*_cos_A23+c[1]*_sin_A12*_sin_A23)*c[3])*c[5]+(-s[1]*s[2]*_sin_A23+c[1]*_cos_A12*c[2]*_sin_A23+c[1]*_sin_A12*_cos_A23)*s[5];
		fk_tip[1][2] =  -(((s[1]*c[2]+c[1]*_cos_A12*s[2])*s[3]+(s[1]*s[2]*_cos_A23-c[1]*_cos_A12*c[2]*_cos_A23+c[1]*_sin_A12*_sin_A23)*c[3])*s[5]+(-s[1]*s[2]*_sin_A23+c[1]*_cos_A12*c[2]*_sin_A23+c[1]*_sin_A12*_cos_A23)*c[5])*c[6]-(-(s[1]*c[2]+c[1]*_cos_A12*s[2])*c[3]+(s[1]*s[2]*_cos_A23-c[1]*_cos_A12*c[2]*_cos_A23+c[1]*_sin_A12*_sin_A23)*s[3])*s[6];
		fk_tip[1][3] =  ((s[1]*c[2]+c[1]*_cos_A12*s[2])*s[3]+(s[1]*s[2]*_cos_A23-c[1]*_cos_A12*c[2]*_cos_A23+c[1]*_sin_A12*_sin_A23)*c[3])*a5*s[5]+(-s[1]*s[2]*_sin_A23+c[1]*_cos_A12*c[2]*_sin_A23+c[1]*_sin_A12*_cos_A23)*a5*c[5]+(s[1]*s[2]*_sin_A23-c[1]*_cos_A12*c[2]*_sin_A23-c[1]*_sin_A12*_cos_A23)*d_4;

		fk_tip[2][0] = 	-((-_sin_A12*s[2]*s[3]+(_sin_A12*c[2]*_cos_A23+_cos_A12*_sin_A23)*c[3])*s[5]+(-_sin_A12*c[2]*_sin_A23+_cos_A12*_cos_A23)*c[5])*s[6]+(_sin_A12*s[2]*c[3]+(_sin_A12*c[2]*_cos_A23+_cos_A12*_sin_A23)*s[3])*c[6];
		fk_tip[2][1] =  -(-_sin_A12*s[2]*s[3]+(_sin_A12*c[2]*_cos_A23+_cos_A12*_sin_A23)*c[3])*c[5]+(-_sin_A12*c[2]*_sin_A23+_cos_A12*_cos_A23)*s[5];
		fk_tip[2][2] =  -((-_sin_A12*s[2]*s[3]+(_sin_A12*c[2]*_cos_A23+_cos_A12*_sin_A23)*c[3])*s[5]+(-_sin_A12*c[2]*_sin_A23+_cos_A12*_cos_A23)*c[5])*c[6]-(_sin_A12*s[2]*c[3]+(_sin_A12*c[2]*_cos_A23+_cos_A12*_sin_A23)*s[3])*s[6];
		fk_tip[2][3] =  (-_sin_A12*s[2]*s[3]+(_sin_A12*c[2]*_cos_A23+_cos_A12*_sin_A23)*c[3])*a5*s[5]+(-_sin_A12*c[2]*_sin_A23+_cos_A12*_cos_A23)*a5*c[5]+(_sin_A12*c[2]*_sin_A23-_cos_A12*_cos_A23)*d_4;

		//fk_tip[3][0] = 0.0;		fk_tip[3][1] = 0.0;		fk_tip[3][2] = 0.0;		fk_tip[3][3] = 1.0;
	}
	else if (mech->type == GREEN_ARM) // was robot 1/3
	{
		// From Maple solution ravenforward_3revolute_rotated_mir.mw
		// alpha := vector([Pi-a1, -b1, 0, -1/2*Pi, -1/2*Pi, -1/2*Pi])
		// theta := vector([Pi-t[1], t[2], 3/2*Pi+t[3], 0, 1/2*Pi+t[5], 1/2*Pi-t[6]])
		fk_tip[0][0] = (-((-c[1]*c[2]+s[1]*_cos_A12*s[2])*s[3]-(c[1]*s[2]*_cos_A23+s[1]*_cos_A12*c[2]*_cos_A23-s[1]*_sin_A12*_sin_A23)*c[3])*s[5]+(-c[1]*s[2]*_sin_A23-s[1]*_cos_A12*c[2]*_sin_A23-s[1]*_sin_A12*_cos_A23)*c[5])*s[6]+(-(-c[1]*c[2]+s[1]*_cos_A12*s[2])*c[3]-(c[1]*s[2]*_cos_A23+s[1]*_cos_A12*c[2]*_cos_A23-s[1]*_sin_A12*_sin_A23)*s[3])*c[6];
		fk_tip[0][1] = ((-c[1]*c[2]+s[1]*_cos_A12*s[2])*s[3]-(c[1]*s[2]*_cos_A23+s[1]*_cos_A12*c[2]*_cos_A23-s[1]*_sin_A12*_sin_A23)*c[3])*c[5]+(-c[1]*s[2]*_sin_A23-s[1]*_cos_A12*c[2]*_sin_A23-s[1]*_sin_A12*_cos_A23)*s[5];
		fk_tip[0][2] = -(-((-c[1]*c[2]+s[1]*_cos_A12*s[2])*s[3]-(c[1]*s[2]*_cos_A23+s[1]*_cos_A12*c[2]*_cos_A23-s[1]*_sin_A12*_sin_A23)*c[3])*s[5]+(-c[1]*s[2]*_sin_A23-s[1]*_cos_A12*c[2]*_sin_A23-s[1]*_sin_A12*_cos_A23)*c[5])*c[6]+(-(-c[1]*c[2]+s[1]*_cos_A12*s[2])*c[3]-(c[1]*s[2]*_cos_A23+s[1]*_cos_A12*c[2]*_cos_A23-s[1]*_sin_A12*_sin_A23)*s[3])*s[6];
		fk_tip[0][3] = -((-c[1]*c[2]+s[1]*_cos_A12*s[2])*s[3]-(c[1]*s[2]*_cos_A23+s[1]*_cos_A12*c[2]*_cos_A23-s[1]*_sin_A12*_sin_A23)*c[3])*a5*s[5]+(-c[1]*s[2]*_sin_A23-s[1]*_cos_A12*c[2]*_sin_A23-s[1]*_sin_A12*_cos_A23)*a5*c[5]+(c[1]*s[2]*_sin_A23+s[1]*_cos_A12*c[2]*_sin_A23+s[1]*_sin_A12*_cos_A23)*d_4;

		fk_tip[1][0] = (-((s[1]*c[2]+c[1]*_cos_A12*s[2])*s[3]-(-s[1]*s[2]*_cos_A23+c[1]*_cos_A12*c[2]*_cos_A23-c[1]*_sin_A12*_sin_A23)*c[3])*s[5]+(s[1]*s[2]*_sin_A23-c[1]*_cos_A12*c[2]*_sin_A23-c[1]*_sin_A12*_cos_A23)*c[5])*s[6]+(-(s[1]*c[2]+c[1]*_cos_A12*s[2])*c[3]-(-s[1]*s[2]*_cos_A23+c[1]*_cos_A12*c[2]*_cos_A23-c[1]*_sin_A12*_sin_A23)*s[3])*c[6];
		fk_tip[1][1] = ((s[1]*c[2]+c[1]*_cos_A12*s[2])*s[3]-(-s[1]*s[2]*_cos_A23+c[1]*_cos_A12*c[2]*_cos_A23-c[1]*_sin_A12*_sin_A23)*c[3])*c[5]+(s[1]*s[2]*_sin_A23-c[1]*_cos_A12*c[2]*_sin_A23-c[1]*_sin_A12*_cos_A23)*s[5];
		fk_tip[1][2] = -(-((s[1]*c[2]+c[1]*_cos_A12*s[2])*s[3]-(-s[1]*s[2]*_cos_A23+c[1]*_cos_A12*c[2]*_cos_A23-c[1]*_sin_A12*_sin_A23)*c[3])*s[5]+(s[1]*s[2]*_sin_A23-c[1]*_cos_A12*c[2]*_sin_A23-c[1]*_sin_A12*_cos_A23)*c[5])*c[6]+(-(s[1]*c[2]+c[1]*_cos_A12*s[2])*c[3]-(-s[1]*s[2]*_cos_A23+c[1]*_cos_A12*c[2]*_cos_A23-c[1]*_sin_A12*_sin_A23)*s[3])*s[6];
		fk_tip[1][3] = -((s[1]*c[2]+c[1]*_cos_A12*s[2])*s[3]-(-s[1]*s[2]*_cos_A23+c[1]*_cos_A12*c[2]*_cos_A23-c[1]*_sin_A12*_sin_A23)*c[3])*a5*s[5]+(s[1]*s[2]*_sin_A23-c[1]*_cos_A12*c[2]*_sin_A23-c[1]*_sin_A12*_cos_A23)*a5*c[5]+(-s[1]*s[2]*_sin_A23+c[1]*_cos_A12*c[2]*_sin_A23+c[1]*_sin_A12*_cos_A23)*d_4;

		fk_tip[2][0] = (-(_sin_A12*s[2]*s[3]-(_sin_A12*c[2]*_cos_A23+_cos_A12*_sin_A23)*c[3])*s[5]+(-_sin_A12*c[2]*_sin_A23+_cos_A12*_cos_A23)*c[5])*s[6]+(-_sin_A12*s[2]*c[3]-(_sin_A12*c[2]*_cos_A23+_cos_A12*_sin_A23)*s[3])*c[6];
		fk_tip[2][1] = (_sin_A12*s[2]*s[3]-(_sin_A12*c[2]*_cos_A23+_cos_A12*_sin_A23)*c[3])*c[5]+(-_sin_A12*c[2]*_sin_A23+_cos_A12*_cos_A23)*s[5];
		fk_tip[2][2] = -(-(_sin_A12*s[2]*s[3]-(_sin_A12*c[2]*_cos_A23+_cos_A12*_sin_A23)*c[3])*s[5]+(-_sin_A12*c[2]*_sin_A23+_cos_A12*_cos_A23)*c[5])*c[6]+(-_sin_A12*s[2]*c[3]-(_sin_A12*c[2]*_cos_A23+_cos_A12*_sin_A23)*s[3])*s[6];
		fk_tip[2][3] = -(_sin_A12*s[2]*s[3]-(_sin_A12*c[2]*_cos_A23+_cos_A12*_sin_A23)*c[3])*a5*s[5]+(-_sin_A12*c[2]*_sin_A23+_cos_A12*_cos_A23)*a5*c[5]+(_sin_A12*c[2]*_sin_A23-_cos_A12*_cos_A23)*d_4;
	}
	else
	{
		log_msg("ERROR: fkin not implemented for mech type: %d", mech->type);
	}


	mech->pos.x = (int)(fk_tip[0][3] * MICRON_PER_M);
	mech->pos.y = (int)(fk_tip[1][3] * MICRON_PER_M);
	mech->pos.z = (int)(fk_tip[2][3] * MICRON_PER_M);

	// copy R matrix
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			mech->ori.R[i][j] = fk_tip[i][j];

	// set orientation
	orientation_from_hmatrix(fk_tip, &(mech->ori));

}
