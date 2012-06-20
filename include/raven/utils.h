/*
 * utils.h
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

#ifndef __UTILS_H__
#define __UTILS_H__

#include <time.h>
#include <tf/transform_datatypes.h>


#ifndef NULL
#define NULL 0
#endif

//Short maximum and minimum
#define SHORT_MAX 32767
#define SHORT_MIN -32768

//Return values
#define SHORT_OVERFLOW    1
#define SHORT_UNDERFLOW  -1

struct tb_angles {
	float yaw;
	float yaw_deg;
	float pitch;
	float pitch_deg;
	float roll;
	float roll_deg;
};

btMatrix3x3 tb_to_mat(float yaw, float pitch, float roll);
inline btQuaternion tb_to_quat(float yaw, float pitch, float roll) { btQuaternion q; tb_to_mat(yaw,pitch,roll).getRotation(q); return q; }

tb_angles get_tb_angles(btMatrix3x3 R);

inline tb_angles get_tb_angles(float rot[3][3]) {
	btMatrix3x3 R;
	for (int i=0;i<3;i++) {
		for (int j=0;j<3;j++) {
			R[i][j] = rot[i][j];
		}
	}
	return get_tb_angles(R);
}

inline tb_angles get_tb_angles(btTransform T) { return get_tb_angles(T.getBasis()); }
inline tb_angles get_tb_angles(btQuaternion q) { return get_tb_angles(btMatrix3x3(q)); }

inline btMatrix3x3 toBt(float R[3][3]) {
	btMatrix3x3 btR;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			btR[i][j] = R[i][j];
		}
	}
	return btR;
}

btTransform Z(float theta,float d);
btTransform X(float alpha,float a);

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

float fix_angle(float angle,float center = 0);

int loop_over_joints(struct robot_device*, struct mechanism*&, struct DOF*&, int&, int&);
int loop_over_joints(struct mechanism* _mech, struct DOF*& _joint, int& jnum);

int toShort(int value, short int *target);
void strtoken(char *str, char *result, char delim);
void strcopy(const char *src, char *dest);

int is_toolDOF(struct DOF*);
int is_toolDOF(int);
int tools_ready(struct mechanism *mech);
int robot_ready(struct robot_device* device0);

const int _Qx=0, _Qy=1, _Qz=2, _Qw=3;
void getQuaternion(float* Q, float mat[3][3]);

/**
 * the struct timespec consists of nanoseconds
 * and seconds. This rolls over the ns to seconds.
 */
#define NSEC_PER_SEC    1000000000          // nanoseconds per sec
static inline void tsnorm(struct timespec *ts)
{
    while (ts->tv_nsec >= NSEC_PER_SEC)
    {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}

struct  timespec  tsSubtract ( struct  timespec  time1,
                                           struct  timespec  time2);
// Reset posd so that it is coincident with pos.
void set_posd_to_pos(struct robot_device* device0);

#endif
