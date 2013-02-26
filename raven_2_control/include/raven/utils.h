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
#include <LinearMath/btMatrix3x3.h>

#include "DS0.h"


#ifndef NULL
#define NULL 0
#endif

//Short maximum and minimum
#define SHORT_MAX 32767
#define SHORT_MIN -32768

//Return values
#define SHORT_OVERFLOW    1
#define SHORT_UNDERFLOW  -1

inline btMatrix3x3 toBt(float R[3][3]) {
	btMatrix3x3 btR;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			btR[i][j] = R[i][j];
		}
	}
	return btR;
}

template<typename OtherType>
btMatrix3x3 toBt_from(OtherType R) {
	btMatrix3x3 btR;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			btR[i][j] = R[i][j];
		}
	}
	return btR;
}

inline btMatrix3x3 toBt(orientation o) {
	return toBt(o.R);
}

inline btVector3 toBt(position p) {
	return btVector3(p.x,p.y,p.z);
}

inline btTransform toBt(position p,orientation o) {
	return btTransform(toBt(o),toBt(p));
}

inline btTransform toBt(position p,float R[3][3]) {
	return btTransform(toBt(R),toBt(p));
}

template<typename OtherType>
inline btTransform toBt(position p,OtherType R) {
	return btTransform(toBt(R),toBt(p));
}

//btTransform Z(float theta,float d);
//btTransform X(float alpha,float a);

template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

float fix_angle(float angle,float center = 0);
float saturatef(float value,float min,float max);
int saturate(int value,int min,int max);

int loop_over_mechs(struct robot_device* device0, struct mechanism*& _mech, int& mechnum);
int loop_over_joints(struct robot_device*, struct mechanism*&, struct DOF*&, int&, int&);
int loop_over_joints(struct mechanism* _mech, struct DOF*& _joint, int& jnum);

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

#ifndef TB_ANGLES_CLOSE_ENOUGH
#define TB_ANGLES_CLOSE_ENOUGH 0.0001
#endif

class tb_angles {
public:
	float yaw;
	float yaw_deg;
	float pitch;
	float pitch_deg;
	float roll;
	float roll_deg;

	tb_angles(float yaw, float pitch, float roll,bool deg=true) {
		if (deg) {
			this->yaw = yaw * M_PI / 180.;
			this->yaw_deg = yaw;
			this->pitch = pitch * M_PI / 180.;
			this->pitch_deg = pitch;
			this->roll = roll * M_PI / 180.;
			this->roll_deg = roll;
		} else {
			this->yaw = yaw;
			this->yaw_deg = yaw * 180. / M_PI;
			this->pitch = pitch;
			this->pitch_deg = pitch * 180. / M_PI;
			this->roll = roll;
			this->roll_deg = roll * 180. / M_PI;
		}
	}
	tb_angles(const btQuaternion& q) { init(btMatrix3x3(q)); }
	tb_angles(const btTransform& T) { init(T.getBasis()); }
	tb_angles(const btMatrix3x3& R) { init(R); }

	template<typename OtherType>
	tb_angles(OtherType R) {
		btMatrix3x3 btR;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				btR[i][j] = R[i][j];
			}
		}
		init(btR);
	}

	btQuaternion toQuaternion() const { btQuaternion q; toMatrix().getRotation(q); return q; }
	btTransform toTransform() const { btTransform T; T.setBasis(toMatrix()); return T; }
	btMatrix3x3 toMatrix() const {
		btMatrix3x3 Ryaw(
				cos(yaw), -sin(yaw), 0,
				sin(yaw),  cos(yaw), 0,
				0,         0,        1);
		btMatrix3x3 Rpitch(
				 cos(pitch), 0, sin(pitch),
				 0,          1, 0,
				-sin(pitch), 0, cos(pitch));
		btMatrix3x3 Rroll(
				1,  0,          0,
				0,  cos(roll), -sin(roll),
				0,  sin(roll),  cos(roll));
		return Ryaw * Rpitch * Rroll;
	}

private:
	void init(const btMatrix3x3& R) {
		yaw = 0;
		pitch = 0;
		roll = 0;

		bool skip = false;
		if (fabs(R[0][1]-R[1][0]) < TB_ANGLES_CLOSE_ENOUGH && fabs(R[0][2]-R[2][0]) < TB_ANGLES_CLOSE_ENOUGH && fabs(R[1][2]-R[2][1]) < TB_ANGLES_CLOSE_ENOUGH) {
			//matrix is symmetric
			if (fabs(R[0][1]+R[1][0]) < TB_ANGLES_CLOSE_ENOUGH && fabs(R[0][2]+R[2][0]) < TB_ANGLES_CLOSE_ENOUGH && fabs(R[1][2]+R[2][1]) < TB_ANGLES_CLOSE_ENOUGH) {
				//diagonal
				if (R[0][0] > 0) {
					if (R[1][1] > 0) {
						skip = true;
					} else {
						roll = M_PI;
					}
				} else if (R[1][1] > 0) {
					yaw = M_PI;
					pitch = M_PI;
				} else {
					yaw = M_PI;
				}
				skip = true;
			}
		}

		if (!skip) {
			btVector3 vx = R * btVector3(1,0,0);
			btVector3 vy = R * btVector3(0,1,0);

			yaw = atan2(vx.y(),vx.x());
			pitch = atan2(-vx.z(), sqrt(vx.x()*vx.x() + vx.y()*vx.y()));

			btMatrix3x3 Ryaw(
						 cos(yaw), -sin(yaw), 0,
						 sin(yaw),  cos(yaw), 0,
						 0,         0,        1);
			btMatrix3x3 Rpitch(
					 cos(pitch), 0, sin(pitch),
					 0,          1, 0,
					-sin(pitch), 0, cos(pitch));
			btVector3 vyp = Ryaw * Rpitch * btVector3(0,1,0);
			btVector3 vzp = Ryaw * Rpitch * btVector3(0,0,1);

			float coeff = vzp.dot(vy) >= 0 ? 1 : -1;

			roll = coeff * acos(vyp.dot(vy));
		}

		yaw_deg = yaw * 180. / M_PI;
		pitch_deg = pitch * 180. / M_PI;
		roll_deg = roll * 180. / M_PI;
	}
};

/*
template<typename OtherType>
tb_angles2 tb_angles_from(OtherType R) {
	btMatrix3x3 btR;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			btR[i][j] = R[i][j];
		}
	}
	return tb_angles(btR);
}
*/

#endif
