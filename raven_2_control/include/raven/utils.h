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

#include <cstdio>


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
	/**
	 * The yaw angle in degrees. If modified, call updateFromDegrees()
	 */
	float yaw_deg;
	/**
	 * The yaw angle in radians. If modified, call updateFromRadians()
	 */
	float yaw_rad;
	/**
	 * The pitch angle in degrees. If modified, call updateFromDegrees()
	 */
	float pitch_deg;
	/**
	 * The pitch angle in radians. If modified, call updateFromRadians()
	 */
	float pitch_rad;
	/**
	 * The roll angle in degrees. If modified, call updateFromDegrees()
	 */
	float roll_deg;
	/**
	 * The roll angle in radians. If modified, call updateFromRadians()
	 */
	float roll_rad;

	enum angle_type { DEG = 1, RAD = 2 };

	/**
	 * Create tb_angles using a yaw, pitch and roll angles, in degrees by
	 * default. Set angle_type to tb_angles::RAD if giving angles in radians.
	 */
	tb_angles(float yaw, float pitch, float roll,int angle_type=DEG) {
		if (angle_type & RAD) {
			this->yaw_rad = yaw;
			this->yaw_deg = yaw * 180. / M_PI;
			this->pitch_rad = pitch;
			this->pitch_deg = pitch * 180. / M_PI;
			this->roll_rad = roll;
			this->roll_deg = roll * 180. / M_PI;
		} else {
			this->yaw_rad = yaw * M_PI / 180.;
			this->yaw_deg = yaw;
			this->pitch_rad = pitch * M_PI / 180.;
			this->pitch_deg = pitch;
			this->roll_rad = roll * M_PI / 180.;
			this->roll_deg = roll;
		}
	}
	/**
	 * Create tb_angles from a btQuaternion.
	 */
	tb_angles(const btQuaternion& q) { init(btMatrix3x3(q)); }
	/**
	 * Create tb_angles from a btTransform.
	 */
	tb_angles(const btTransform& T) { init(T.getBasis()); }
	/**
	 * Create tb_angles from a btMatrix3x3.
	 */
	tb_angles(const btMatrix3x3& R) { init(R); }

	/**
	 * Create tb_angles from a matrix type. The matrix type must have its
	 * elements accessible as M[i][j]. This includes multidimensional C arrays.
	 */
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

	/**
	 * Create tb_angles from a ROS geometry_msgs/Quaternion.
	 */
	tb_angles(const geometry_msgs::Quaternion& q) {
		btQuaternion btq(q.x,q.y,q.z,q.w);
		init(btMatrix3x3(btq));
	}

	/**
	 * Convert to btQuaternion.
	 */
	btQuaternion toQuaternion() const { btQuaternion q; toMatrix().getRotation(q); return q; }
	/**
	 * Convert to btTransform
	 */
	btTransform toTransform() const { btTransform T; T.setBasis(toMatrix()); return T; }
	/**
	 * Convert to btMatrix
	 */
	btMatrix3x3 toMatrix() const {
		btMatrix3x3 Ryaw(
				cos(yaw_rad), -sin(yaw_rad), 0,
				sin(yaw_rad),  cos(yaw_rad), 0,
				0,         0,        1);
		btMatrix3x3 Rpitch(
				cos(pitch_rad), 0, sin(pitch_rad),
				0,          1, 0,
				-sin(pitch_rad), 0, cos(pitch_rad));
		btMatrix3x3 Rroll(
				1,  0,          0,
				0,  cos(roll_rad), -sin(roll_rad),
				0,  sin(roll_rad),  cos(roll_rad));
		return Ryaw * Rpitch * Rroll;
	}
	/**
	 * Convert to ROS geometry_msgs/Quaternion
	 */
	geometry_msgs::Quaternion toMsg() const {
		btQuaternion btq = toQuaternion();
		geometry_msgs::Quaternion q;
		q.x = btq[0];
		q.y = btq[1];
		q.z = btq[2];
		q.w = btq[3];
		return q;
	}

	enum string_options { FIXED_WIDTH = 4, SHORT_NAMES = 8 };

	/**
	 * Get the string representation of this rotation.
	 * By default, the values are shown in degrees, without units.
	 * The options tb_angles::DEG and tb_angles::RAD can be used to display the
	 * values in degrees and radians, with units.
	 * tb.tostring():              values in degrees without units
	 * tb.tostring(tb_angles::DEG) values in degrees    with units
	 * tb.tostring(tb_angles::RAD) values in radians    with units
	 * tb.tostring(tb_angles::DEG & tb_angles::RAD) will print values in both degrees
	 *     and radians
	 *
	 * The option tb_angles::FIXED_WIDTH will cause the output to use fixed-width
	 * fields for the values, which can be useful if many values are being
	 * printed in succession.
	 *
	 * The option tb_angles::SHORT_NAMES causes the field names to be abbreviated
	 * to one letter, e.g., 'y' instead of 'yaw'
	 */
	std::string toString(int options = 0) const {
		char buf[120];
		const char* deg_fmt_fixed = "% 6.1f";
		const char* deg_fmt_var = "%.1f";
		const char* rad_fmt_fixed = "% 6.3f";
		const char* rad_fmt_var = "%.3f";

		bool deg = (options & DEG) || !(options & RAD);
		bool rad = options & RAD;

		std::string deg_fmt_str;
		std::string rad_fmt_str;
		if (options & FIXED_WIDTH) {
			deg_fmt_str = deg_fmt_fixed;
			rad_fmt_str = rad_fmt_fixed;
		} else {
			deg_fmt_str = deg_fmt_var;
			rad_fmt_str = rad_fmt_var;
		}

		std::string deg_str;
		std::string rad_str;
		if (deg) {
			if (! (options & DEG)) {
				deg_str = deg_fmt_str;
			} else {
				deg_str = deg_fmt_str + " deg";
			}
		} else {
			deg_str = "%.s";
		}
		if (rad) {
			rad_str = rad_fmt_str + " rad";
			if (deg) {
				rad_str = " (" + rad_str + ")";
			}
		} else {
			rad_str = "%.s";
		}

		char yaw_deg_str[20];   sprintf(yaw_deg_str,deg_str.c_str(),yaw_deg);
		char pitch_deg_str[20]; sprintf(pitch_deg_str,deg_str.c_str(),pitch_deg);
		char roll_deg_str[20];  sprintf(roll_deg_str,deg_str.c_str(),roll_deg);

		char yaw_rad_str[20];   sprintf(yaw_rad_str,rad_str.c_str(),yaw_rad);
		char pitch_rad_str[20]; sprintf(pitch_rad_str,rad_str.c_str(),pitch_rad);
		char roll_rad_str[20];  sprintf(roll_rad_str,rad_str.c_str(),roll_rad);

		char yaw_str[35];
		char pitch_str[35];
		char roll_str[35];

		sprintf(yaw_str,"%s%s",yaw_deg_str,yaw_rad_str);
		sprintf(pitch_str,"%s%s",pitch_deg_str,pitch_rad_str);
		sprintf(roll_str,"%s%s",roll_deg_str,roll_rad_str);

		std::string fmt_str1;
		if (options & SHORT_NAMES) {
			fmt_str1 = "[y:%s, p:%s, r:%s]";
		} else {
			fmt_str1 = "[yaw:%s, pitch:%s, roll:%s]";
		}

		sprintf(buf,fmt_str1.c_str(),yaw_str,pitch_str,roll_str);
		return std::string(buf);
	}

	/**
	 * Update the radian fields from the degree fields (e.g., yaw_rad from
	 * yaw_deg). Call this method after modifying one of the degree fields.
	 */
	void updateFromDegrees() {
		yaw_rad = yaw_deg * M_PI / 180.;
		pitch_rad = pitch_deg * M_PI / 180.;
		roll_rad = roll_deg * M_PI / 180.;
	}

	/**
	 * Update the degree fields from the radian fields (e.g., yaw_deg from
	 * yaw_rad). Call this method after modifying one of the radian fields.
	 */
	void updateFromRadians() {
		yaw_deg = yaw_deg * 180. / M_PI;
		pitch_deg = pitch_deg * 180. / M_PI;
		roll_deg = roll_deg * 180. / M_PI;
	}

private:
	void init(const btMatrix3x3& R) {
		yaw_rad = 0;
		pitch_rad = 0;
		roll_rad = 0;

		bool skip = false;
		if (fabs(R[0][1]-R[1][0]) < TB_ANGLES_CLOSE_ENOUGH && fabs(R[0][2]-R[2][0]) < TB_ANGLES_CLOSE_ENOUGH && fabs(R[1][2]-R[2][1]) < TB_ANGLES_CLOSE_ENOUGH) {
			//matrix is symmetric
			if (fabs(R[0][1]+R[1][0]) < TB_ANGLES_CLOSE_ENOUGH && fabs(R[0][2]+R[2][0]) < TB_ANGLES_CLOSE_ENOUGH && fabs(R[1][2]+R[2][1]) < TB_ANGLES_CLOSE_ENOUGH) {
				//diagonal
				if (R[0][0] > 0) {
					if (R[1][1] > 0) {
						skip = true;
					} else {
						roll_rad = M_PI;
					}
				} else if (R[1][1] > 0) {
					yaw_rad = M_PI;
					pitch_rad = M_PI;
				} else {
					yaw_rad = M_PI;
				}
				skip = true;
			}
		}

		if (!skip) {
			btVector3 vx = R * btVector3(1,0,0);
			btVector3 vy = R * btVector3(0,1,0);

			yaw_rad = atan2(vx.y(),vx.x());
			pitch_rad = atan2(-vx.z(), sqrt(vx.x()*vx.x() + vx.y()*vx.y()));

			btMatrix3x3 Ryaw(
						 cos(yaw_rad), -sin(yaw_rad), 0,
						 sin(yaw_rad),  cos(yaw_rad), 0,
						 0,         0,        1);
			btMatrix3x3 Rpitch(
					 cos(pitch_rad), 0, sin(pitch_rad),
					 0,          1, 0,
					-sin(pitch_rad), 0, cos(pitch_rad));
			btVector3 vyp = Ryaw * Rpitch * btVector3(0,1,0);
			btVector3 vzp = Ryaw * Rpitch * btVector3(0,0,1);

			float coeff = vzp.dot(vy) >= 0 ? 1 : -1;

			roll_rad = coeff * acos(vyp.dot(vy));
		}

		yaw_deg = yaw_rad * 180. / M_PI;
		pitch_deg = pitch_rad * 180. / M_PI;
		roll_deg = roll_rad * 180. / M_PI;
	}
};

inline std::ostream& operator<<(std::ostream& o, const tb_angles& tb) {
	o << tb.toString();
	return o;
}

#endif
