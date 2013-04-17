/*
 * defines.h
 *
 * Global defines
 *
 */

#ifndef __DEFINE_H__
#define __DEFINE_H__

#include <math.h>
#include <string>
#include <sstream>
#include <stdio.h>

#define RAVEN_MODULE_VERSION RAVEN_II_RELEASE_02
#define device robot_device

#define SURGICAL_ROBOT  1
#define RAVEN_II        1

// Two arm identification
// Change this to match device ID in /dev/brl_usb_XX
#define GREEN_ARM_SERIAL 23
#define GOLD_ARM_SERIAL  34

#define GREEN_ARM        GREEN_ARM_SERIAL
#define GOLD_ARM         GOLD_ARM_SERIAL

#define GOLD_ARM_ID 0
#define GREEN_ARM_ID 1

#define GREEN_ARM_BASE_POSE btTransform(btQuaternion(0,0,M_PI_2),btVector3(-0.14858,0.002,0))
//#define GREEN_ARM_BASE_POSE btTransform(btVector3(-0.14858,0.002,0))

inline int armIdFromSerial(int arm_serial) {
	if (arm_serial == GOLD_ARM_SERIAL) { return GOLD_ARM_ID; }
	if (arm_serial == GREEN_ARM_SERIAL) { return GREEN_ARM_ID; }
	return -1;
}

inline int armSerialFromID(int arm_id) {
	if (arm_id == GOLD_ARM_ID) { return GOLD_ARM_SERIAL; }
	if (arm_id == GREEN_ARM_ID) { return GREEN_ARM_SERIAL; }
	return -1;
}

inline int armIdFromMechType(int mech_type) {
	if (mech_type == GOLD_ARM) { return GOLD_ARM_ID; }
	if (mech_type == GREEN_ARM) { return GREEN_ARM_ID; }
	return -1;
}

inline std::string armNameFromId(int arm_id) {
	if (arm_id == GOLD_ARM_ID) { return "L"; }
	if (arm_id == GREEN_ARM_ID) { return "R"; }
	std::stringstream ss; ss << "UNKNOWN:" << arm_id; return ss.str();
}

inline int armIdFromName(const std::string& name) {
	if (name == "L") { return GOLD_ARM_ID; }
	if (name == "R") { return GREEN_ARM_ID; }
	return -1;
}

inline std::string armNameFromSerial(int arm_serial) { return armNameFromId(armIdFromSerial(arm_serial)); }
inline std::string armNameFromMechType(int mech_type) { return armNameFromId(armIdFromMechType(mech_type)); }

// Event logging function
//  Log levels:
//     0: Trivial   1: Normal/informational    2: Critical/Important
//  Log Codes:
//     0-3: Entered runlevel 0-3   4: RT_Process started  5: RT_Process ended
///TODO: Redefine rt_printk to suitable debugging function
#define ROBOT_LOG(logLevel, logCode, fmt, args...) //rt_printk("ROBOT LOG[%d/%d]: " fmt, logLevel, logCode, ##args)

//Include current clipping
#define CURRENT_CLIPPING

// Degree - radian converion
#define DEG2RAD *(M_PI/180.0)
#define RAD2DEG *(180.0/M_PI)

#define ENC_CNT_PER_DEG (float)(ENC_CNTS_PER_REV / 360)
#define ENC_CNT_PER_RAD (float)(ENC_CNTS_PER_REV / (2*M_PI))

//Verbose mode
//#define MORE_MESSAGES 1

// Software can set runlevels
//#define SOFTWARE_RUNLEVELS 1

// PLC can set runlevels (PLC takes priority if SOFTWARE_RUNLEVELS is also defined.)
#define PLC_RUNLEVELS 1

//RUN LEVELS
#define RL_E_STOP    0
#define RL_INIT      1
#define RL_PEDAL_UP  2
#define RL_PEDAL_DN  3

inline std::string runLevelName(int val) {
	switch (val) {
	case RL_E_STOP: return "E STOP";
	case RL_INIT: return "INIT";
	case RL_PEDAL_UP: return "PEDAL UP";
	case RL_PEDAL_DN: return "PEDAL DOWN";
	default: std::stringstream ss; ss << "UNKNOWN:" << val; return ss.str();
	}
}

#define SL_PD_CTRL   0
#define SL_DAC_CTRL  1
#define SL_AUTO_INIT 3

inline std::string runSublevelName(int val) {
	switch (val) {
	case SL_PD_CTRL: return "PD CTRL";
	case SL_DAC_CTRL: return "DAC CTRL";
	case SL_AUTO_INIT: return "AUTO INIT";
	default: std::stringstream ss; ss << "UNKNOWN:" << val; return ss.str();
	}
}

//Joint Defines
#define SHOULDER   0
#define ELBOW      1
#define Z_INS      2
//<not connected>    3
#define TOOL_ROT   4
#define WRIST      5
#define GRASP1     6
#define GRASP2     7
#define NO_CONNECTION 3

inline std::string jointIndexName(int val) {
	switch (val) {
	case SHOULDER: return "SHOULDER";
	case ELBOW: return "ELBOW";
	case Z_INS: return "Z_INS";
	case TOOL_ROT: return "TOOL_ROT";
	case WRIST: return "WRIST";
	case GRASP1: return "GRASP1";
	case GRASP2: return "GRASP2";
	case NO_CONNECTION: return "NO_CONNECTION";
	default: std::stringstream ss; ss << "UNKNOWN:" << val; return ss.str();
	}
}

//GOLD Arm Defines
#define SHOULDER_GOLD   0
#define ELBOW_GOLD      1
#define Z_INS_GOLD      2

#define TOOL_ROT_GOLD   4
#define WRIST_GOLD      5
#define GRASP1_GOLD     6
#define GRASP2_GOLD     7
#define NO_CONNECTION_GOLD 3

//GREEN Arm Defines
#define SHOULDER_GREEN   8
#define ELBOW_GREEN      9
#define Z_INS_GREEN      10
//<not connected>    11
#define TOOL_ROT_GREEN   12
#define WRIST_GREEN      13
#define GRASP1_GREEN     14
#define GRASP2_GREEN     15
#define NO_CONNECTION_GREEN 11

inline std::string jointIndexAndArmName(int combinedType) {
	switch (combinedType) {
	case SHOULDER_GOLD: return "SHOULDER_GOLD";
	case ELBOW_GOLD: return "ELBOW_GOLD";
	case Z_INS_GOLD: return "Z_INS_GOLD";
	case TOOL_ROT_GOLD: return "TOOL_ROT_GOLD";
	case WRIST_GOLD: return "WRIST_GOLD";
	case GRASP1_GOLD: return "GRASP1_GOLD";
	case GRASP2_GOLD: return "GRASP2_GOLD";
	case NO_CONNECTION_GOLD: return "NO_CONNECTION_GOLD";
	case SHOULDER_GREEN: return "SHOULDER_GREEN";
	case ELBOW_GREEN: return "ELBOW_GREEN";
	case Z_INS_GREEN: return "Z_INS_GREEN";
	case TOOL_ROT_GREEN: return "TOOL_ROT_GREEN";
	case WRIST_GREEN: return "WRIST_GREEN";
	case GRASP1_GREEN: return "GRASP1_GREEN";
	case GRASP2_GREEN: return "GRASP2_GREEN";
	case NO_CONNECTION_GREEN: return "NO_CONNECTION_GREEN";
	default: std::stringstream ss; ss << "UNKNOWN:" << combinedType; return ss.str();
	}
}

inline int jointTypeFromCombinedType(int combinedType) {
	switch (combinedType) {
		case SHOULDER_GOLD: return SHOULDER;
		case ELBOW_GOLD: return ELBOW;
		case Z_INS_GOLD: return Z_INS;
		case TOOL_ROT_GOLD: return TOOL_ROT;
		case WRIST_GOLD: return WRIST;
		case GRASP1_GOLD: return GRASP1;
		case GRASP2_GOLD: return GRASP2;
		case NO_CONNECTION_GOLD: return NO_CONNECTION;
		case SHOULDER_GREEN: return SHOULDER;
		case ELBOW_GREEN: return ELBOW;
		case Z_INS_GREEN: return Z_INS;
		case TOOL_ROT_GREEN: return TOOL_ROT;
		case WRIST_GREEN: return WRIST;
		case GRASP1_GREEN: return GRASP1;
		case GRASP2_GREEN: return GRASP2;
		case NO_CONNECTION_GREEN: return NO_CONNECTION;
		default: printf("UNKNOWN COMBINED TYPE: %d\n",combinedType); return -1;
		}
}

inline int combinedJointIndex(int armId,int jointId) {
	if (armId == GOLD_ARM_ID) {
		switch (jointId) {
		case SHOULDER: return SHOULDER_GOLD;
		case ELBOW: return ELBOW_GOLD;
		case Z_INS: return Z_INS_GOLD;
		case TOOL_ROT: return TOOL_ROT_GOLD;
		case WRIST: return WRIST_GOLD;
		case GRASP1: return GRASP1_GOLD;
		case GRASP2: return GRASP2_GOLD;
		case NO_CONNECTION: return NO_CONNECTION_GOLD;
		default: return -1;
		}
	} else {
		switch (jointId) {
		case SHOULDER: return SHOULDER_GREEN;
		case ELBOW: return ELBOW_GREEN;
		case Z_INS: return Z_INS_GREEN;
		case TOOL_ROT: return TOOL_ROT_GREEN;
		case WRIST: return WRIST_GREEN;
		case GRASP1: return GRASP1_GREEN;
		case GRASP2: return GRASP2_GREEN;
		case NO_CONNECTION: return NO_CONNECTION_GREEN;
		default: return -1;
		}
	}
}

inline void getArmAndJointIndices(int combinedId,int& armId,int& jointId) {
	switch (combinedId) {
	case SHOULDER_GOLD: jointId = SHOULDER; armId = GOLD_ARM_ID; return;
	case ELBOW_GOLD: jointId = ELBOW; armId = GOLD_ARM_ID; return;
	case Z_INS_GOLD: jointId = Z_INS; armId = GOLD_ARM_ID; return;
	case TOOL_ROT_GOLD: jointId = TOOL_ROT; armId = GOLD_ARM_ID; return;
	case WRIST_GOLD: jointId = WRIST; armId = GOLD_ARM_ID; return;
	case GRASP1_GOLD: jointId = GRASP1; armId = GOLD_ARM_ID; return;
	case GRASP2_GOLD: jointId = GRASP2; armId = GOLD_ARM_ID; return;
	case NO_CONNECTION_GOLD: jointId = NO_CONNECTION; armId = GOLD_ARM_ID; return;
	case SHOULDER_GREEN: jointId = SHOULDER; armId = GREEN_ARM_ID; return;
	case ELBOW_GREEN: jointId = ELBOW; armId = GREEN_ARM_ID; return;
	case Z_INS_GREEN: jointId = Z_INS; armId = GREEN_ARM_ID; return;
	case TOOL_ROT_GREEN: jointId = TOOL_ROT; armId = GREEN_ARM_ID; return;
	case WRIST_GREEN: jointId = WRIST; armId = GREEN_ARM_ID; return;
	case GRASP1_GREEN: jointId = GRASP1; armId = GREEN_ARM_ID; return;
	case GRASP2_GREEN: jointId = GRASP2; armId = GREEN_ARM_ID; return;
	case NO_CONNECTION_GREEN: jointId = NO_CONNECTION; armId = GREEN_ARM_ID; return;
	default: armId = -1; jointId = -1; return;
	}
}

//Joint Scale Factors
#define WRIST_SCALE_FACTOR (float)(1.5) /*used in update_device_state.c on incoming param*/

//Amplifier constants.
#define K_DAC_PER_AMP_LOW_CURRENT   5461   // Empirically validated as the same as MFHD constant
#define K_DAC_PER_AMP_HIGH_CURRENT  2730   // High current amplifiers output double the low.

// Gear box ratios
#define GEAR_BOX_GP42_TR 12.0 // (49.0/4.0)  // 12.25
#define GEAR_BOX_GP32_TR 3.7 //(26.0/7.0)	// 3.7

#define GEAR_BOX_TR_BIG_MOTOR     GEAR_BOX_GP42_TR
#define GEAR_BOX_TR_SMALL_MOTOR   GEAR_BOX_GP32_TR

//Transmission Ratios
// See also, motor.h
#define CABLE_RADIUS_1050 (1.19/2.0)
#define CABLE_RADIUS_1024 (0.61/2.0)

#define CAPSTAN_RADIUS_GP42_1050 (5.4 + CABLE_RADIUS_1050)
#define CAPSTAN_RADIUS_GP42_1024 (5.4 + CABLE_RADIUS_1024)
#define CAPSTAN_RADIUS_GP32 (4.7  + CABLE_RADIUS_1024)

#define TRANSMISSION_PULLEY_RADIUS_SMALL (7.60 + CABLE_RADIUS_1024)
#define TRANSMISSION_PULLEY_RADIUS_LARGE (7.60 + CABLE_RADIUS_1050)

#define PARTIAL_PULLEY_LINK1_RADIUS (62.5  + CABLE_RADIUS_1050)

#define CAPSTAN_LINK2_LARGE_RADIUS (7.82  + CABLE_RADIUS_1050)
#define CAPSTAN_LINK2_SMALL_RADIUS (5.60  + CABLE_RADIUS_1050)
#define PARTIAL_PULLEY_LINK2_RADIUS (40.85 + CABLE_RADIUS_1050)

#define CAPSTAN_TOOL_RADIUS (10.6 + CABLE_RADIUS_1024)

//tool driving capston on top
#define CAPSTAN_RADIUS_ROLL  (7.2  + CABLE_RADIUS_1024)
#define CAPSTAN_RADIUS_PITCH (3.63 + CABLE_RADIUS_1024)
#define CAPSTAN_RADIUS_YAW   (3.63 + CABLE_RADIUS_1024)

#define TOOL_PULLEY_RADIUS_ROLL  (4.675 + CABLE_RADIUS_1024)
#define TOOL_PULLEY_RADIUS_PITCH (3.6   + CABLE_RADIUS_1024)
#define TOOL_PULLEY_RADIUS_YAW   (3.6   + CABLE_RADIUS_1024)




// Cable coupling constants
//  Note: The numbering is wierd b/c the UCSC software puts Z-axis after tool rotation.
//        Take care to use the #defined name (e.g. Z_INS / TOOL_ROT) when possible.
//        If not, then may god have mercy on your soul.  sSee fwd_cable_coupling.cpp for example
//
//   Note: By observation Hawkeye determined the following:
// TR1 : First joint, Shoulder
// TR2 : Second joint, Elbow
// TR3 : Fourth joint, Tool rotation
// TR4 : Third joint, Z-Insertion, prismatic
// TR5 : Fifth Joint, Wrist
// TR6 : Sixth joint, Grasper jaw 1
// TR7 : Seventh joint, Grasper jaw 2
//    This is most critical in init.cpp - initDOFparams() and
#define SHOULDER_TR_GREEN_ARM (float)( (PARTIAL_PULLEY_LINK1_RADIUS/CAPSTAN_RADIUS_GP42_1050) * GEAR_BOX_GP42_TR) // RE-40, GP-42 // UNITLESS
#define ELBOW_TR_GREEN_ARM    (float)( (PARTIAL_PULLEY_LINK2_RADIUS/CAPSTAN_LINK2_SMALL_RADIUS)  *  (CAPSTAN_LINK2_LARGE_RADIUS/CAPSTAN_RADIUS_GP42_1050) * GEAR_BOX_GP42_TR) // RE-40, GP-42 // UNITLESS
#define Z_INS_TR_GREEN_ARM    (float)( (1.0/((2*M_PI*CAPSTAN_RADIUS_GP42_1024)/1000.0)) * GEAR_BOX_GP42_TR *2*M_PI) // UNITS: rad/meter  Note: 2pi cancels
#define TOOL_ROT_TR_GREEN_ARM (float)( CAPSTAN_TOOL_RADIUS/CAPSTAN_RADIUS_GP32 * GEAR_BOX_GP32_TR * 120.0/180.0) // (asked 180, received 120)
#define WRIST_TR_GREEN_ARM    (float)( CAPSTAN_TOOL_RADIUS/CAPSTAN_RADIUS_GP32 * GEAR_BOX_GP32_TR * 200.0/180.0) //  RE-30, GP-32  ( /180 is a fudge factor put in by HK through observation. Was 140/180
#define GRASP1_TR_GREEN_ARM   (float)( CAPSTAN_TOOL_RADIUS/CAPSTAN_RADIUS_GP32 * GEAR_BOX_GP32_TR * 100.0/90.0) //  RE-30, GP-32 (100/90 is a fudge factor put in by HK through observation. Was 75/90
#define GRASP2_TR_GREEN_ARM   (float)( CAPSTAN_TOOL_RADIUS/CAPSTAN_RADIUS_GP32 * GEAR_BOX_GP32_TR * 100.0/90.0) //  RE-30, GP-32

#define SHOULDER_TR_GOLD_ARM   (SHOULDER_TR_GREEN_ARM)
#define ELBOW_TR_GOLD_ARM      (ELBOW_TR_GREEN_ARM)
#define Z_INS_TR_GOLD_ARM      (Z_INS_TR_GREEN_ARM)
#define TOOL_ROT_TR_GOLD_ARM   (TOOL_ROT_TR_GREEN_ARM)
#define WRIST_TR_GOLD_ARM      (WRIST_TR_GREEN_ARM)
#define GRASP1_TR_GOLD_ARM     (GRASP1_TR_GREEN_ARM)
#define GRASP2_TR_GOLD_ARM     (GRASP2_TR_GREEN_ARM)

//Link Angles/Lengths
const float A12  = 1.30899694;    /*Link1 - 75deg in RAD*/
#define A23 (float)(0.907571211)    /*Link2 - 52deg in RAD - was set to 60*/

//Kinematic Zero Offset (encoder space)
#define SHOULDER_GOLD_KIN_OFFSET (float)(0.0) //(62.0)
#define ELBOW_GOLD_KIN_OFFSET    (float)(0.0) //(-328.0)//d enc -offset
#define Z_INS_GOLD_KIN_OFFSET    (float)(0.0)

#define SHOULDER_GREEN_KIN_OFFSET SHOULDER_GOLD_KIN_OFFSET
#define ELBOW_GREEN_KIN_OFFSET    ELBOW_GOLD_KIN_OFFSET
#define Z_INS_GREEN_KIN_OFFSET    Z_INS_GOLD_KIN_OFFSET

//Kinematic Zero Offset (encoder space) RAVEN_I used differnet kin. eq's
//#define SHOULDER_A_KIN_OFFSET (float)(-62.0)
//#define ELBOW_A_KIN_OFFSET    (float)(-32.0)
//#define Z_INS_A_KIN_OFFSET    (float)(0.0)
//
//#define SHOULDER_B_KIN_OFFSET (float)(62.0)
//#define ELBOW_B_KIN_OFFSET    (float)(-328.0)
//#define Z_INS_B_KIN_OFFSET    (float)(0.0)


//InvKinematic Software Stops
//   Kinematics equations for R+IIare apparently formulated so that joint ranges are identical on each arm.
#define SHOULDER_MIN_LIMIT (float)(   0.0 DEG2RAD)
#define SHOULDER_MAX_LIMIT (float)(  90.0 DEG2RAD)
#define ELBOW_MIN_LIMIT (float)(  45.0 DEG2RAD)
#define ELBOW_MAX_LIMIT (float)( 135.0 DEG2RAD)

#define Z_INS_MIN_LIMIT (float)(-0.230)
#define Z_INS_MAX_LIMIT (float)( 0.010)

#define TOOL_ROLL_MIN_LIMIT (float)(-182.0 DEG2RAD)
#define TOOL_ROLL_MAX_LIMIT (float)( 182.0 DEG2RAD)
#define TOOL_WRIST_MIN_LIMIT (float)(-75.0 DEG2RAD)
#define TOOL_WRIST_MAX_LIMIT (float)( 75.0 DEG2RAD)

#define TOOL_GRASP_LIMIT ((float) 89.0 DEG2RAD)
#define TOOL_GRASP1_MIN_LIMIT (-TOOL_GRASP_LIMIT)
#define TOOL_GRASP1_MAX_LIMIT   TOOL_GRASP_LIMIT
#define TOOL_GRASP2_MIN_LIMIT (-TOOL_GRASP_LIMIT)
#define TOOL_GRASP2_MAX_LIMIT   TOOL_GRASP_LIMIT

#define TOOL_GRASP_COMMAND_MAX ((float) 90.0 DEG2RAD)
#define TOOL_GRASP_COMMAND_MIN ((float) -30.0 DEG2RAD)

/*#define TOOL_GRASP1_MIN_LIMIT (float)(-45.0 DEG2RAD)
#define TOOL_GRASP1_MAX_LIMIT (float)( 45.0 DEG2RAD)
#define TOOL_GRASP2_MIN_LIMIT (float)(-45.0 DEG2RAD)
#define TOOL_GRASP2_MAX_LIMIT (float)( 45.0 DEG2RAD)
*/

//DAC
#define DAC_OFFSET        0x8000  /* Used to convert DAC vals to midrange */

// Current limits
#define MAX_INST_DAC 6000 //32000

#define SHOULDER_MAX_DAC   2500   // 2000 usually moves 1000 doesn't
#define ELBOW_MAX_DAC      2500   //  ""
#define Z_INS_MAX_DAC      1500   //  1000 moves but doesn't overcome friction in tool joints
#define TOOL_ROT_MAX_DAC   1900  // 10000   These are set really low for safety sake
#define WRIST_MAX_DAC      1900  // 20000
#define GRASP1_MAX_DAC     1900  // 15000
#define GRASP2_MAX_DAC     1900

//Joint values for Homing
#define SHOULDER_HOME_ANGLE   M_PI/6
#define ELBOW_HOME_ANGLE      M_PI/2
#define Z_INS_HOME_ANGLE      -0.05
#define TOOL_ROT_HOME_ANGLE   0
#define WRIST_HOME_ANGLE      0
#define GRASP1_HOME_ANGLE     M_PI/4
#define GRASP2_HOME_ANGLE     M_PI/4

#define MICRON_PER_M      1000000.0
#define MICRORADS_PER_RAD 1000000.0

#define PI           ((float)3.1415926535)
#define ZERO_THRESHOLD ((float)0.000001)

//Time Defines
#define ONE_MS       ((float)0.001)
#define STEP_PERIOD  ONE_MS
#define SECOND       1000

//Speed Limits
#define V_MAX        ((float)3.0) /* rad/s   */
#define A_MAX        ((float)1.0) /* rad/s^2 */

//Grasping Defines
#define GRASP_OPEN   1
#define GRASP_CLOSE  0

//Footpedal states
#define PEDAL_UP 0
#define PEDAL_DN 1

//Watchdog timer Frequency
#define WD_PERIOD      50

//Master connection timeout (time to trigger pedal up)
#define MASTER_CONN_TIMEOUT 500 //5000


#endif
