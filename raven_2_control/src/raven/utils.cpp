/*
 * utils.h - some common utility functions
 *      toShort - convert an int to a short int
 *
 * Kenneth Fodero
 * Biorobotics Lab 2005
 *
 */

#include <math.h>
#include "hmatrix.h"
#include "utils.h"
#include "DS0.h"
#include "defines.h"
#include "log.h"

#include <raven/control/control_input.h>

extern int NUM_MECH;
extern bool disable_arm_id[2];

float fix_angle(float angle,float center) {
	float test_angle = angle;
	int cnt = 1;
	while ((test_angle-center) > M_PI) {
		test_angle = angle - cnt * 2*M_PI;
		cnt++;
	}
	angle = test_angle;
	cnt = 1;
	while ((test_angle-center) < -M_PI) {
		test_angle = angle + cnt * 2*M_PI;
		cnt++;
	}
	return test_angle;
}

float saturatef(float value,float min,float max) {
	if (value > max) {
		return max;
	} else if (value < min) {
		return min;
	}
	return value;
}

int saturate(int value,int min,int max) {
	if (value > max) {
		return max;
	} else if (value < min) {
		return min;
	}
	return value;
}

/*
btTransform Z(float theta,float d) {
	btMatrix3x3 rot(
			cos(theta), -sin(theta), 0,
			sin(theta),  cos(theta), 0,
			0,           0,          1);
	btVector3 shift(0,0,d);
	return btTransform(rot,shift);
}

btTransform X(float alpha,float a) {
	btMatrix3x3 rot(
			1, 0,           0,
			0, cos(alpha), -sin(alpha),
			0, sin(alpha),  cos(alpha));
	btVector3 shift(a,0,0);
	return btTransform(rot,shift);
}
*/


/*
 * toShort - function that takes an integer and places it in the
 *   target short int, reporting under/overflow
 *
 * inputs - value
 *
 */
int toShort(int value, short int *target)
{

    //Overflow
    if (value > SHORT_MAX)
    {
        *target = SHORT_MAX;
        return SHORT_OVERFLOW;
    }
    //Underflow
    else if (value < SHORT_MIN)
    {
        *target = SHORT_MIN;
        return SHORT_UNDERFLOW;
    }
    //No problems
    else
    {
        *target = value;
        return 0;
    }
}

int loop_over_mechs(struct robot_device* device0, struct mechanism*& _mech, int& mechnum) {
	// Initialize iterators
	if (_mech == NULL) {
		mechnum = 0;
	} else if (mechnum >= (NUM_MECH-1)) {
		return 0;
	} else {
		mechnum++;
	}
	while (disable_arm_id[armIdFromMechType(device0->mech[mechnum].type)]) {
		if (mechnum >= (NUM_MECH-1)) {
			return 0;
		} else {
			mechnum++;
		}
	}
	_mech = &(device0->mech[mechnum]);
	return 1;
}

/**
*    Iterate over all joints of all mechanisms.
*    To start iteration, call function with _joint == _mech == NULL
*    Function iterates by incrementing jnum and mnum from zero and returning the appropriate mech and joint.
*    Iteration terminates when function is called with jnum == MAX_DOF-1 and mnum = NUM_MECH-1.
*       In the terminating case, mnum and jnum are not modified.  _joint and _mech are also unmodified in that case.
*
*    Precondition:  mechnum and jnum point to _previous_ joint.
*                   At start of iteration _mech == _joint == NULL
*
*    Postcondition: jnum and mnum are incremented or reset as necessary
*                   _mech points to device0.mech[mechnum]
*                   _joint points to device0.mech[mechnum].joint[jnum+1]
*/
int loop_over_joints(struct robot_device* device0, struct mechanism*& _mech, struct DOF*& _joint, int& mechnum, int& jnum)
{
    // Initialize iterators
    if (_mech == NULL || _joint == NULL)
    {
        mechnum = 0;
        jnum = 0;
    }

    // Terminating condition
    else if ( mechnum >= (NUM_MECH-1) && jnum >= (MAX_DOF_PER_MECH-1) )
        return 0;

    // Joint rollover
    else if ( jnum >= (MAX_DOF_PER_MECH-1) )
    {
        mechnum++;
        jnum=0;
    }

    // Joint increment
    else
    {
        jnum++;
        if (jnum == NO_CONNECTION) jnum++;
    }

    //check if mech disabled
    while (disable_arm_id[armIdFromMechType(device0->mech[mechnum].type)]) {
    	if (mechnum >= (NUM_MECH-1)) {
    		return 0;
    	} else {
    		mechnum++;
    		jnum=0;
    	}
    }

    // Set return structs
    _mech = &(device0->mech[mechnum]);
    _joint =&(device0->mech[mechnum].joint[jnum]);

    return 1;
}

/**
*    Iterate over all joints of one mechanism.
*    To start iteration, call function with _joint == NULL
*    Function iterates by incrementing jnum from zero and returning the appropriate joint.
*    Iteration terminates when function is called with jnum == MAX_DOF-1.
*       In the terminating case, jnum is not modified.  _joint is also unmodified in that case.
*
*    Precondition:  jnum point to _previous_ joint.
*                   At start of iteration _joint == NULL
*
*    Postcondition: jnum is incremented or reset as necessary
*                   _joint points to mech->joint[jnum+1]
*/
int loop_over_joints(struct mechanism* _mech, struct DOF*& _joint, int& jnum)
{
	if (disable_arm_id[armIdFromMechType(_mech->type)]) {
		return 0;
	}
    // Initialize iterators
    if (_joint == NULL)
    {
        jnum = 0;
    }

    // Terminating condition
    else if ( jnum >= (MAX_DOF_PER_MECH-1) )
        return 0;

    // Joint increment
    else
    {
        jnum++;
        if (jnum == NO_CONNECTION) jnum++;
    }

    // Set return structs
    _joint =&(_mech->joint[jnum]);

    return 1;
}


int is_toolDOF(struct DOF *_joint){
    return is_toolDOF(_joint->type);
}

int is_toolDOF(int jointType)
{
    if ( jointType == TOOL_ROT_GOLD || jointType == TOOL_ROT_GREEN)
        return 1;
    if ( jointType == WRIST_GOLD    || jointType == WRIST_GREEN)
        return 1;
    if ( jointType == GRASP1_GOLD   || jointType == GRASP1_GREEN)
        return 1;
    if ( jointType == GRASP2_GOLD   || jointType == GRASP2_GREEN)
        return 1;

    return 0;
}
int tools_ready(struct mechanism *mech)
{
	if (disable_arm_id[armIdFromMechType(mech->type)]) {
		return 1;
	}
    if ( mech->joint[TOOL_ROT].state != jstate_ready )
        return 0;
    if ( mech->joint[WRIST].state    != jstate_ready )
        return 0;
    if ( mech->joint[GRASP1].state   != jstate_ready )
        return 0;
    if ( mech->joint[GRASP2].state   != jstate_ready )
        return 0;

    return 1;
}


int robot_ready(struct robot_device* device0)
{
    struct mechanism* _mech = NULL;
    struct DOF* _joint = NULL;
    int i, j;

    while ( loop_over_joints(device0, _mech, _joint, i, j) )
    {
    	if (disable_arm_id[armIdFromMechType(_mech->type)]) {
    		continue;
    	}
        if (_joint->state != jstate_ready)
            return 0;
    }
    return 1;
}


/*
 * strtok - function to tokenize a string
 *
 * inputs - str - the string
 *          result - the resulting string
 *          delim - the delimeter
 *
 * returns location of delimeter
 */
void strtoken(char *str, char *result, char delim)
{
    static char data[200] = {0};
    static int index = 0;
    int i = 0;

    //Copy over string
    if (str != NULL)
    {
        strcopy(str, data);
        index = 0;
    }

    //Loop through for delimeter
    while (data[index] != '\0')
    {
        //Found delimeter
        if (data[index] == delim)
        {
            result[i] = '\0';
            index++;
            return;
        }

        result[i++] = data[index];
        index++;
    }

    result[i] = NULL;
    return;
}

void strcopy(const char *src, char *dest)
{
    int i = 0;

    while (src[i] != '\0')
    {
        dest[i] = src[i];
        i++;
    }

    dest[i] = NULL;
}

/**
 * tsSubtract() - returns time1-time2  or  (0,0) if time2>time1
 */
struct  timespec  tsSubtract ( struct  timespec  time1,
                                           struct  timespec  time2)
{
    struct  timespec  result ;

    /* Subtract the second time from the first. */
    if ((time1.tv_sec < time2.tv_sec) ||
            ((time1.tv_sec == time2.tv_sec) &&
             (time1.tv_nsec <= time2.tv_nsec)))   /* TIME1 <= TIME2? */
    {
        result.tv_sec = result.tv_nsec = 0 ;
    }
    else                                      /* TIME1 > TIME2 */
    {
        result.tv_sec = time1.tv_sec - time2.tv_sec ;
        if (time1.tv_nsec < time2.tv_nsec)
        {
            result.tv_nsec = time1.tv_nsec + 1000000000L - time2.tv_nsec ;
            result.tv_sec-- ;                    /* Borrow a second. */
        }
        else
        {
            result.tv_nsec = time1.tv_nsec - time2.tv_nsec ;
        }
    }
    return (result);
}

float copysignf(float a,float b);
void getQuaternion(float* Q, float mat[3][3])
{
    Q[_Qw] = sqrt( fmax( 0, 1 + mat[0][0] + mat[1][1] + mat[2][2] ) ) / 2;
    Q[_Qx] = sqrt( fmax( 0, 1 + mat[0][0] - mat[1][1] - mat[2][2] ) ) / 2;
    Q[_Qy] = sqrt( fmax( 0, 1 - mat[0][0] + mat[1][1] - mat[2][2] ) ) / 2;
    Q[_Qz] = sqrt( fmax( 0, 1 - mat[0][0] - mat[1][1] + mat[2][2] ) ) / 2;

    Q[_Qx] = copysignf( Q[1], mat[2][1] - mat[1][2] );
    Q[_Qy] = copysignf( Q[2], mat[0][2] - mat[2][0] );
    Q[_Qz] = copysignf( Q[3], mat[1][0] - mat[0][1] );
}

void set_posd_to_pos(struct robot_device* device0)
{
    for (int m = 0; m < NUM_MECH; m++) {
#ifdef USE_NEW_DEVICE
    	ArmPtr arm = Device::currentNoClone()->getArmById(device0->mech[m].type);
    	btTransform tf = toBt(device0->mech[m].pos,device0->mech[m].ori);
    	ControlInput::getOldControlInput()->armById(device0->mech[m].type).pose() = tf;
    	ControlInput::getOldControlInput()->armById(device0->mech[m].type).grasp() = arm->joint(Joint::Type::GRASP_)->position();
#endif
    	device0->mech[m].pos_d.x     = device0->mech[m].pos.x;
        device0->mech[m].pos_d.y     = device0->mech[m].pos.y;
        device0->mech[m].pos_d.z     = device0->mech[m].pos.z;
        device0->mech[m].ori_d.yaw   = device0->mech[m].ori.yaw;
        device0->mech[m].ori_d.pitch = device0->mech[m].ori.pitch;
        device0->mech[m].ori_d.roll  = device0->mech[m].ori.roll;
        device0->mech[m].ori_d.grasp = device0->mech[m].ori.grasp;

        for (int k = 0; k < 3; k++) {
          for (int j = 0; j < 3; j++) {
              device0->mech[m].ori_d.R[k][j] = device0->mech[m].ori.R[k][j];
          }
        }
    }
}
