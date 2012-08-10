/**
*  File: console_process.cpp
*
*  Outputs data to the console periodically, so that we know our robot is alives.
*/

#include <stdio.h>
#include <iomanip>
#include <termios.h>   // needed for terminal settings in getkey()

#include "rt_process_preempt.h"
#include "rt_raven.h"

using namespace std;

// from rt_process.cpp
extern struct device device0;

extern unsigned long int gTime;
extern int soft_estopped;
extern struct DOF_type DOF_types[];

void outputRobotState();
int getkey();

/**
* This is the thread dedicated to console io
*/
void *console_process(void *)
{
    ros::Time t1, t2;
    ros::Duration d;
    t1=t1.now();
    t2=t2.now();

    //Low priority non realtime thread
    struct sched_param param;                    // priority settings
    param.sched_priority = 0;
    if (sched_setscheduler(0, SCHED_OTHER, &param)==-1)
    {
        perror("sched_setscheduler failed for console process");
        exit(-1);
    }

    int output_robot=false;
    int theKey,print_msg=1;
    char inputbuffer[100];
    sleep(1);
    // Run shell interaction
    while (ros::ok())
    {
        // Output UI hints
        if ( print_msg ){
            log_msg("[[\t'C'  : toggle console messages ]]");
            log_msg("[[\t'T'  : specify joint torque    ]]");
            log_msg("[[\t'M'  : set control mode        ]]");
            log_msg("[[\t'^C' : Quit                    ]]");
            print_msg=0;
        }
        // Get UI command
        theKey = getkey();
        switch (theKey){
            case 'z':
            {
                output_robot = 0;
                setDofTorque(0,0,0);
                log_msg("Torque zero'd");
                print_msg=1;
                break;
            }

            case 'e':
            case 'E':
            case '0':
            {
                output_robot = 0;
                soft_estopped=TRUE;
                print_msg=1;
                log_msg("Soft estopped");
                break;
            }
            case '+':
            case '=':
            {
                soft_estopped=FALSE;
                print_msg=1;
                log_msg("Soft estop off");
                break;
            }
            case 'c':
            case 'C':
            {
                log_msg("Console output on:%d", output_robot);
                output_robot = !output_robot;
                print_msg=1;
                break;
            }
            case 't':
            case 'T':
            {
                print_msg=1;
                // Get user-input mechanism #
                printf("\n\nEnter a mechanism number: 0-Gold, 1-Green:\t");
                cin.getline (inputbuffer,100);
                unsigned int _mech = atoi(inputbuffer);
                if ( _mech > 1 ) break;

                // Get user-input joint #
                printf("\nEnter a joint number: 0-shoulder, 1-elbow, 2-zins, 4-tool_rot, 5-wrist, 6/7- grasp 1/2:\t");
                cin.getline (inputbuffer,100);
                unsigned int _joint = atoi(inputbuffer);
                if ( _joint > MAX_DOF_PER_MECH ) break;

                // Get user-input DAC value #
                printf("\nEnter a torque value in miliNewton-meters:\t");
                cin.getline (inputbuffer,100);
                int _torqueval = atoi(inputbuffer);

                log_msg("Commanded mech.joint (tau):%d.%d (%d))\n",_mech, _joint, _torqueval);
                setDofTorque(_mech, _joint, _torqueval);
                break;
            }
            case 'm':
            case 'M':
            {
                // Get user-input DAC value #
                printf("\n\nEnter new control mode: 0=NULL, 1=NULL, 2=joint_velocity, 3=apply_torque, 4=homing, 5=motor_pd, 6=cartesian_space_motion, 7=multi_dof_sinusoid 8=joint_position \t");
                cin.getline (inputbuffer,100);
                t_controlmode _cmode = (t_controlmode)(atoi(inputbuffer));
                log_msg("recieved control mode:%d\n\n",_cmode);
                setRobotControlMode(_cmode);
                print_msg=1;
                break;
            }
        }

        // Output the robot state once/sec
        if ( output_robot        &&
             (t1.now()-t1).toSec() > 1 )
        {
            outputRobotState();
            t1=t1.now();
        }

//        printf("g: %1.5f %1.5f\n",device0.mech[0].joint[GRASP1].jpos,device0.mech[1].joint[GRASP1].jpos);
//        printf("d: %1.5f %1.5f\n\n",device0.mech[0].joint[GRASP1].jpos_d,device0.mech[1].joint[GRASP1].jpos_d);

        usleep(1e5); //Sleep for 1/10 second
    }

    return(NULL);
}

int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}

void outputRobotState(){
    cout << "Runlevel: " << static_cast<unsigned short int>(device0.runlevel) << endl;
    mechanism* _mech = NULL;
    int mechnum=0;
    while (loop_over_mechs(&device0,_mech,mechnum)) {
        if (_mech->type == GOLD_ARM)
            cout << "Gold arm:\t";
        else if (_mech->type == GREEN_ARM)
            cout << "Green arm:\t";
        else
            cout << "Unknown arm:\t";

        cout << "Board " << mechnum << ", type " << _mech->type << ":" << endl;
//
//        cout<<"pos:\t";
//        cout<<_mech->pos.x<<"\t";
//        cout<<_mech->pos.y<<"\t";
//        cout<<_mech->pos.z<<"\n";
//
//        cout<<"pos_d:\t";
//        cout<<_mech->pos_d.x<<"\t";
//        cout<<_mech->pos_d.y<<"\t";
//        cout<<_mech->pos_d.z<<"\n";

        DOF* _joint = NULL;
        int jnum=0;

        cout<<"type:\t\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	cout << _joint->type<<"\t";
        cout << endl;

        cout<<"enc_val:\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	cout << _joint->enc_val<<"\t";
        cout << endl;

        cout<<"enc_off:\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	cout << _joint->enc_offset<<"\t";
        cout << endl;

        cout<<"mpos:\t\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	cout<<fixed<<setprecision(2)<<_joint->mpos <<"\t";
        cout << endl;

        cout<<"mpos_d:\t\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	cout<<fixed<<setprecision(2)<<_joint->mpos_d <<"\t";
        cout << endl;

        cout<<"mvel:\t\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	cout<<fixed<<setprecision(0)<<_joint->mvel <<"\t";
        cout << endl;

        cout<<"mvel_d:\t\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	cout<<fixed<<setprecision(0)<<_joint->mvel_d <<"\t";
        cout << endl;

        cout<<"jpos:\t\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	if (jnum != Z_INS)
        		cout<<fixed<<setprecision(2)<<_joint->jpos <<"\t";
        	else
        		cout<<fixed<<setprecision(2)<<_joint->jpos<<"\t";
        cout << endl;

        cout<<"jpos_d:\t\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	if (jnum != Z_INS)
        		cout<<fixed<<setprecision(2)<<_joint->jpos_d <<"\t";
        	else
        		cout<<fixed<<setprecision(2)<<_joint->jpos_d<<"\t";
        cout << endl;

        cout<<"jvel:\t\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	if (jnum != Z_INS)
        		cout<<fixed<<setprecision(2)<<_joint->jvel <<"\t";
        	else
        		cout<<fixed<<setprecision(2)<<_joint->jvel<<"\t";
        cout << endl;

        cout<<"jvel_d:\t\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	if (jnum != Z_INS)
        		cout<<fixed<<setprecision(2)<<_joint->jvel_d <<"\t";
        	else
        		cout<<fixed<<setprecision(2)<<_joint->jvel_d<<"\t";
        cout << endl;

        cout<<"tau_d:\t\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	cout<<fixed<<setprecision(3)<<_joint->tau_d<<"\t";
        cout << endl;

        cout<<"DAC:\t\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	cout<<fixed<<setprecision(3)<<_joint->current_cmd<<"\t";
        cout << endl;

        cout<<"KP gains:\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	cout<<fixed<<setprecision(3)<<DOF_types[mechnum*MAX_DOF_PER_MECH+jnum].KP<<"\t";
        cout << endl;

        cout<<"KD gains:\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
            cout<<fixed<<setprecision(3)<<DOF_types[mechnum*MAX_DOF_PER_MECH+jnum].KD<<"\t";
        cout << endl;

        cout<<"KI gains:\t";
        _joint = NULL; jnum=0;
        while (loop_over_joints(_mech,_joint,jnum))
        	cout<<fixed<<setprecision(3)<<DOF_types[mechnum*MAX_DOF_PER_MECH+jnum].KI<<"\t";
        cout << endl;

//        cout<<"enc_offset:\t";
//        _joint = NULL; jnum=0;
//        while (loop_over_joints(_mech,_joint,jnum))
//            cout<<_joint->enc_offset<<"\t";
//        cout << endl;


        cout << endl;
    }
}

