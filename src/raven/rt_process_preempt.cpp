/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University of Washington BioRobotics Laboratory
 *
 * This file is part of Raven 2 Control.
 *
 * Raven 2 Control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven 2 Control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * PREEMPT_RT Raven control implementation
 * RTAI version info
 *------------------------------------------------*
 *   Added data_router module
 *   Data_router module put in ifdef
 * RTAI version written by Ken Fodero, Hawkeye King
 * BioRobotics Lab, University of Washington
 * ken@ee.washington.edu
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h> // Needed for mlockall()
#include <unistd.h> // needed for sysconf(int name);
#include <malloc.h>
#include <sys/time.h> // needed for getrusage
#include <sys/resource.h> // needed for getrusage
#include <sched.h>
#include <stropts.h>
#include <time.h>
#include <pthread.h>
#include <string.h>
#include <iostream>
#include <fcntl.h>
#include <sys/stat.h> //Needed for umask

#include <ros/ros.h>     // Use ROS

#include "rt_process_preempt.h"
#include "console_process.h"
#include "rt_raven.h"
#include "network_layer.h"

using namespace std;

// Defines
#define POOLSIZE (200*1024*1024) // 200 MB   Size of mlocked memory pool

#define NS  1
#define US  (1000 * NS)
#define MS  (1000 * US)
#define SEC (1000 * MS)
#define PORT            0x378 //0xec00      // address of parallel port for debugging

//Global Variables
unsigned long int gTime;
int initialized=0;     // State initialized flag
int soft_estopped=0;   // Soft estop flag- indicate desired software estop.

int    deviceType = SURGICAL_ROBOT;//PULLEY_BOARD;
struct device device0 ={0};  //Declaration Moved outside rt loop for access from console thread
int    mech_gravcomp_done[2]={0};

int NUM_MECH=0;   // Define NUM_MECH as a C variable, not a c++ variable

pthread_t rt_thread;
pthread_t net_thread;
pthread_t fiforcv_thread;
pthread_t fifosend_thread;
pthread_t console_thread;

//Global Variables from globals.c
extern struct DOF_type DOF_types[];

/**
*From PREEMPT_RT Dynamic memory allocation tips page.
*This function creates a pool of memory in ram for use with any malloc or new calls so that they do not cause page faults.
*https://rt.wiki.kernel.org/index.php/Dynamic_memory_allocation_example
*/
int initialize_rt_memory_pool()
{
    int i, page_size;
    char* buffer;

    // Now lock all current and future pages from preventing of being paged
    if (mlockall(MCL_CURRENT | MCL_FUTURE ))
    {
        perror("mlockall failed:");
        return -1;
    }
    mallopt (M_TRIM_THRESHOLD, -1);  // Turn off malloc trimming.
    mallopt (M_MMAP_MAX, 0);         // Turn off mmap usage.

    page_size = sysconf(_SC_PAGESIZE);
    buffer = (char *)malloc(POOLSIZE);

    // Touch each page in this piece of memory to get it mapped into RAM for performance improvement
    // Once the pagefault is handled a page will be locked in memory and never given back to the system.
    for (i=0; i < POOLSIZE; i+=page_size)
    {
        buffer[i] = 0;
    }
    free(buffer);        // buffer is now released but mem is locked to process

    return 0;
}

/**
 * This is the real time thread.
 *
 */
static void *rt_process(void* )
{
    struct param_pass currParams =
    {
        0
    };          // robot command struct
    struct param_pass rcvdParams =
    {
        0
    };
    struct sched_param param;                    // process / thread priority settings
    struct timespec t;                           // Tracks the timer value
    int interval= 1 * MS;                        // task period in nanoseconds

    //Lock thread to first available CPU
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(0,&set);
    sched_setaffinity(0,sizeof(set),&set);

    log_msg("Using realtime, priority: %d",96);
    param.sched_priority = 96;

    // enable realtime fifo scheduling and set process priority
    if (sched_setscheduler(0, SCHED_FIFO, &param)==-1)
    {
        perror("sched_setscheduler failed");
        exit(-1);
    }

    currParams.runlevel = STOP;
    currParams.sublevel = 0;

    log_msg("Starting RT Process..");

    // Initializations (run here and again in init.cpp)
    initDOFs(&device0);

    // initialize global loop count
    gTime=0;

    // Setup periodic timer
    clock_gettime(CLOCK_REALTIME,&t);   // get current time
    t.tv_sec += 1;                      // start after short delay
    tsnorm(&t);

    log_msg("*** Ready to teleoperate ***");

    ///TODO: Break loop when board becomes disconnected.
    //Only run while USB board is attached
    while (ros::ok())
    {
        /// SLEEP until next timer shot
        clock_nanosleep(0, TIMER_ABSTIME, &t, NULL);
        gTime++;

        //Get and Process USB Packets
        getUSBPackets(&device0); //disable usb for parport test

        //Run Safety State Machine
        stateMachine(&device0, &currParams, &rcvdParams);

        //Update Atmel Input Pins
        // TODO: deleteme
        updateAtmelInputs(device0, currParams.runlevel);

        //Get state updates from master
        if ( checkLocalUpdates() == TRUE)
            updateDeviceState(&currParams, getRcvdParams(&rcvdParams), &device0);
        else
            rcvdParams.runlevel = currParams.runlevel;

        //Clear DAC Values (set current_cmd to zero on all joints)
        clearDACs(&device0);

        //////////////// SURGICAL ROBOT CODE //////////////////////////
        if (deviceType == SURGICAL_ROBOT)
        {
            // Calculate Raven control
            controlRaven(&device0, &currParams);
        }
        //////////////// END SURGICAL ROBOT CODE ///////////////////////////

        // Check for overcurrent and impose safe torque limits
        if (overdriveDetect(&device0))
            soft_estopped = TRUE;

        //Update Atmel Output Pins
        updateAtmelOutputs(&device0, currParams.runlevel);

        //Fill USB Packet and send it out
        putUSBPackets(&device0); //disable usb for par port test

        //Publish current raven state
        publish_ravenstate_ros(&device0,currParams.runlevel,currParams.sublevel);   // from local_io

        t.tv_nsec+=interval; //Update timer count for next clock interrupt
        tsnorm(&t);

        //Done for this cycle
    }


    log_msg("Raven Control is shutdown");
    return 0;
}

/**Initialize by initializing usb boards, etc*/
int init_module(void)
{
    log_msg("Initializing USB I/O...");

    //Initiailze USB Board
    if (USBInit(&device0) == FALSE)
    {
        err_msg("\nERROR: Could not init USB. Boards on?");
        return STARTUP_ERROR;
    }

    // Initialize Local_io datastructs.
    log_msg("Initializing Local I/O...");
    initLocalioData();

    return 0;
}

int init_ros(int argc, char **argv)
{
    /**
    * Initialize ros and rosrt
    */
    ros::init(argc, argv, "r2_control");
    ros::NodeHandle n;
//    rosrt::init();
    init_ravenstate_publishing(n);
    init_ravengains(n, &device0);
    init_joint_subs(n);

    return 0;
}

int main(int argc, char **argv)
{

    //signal( SIGINT,&sigTrap);                // catch ^C for graceful close.  Unused under ROS
    ioperm(PORT,1,1);                        // set parallelport permissions
    if ( init_module() )
    {
        cerr << "ERROR! Failed to init module.  Exiting.\n";
        exit(1);
    }
    if ( init_ros(argc, argv) )
    {
        cerr << "ERROR! Failed to init ROS.  Exiting.\n";
        exit(1);
    }
    if ( initialize_rt_memory_pool() )
    {
        cerr << "ERROR! Failed to init memory_pool.  Exiting.\n";
        exit(1);
    }

    pthread_create(&net_thread, NULL, network_process, NULL); //Start the network thread
//    pthread_create(&fiforcv_thread, NULL, data_fifo_rcv_process, NULL); //Start the    thread
//    pthread_create(&fifosend_thread, NULL, data_fifo_send_process, NULL); //Start the   thread
    pthread_create(&console_thread, NULL, console_process, NULL); //Start the     thread
    pthread_create(&rt_thread, NULL, rt_process, NULL); //Start the   thread
    pthread_join(rt_thread,NULL); //Suspend main until rt thread terminates

    log_msg("\n\n\nI'm shutting down now... \n\n\n");
    usleep(1e6); //Sleep for 1 second

    exit(0);
}






















/** This function dumps device (robot states, torques etc etc) info to FIFO
* TODO: Should this run at a higher priority?? If the realtime code consumes all cpu cycles, this thread may not run at all and the system will be unable
* to send any data out
DEPRECATED since we're not using the toolkit anymore.  Besides, it's stupid to output status info from a different thread.
TODO: Deleteme
*/
/*
void *data_fifo_send_process(void *)
{
    int fd=0;
    int dummy_rd_fd=0;
    struct sched_param param;                    // priority settings
    param.sched_priority = 0;
    if (sched_setscheduler(0, SCHED_OTHER, &param)==-1)
    {
        perror("data_fifo_send_process: sched_setscheduler failed in ");
        exit(-1);
    }
    unlink("/tmp/fifosend");
    umask(0);
    fd=mkfifo("/tmp/fifosend",S_IWUSR|S_IRUSR|S_IROTH|S_IWOTH);
    if (fd<0)
    {
        printf("recieve fifo error\n");
        return 0;
    }
    fd=open("/tmp/fifosend",O_WRONLY); //OPen writing end with blocking writes
    if (fd==-1)
    {
        cout<<"Invalid write fifo\n";
        return 0;
    }
    dummy_rd_fd=open("/tmp/fifosend",O_RDONLY); ///TODO: Figure out a better mechanism to prevent sigpipe when toolkit closes pipe after ning//Open dummy
    ///file to suppress broken pipe signal when the toolkit closes.

    while (ros::ok())
    {
        senddatafromoutputbuffer(fd);
    }
    log_msg("Data output is shutdown");

    return NULL;
}
*/

/** This function recieves toolkit information
* TODO: Should this run at a higher priority?? If the realtime code consumes all cpu cycles, this thread may not run at all and the system will be unable
* to respond to the toolkit. But if we are using all cpu cycles for control, we shouldn't be doing this anyway.
*/
/*
DEPRECATED since we're not using a toolkit.
TODO: Deleteme
void *data_fifo_rcv_process(void *)
{
    int fd=0;
    int dummy_wr_fd=0;
    struct sched_param param;                    // priority settings
    param.sched_priority = 0;
    if (sched_setscheduler(0, SCHED_OTHER, &param)==-1)
    {
        perror("sched_setscheduler failed");
        exit(-1);
    }
    unlink("/tmp/fiforcv");
    umask(0);
    fd=mkfifo("/tmp/fiforcv",S_IRUSR|S_IWOTH);
    if (fd<0)
    {
        printf("recieve fifo error\n");
        return 0;
    }
    fd=open("/tmp/fiforcv",O_RDONLY);
    if (fd==-1)
    {
        cout<<"Invalid write fifo\n";
        return 0;
    }
    dummy_wr_fd=open("/tmp/fiforcv",O_WRONLY); ///TODO: Figure out way to detect close of other end. Keep open to prevent continuous EOF reads since closed pipe returns EOF on read and causes loop to run quickly.
    while (ros::ok())
    {
        if (recieveToolkit(fd)==-1)
        {
            //usleep(1e4); //Reading fifo failed. Wait for sometime.
        }
    }

    log_msg("Data input process is shutdown");
    return (NULL);
}

*/

/// Trap ctrl-C
// unused.  Now trapped by ROS
/*void sigTrap(int sig)
{
    runable = 0;
    usleep(10e3);
    pthread_cancel(rt_thread);
}*/



/*

SAMPLE CODE FOR TESTING f/inv kinematics
{0.386,	0.057,	-0.921},
{-0.201,	-0.969,	-0.144},
{-0.900,	0.240,	-0.363}}

            printf("\n\n Test 1:-------------\n");
            struct mechanism mechTest;
            mechTest.type=GREEN_ARM;
            mechTest.pos_d.x = 6382;
            mechTest.pos_d.y = 1367;
            mechTest.pos_d.z = 2531;
            double o[3][3] = {{0.386,	0.057,	-0.921},{-0.201,	-0.969,	-0.144},{-0.900,	0.240,	-0.363}};
            for (int i=0;i<3;i++)
                for (int j=0; j<3; j++)
                    mechTest.ori_d.R[i][j]=o[i][j];
            mechTest.ori_d.grasp = 0;
            mechTest.joint[GRASP1].tau = 0;
            mechTest.joint[GRASP2].tau = 0;
            printf("pos_d: (%d,\t%d\t%d)\nori_d:\t(%.3f,\t%.3f,\t%.3f)\n\t(%.3f,\t%.3f,\t%.3f)\n\t(%.3f,\t%.3f,\t%.3f)\n",
                mechTest.pos_d.x,mechTest.pos_d.y,mechTest.pos_d.z,
                mechTest.ori_d.R[0][0],mechTest.ori_d.R[0][1],mechTest.ori_d.R[0][2],
                mechTest.ori_d.R[1][0],mechTest.ori_d.R[1][1],mechTest.ori_d.R[1][2],
                mechTest.ori_d.R[2][0],mechTest.ori_d.R[2][1],mechTest.ori_d.R[2][2]);

            int ret=invMechKin(&mechTest);
            printf("\ninv kin returned: %d\n", ret);
            for(int i=0;i<MAX_DOF_PER_MECH;i++){
                mechTest.joint[i].jpos=0;
                printf("joint (p/d) %d: %.3f\t %.3f\n",i, mechTest.joint[i].jpos,mechTest.joint[i].jpos_d);
                mechTest.joint[i].jpos=mechTest.joint[i].jpos_d;
            }
            fwdMechKin(&mechTest);
            printf("\nfwd kin returned.\n");

            printf("pos: (%d,\t%d\t%d)\nori:\t(%.3f,\t%.3f,\t%.3f)\n\t(%.3f,\t%.3f,\t%.3f)\n\t(%.3f,\t%.3f,\t%.3f)\n",
                mechTest.pos.x,mechTest.pos.y,mechTest.pos.z,
                mechTest.ori.R[0][0],mechTest.ori.R[0][1],mechTest.ori.R[0][2],
                mechTest.ori.R[1][0],mechTest.ori.R[1][1],mechTest.ori.R[1][2],
                mechTest.ori.R[2][0],mechTest.ori.R[2][1],mechTest.ori.R[2][2]);


            printf("\n\n Test 2:-------------\n");
            mechTest.pos_d.x=mechTest.pos.x;
            mechTest.pos_d.y=mechTest.pos.y;
            mechTest.pos_d.z=mechTest.pos.z;

            printf("set pos_d=pos\n");
            ret=invMechKin(&mechTest);
            printf("inv kin returned: %d\n", ret);
            for(int i=0;i<MAX_DOF_PER_MECH;i++){
                printf("joint (p/d) %d: %.3f\t %.3f\n",i, mechTest.joint[i].jpos,mechTest.joint[i].jpos_d);
            }
            printf("\n\n\n");
            break;
*/



/*

This code snippet moved to rt_raven.cpp

            //Initialization code
            initSurgicalArms(&device0, currParams.runlevel, &currParams);

            //Inverse kinematics
            invKin(&device0, &currParams);

            //Inverse Cable Coupling
            invCableCoupling(&device0, currParams.runlevel);

            //Compute Velocities
            //computeVelocity(&device0);
            stateEstimate(&device0);

            //Run Control
            // PD control should run in auto-init run/sublevel.
            // PD control should run in pedal-down runlevel.
            if ( currParams.runlevel == RL_PEDAL_DN ||
                    ( currParams.runlevel == RL_INIT &&
                      currParams.sublevel == SL_AUTO_INIT ))
            {
                pdController(&device0, MOTOR_PD_CTRL, currParams.runlevel);
                gravComp(&device0);
                TorqueToDAC(&device0);
            }

            //Foward Cable Coupling
            fwdCableCoupling(&device0, currParams.runlevel);

            //Forward kinematics
            fwdKin(&device0, currParams.runlevel);

*/
