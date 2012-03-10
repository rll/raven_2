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
#include "saveload.h"

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
        log_msg("\nERROR: Could not init USB. Boards on?");
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
    saveDOFInfo();
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

    pthread_create(&console_thread, NULL, console_process, NULL); //Start the     thread
    pthread_create(&rt_thread, NULL, rt_process, NULL); //Start the   thread
    pthread_join(rt_thread,NULL); //Suspend main until rt thread terminates

    log_msg("\n\n\nI'm shutting down now... Please close the USB!\n\n\n");
    usleep(1e6); //Sleep for 1 second

    exit(0);
}
