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
#include "ros_io.h"

#include "rt_process_preempt.h"
#include "console_process.h"
#include "rt_raven.h"
#include "network_layer.h"
#include "control_process.h"
#include "saveload.h"

#include <raven/state/runlevel.h>
#include <raven/util/timing.h>

#include <raven/state/initializer.h>

#include <raven/control/controller.h>
#include <raven/control/controllers/motor_position_pid.h>
#include <raven/control/controllers/end_effector_control.h>

using namespace std;

//#define TEST_NEW_CTRL

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

bool disable_arm_id[2] = {false,false};

pthread_t rt_thread;
pthread_t control_thread;
pthread_t net_thread;
pthread_t fiforcv_thread;
pthread_t fifosend_thread;
pthread_t console_thread;

//Global Variables from globals.c
extern struct DOF_type DOF_types[];

extern bool omni_to_ros;

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

#ifndef USE_NEW_RUNLEVEL
    currParams.runlevel = STOP;
    currParams.sublevel = 0;
#endif

    log_msg("Starting RT Process...");

    // Initializations (run here and again in init.cpp)
#ifdef USE_NEW_DEVICE
    DeviceInitializer initr;
    initr.initializeDeviceInstance();
#endif
    initDOFs(&device0);

#ifdef TEST_NEW_CTRL
    {
    	log_msg("Creating controllers");

    	ControllerPtr pid(new MotorPositionPID());
		ControllerPtr ee(new EndEffectorController());

    	log_msg("Registering controllers");

    	Controller::registerController("motor/position",pid);
		Controller::registerController("end_effector/pose",ee);
		Controller::registerController("end_effector/grasp+pose",ee);

    	log_msg("Setting controller");
    	//Controller::setController("motor/position");
		Controller::setController("end_effector/pose");
    }
#endif

    /*{
    	int numIter=50000;
    	{
    		std::cout << "beginning test 1" << std::endl;
    		ros::Time start = ros::Time::now();
    		for (int i=0;i<numIter;i++) {
    			Device::currentNoClone()->arms()[0]->clone();
    		}
    		ros::Time finish = ros::Time::now();
    		ros::Duration span = finish-start;
    		std::cout << numIter << " iterations of clone took     " << span.toSec() << "s, avg=" << 1000*span.toSec()/numIter << "ms" << std::endl;
    		std::cout << 0.001 / (span.toSec()/numIter) << " per ms" << std::endl;
    	}
    	{
    		std::cout << "beginning test 2" << std::endl;
    		ros::Time start = ros::Time::now();
    		ArmPtr arm;
    		for (int i=0;i<numIter;i++) {
    			Device::currentNoClone()->arms()[0]->cloneInto(arm);
    		}
    		ros::Time finish = ros::Time::now();
    		ros::Duration span = finish-start;
    		std::cout << numIter << " iterations of cloneInto took " << span.toSec() << "s, avg=" << 1000*span.toSec()/numIter << "ms" << std::endl;
    		std::cout << 0.001 / (span.toSec()/numIter) << " per ms" << std::endl;
    	}
    }*/

    // initialize global loop count
    gTime=0;

    // Setup periodic timer
    clock_gettime(CLOCK_REALTIME,&t);   // get current time
    t.tv_sec += 1;                      // start after short delay
    tsnorm(&t);

    log_msg("*** Press pedal to begin homing ***");

    TRACER_ON();

    ///TODO: Break loop when board becomes disconnected.
    //Only run while USB board is attached
    while (ros::ok()) {
        /// SLEEP until next timer shot
        clock_nanosleep(0, TIMER_ABSTIME, &t, NULL);
        gTime++;
        LoopNumber::incrementMain();
        int loopNumber = LoopNumber::get();

        if (RunLevel::hasHomed()) {
        	LOOP_NUMBER_ONCE(__FILE__,__LINE__) {
        		printf("****** Newly homed! [%i]\n",loopNumber);
        		TRACER_ON();
        	}
        }

        TimingInfo t_info;
        t_info.mark_overall_start();

        t_info.mark_usb_read_start();
        //Get and Process USB Packets
        getUSBPackets(&device0); //disable usb for parport test
        t_info.mark_usb_read_end();


        t_info.mark_state_machine_start();
        //Run Safety State Machine
        stateMachine(&device0, &currParams, &rcvdParams);
        t_info.mark_state_machine_end();

        TRACER_OFF();

        t_info.mark_update_state_start();
        //Update Atmel Input Pins

        updateAtmelInputs(device0, currParams.runlevel);
        //Get state updates from master
        if ( checkLocalUpdates() == TRUE)
            updateDeviceState(&currParams, getRcvdParams(&rcvdParams), &device0);
        else
            rcvdParams.runlevel = currParams.runlevel;
        t_info.mark_update_state_end();

        //Clear DAC Values (set current_cmd to zero on all joints)
        clearDACs(&device0);

        t_info.mark_control_start();
        // Calculate Raven control
        if (!RunLevel::hasHomed()) {
        	controlRaven(&device0, &currParams);
        } else {
        	controlRaven(&device0, &currParams);
        	bool scope_tracer_on = false;
        	if (RunLevel::newlyHomed()) {
        		printf("***Newly homed! [%i]\n",loopNumber);
        		scope_tracer_on = true;
        	}
        	if (LoopNumber::onlyEvery("tracer control",5,1000)) {
        		//scope_tracer_on = true;
        	}
        	TRACER_ON_IN_SCOPE_IF(scope_tracer_on);
        	int ctrl_ret = Controller::executeInProcessControl();
#ifdef TEST_NEW_CTRL
        	OldControlInputPtr ptr = ControlInput::oldControlInputUpdateBegin();
        	FOREACH_ARM_IN_DEVICE(arm,Device::currentNoClone()) {
        		ptr->armById(arm->id()).pose() = arm->pose();
        	}
        	ControlInput::oldControlInputUpdateEnd();

        	int ctrl_ret = Controller::executeInProcessControl();

        	DevicePtr ctrl_output = Controller::getControlOutput();

        	struct DOF *_joint = NULL;
        	struct mechanism* _mech = NULL;
        	int i=0,j=0;

        	std::stringstream ss;

        	ss << "tau diff: " << std::endl;
        	while (loop_over_joints(&device0, _mech, _joint, i,j) ) {
        		int joint_ind = j;
        		if (joint_ind == 3) {
        			continue;
        		} else if (joint_ind > 3) {
        			joint_ind--;
        		}
        		MotorPtr m = ctrl_output->arm(i)->motor(joint_ind);
        		float old_torque = _joint->tau_d;
        		float new_torque = m->torque();
        		ss << "  " << i << " " << joint_ind << ": " << old_torque-new_torque << " (" << old_torque << "," << new_torque << ")" << std::endl;
        	}
        	ss << std::endl;
        	log_msg_throttle(0.25,"%s",ss.str().c_str());
#endif
        }
        //////////////// END SURGICAL ROBOT CODE ///////////////////////////

        // Check for overcurrent and impose safe torque limits
        if (overdriveDetect(&device0,currParams.runlevel)) {
            log_warn("Setting soft e stop");
#ifdef USE_NEW_RUNLEVEL
            RunLevel::eStop();
#else
        	soft_estopped = TRUE;
#endif
        }

        t_info.mark_control_end();

        t_info.mark_usb_write_start();
        //Update Atmel Output Pins
        updateAtmelOutputs(&device0, currParams.runlevel);

        //Fill USB Packet and send it out
        putUSBPackets(&device0); //disable usb for par port test
        t_info.mark_usb_write_end();

        t_info.mark_ros_start();
        //Publish current raven state
        publish_ros(&device0,currParams);   // from local_io

        ros::spinOnce();
        t_info.mark_ros_end();

        t.tv_nsec+=interval; //Update timer count for next clock interrupt
        tsnorm(&t);

        t_info.mark_overall_end();

        TimingInfo::mark_loop_end();
        TRACER_OFF();

        /*
        static bool test_done = false;
        if (!test_done &&  TimingInfo::NUM_LOOPS_ALL >= 50) {
        	History<Device>::Type hist = Device::HISTORY;
        	printf("ptrs equal? %p %p %i\n",hist.front().value.get(),Device::HISTORY.front().value.get(),hist.front().value.get()==Device::HISTORY.front().value.get());
        	if (hist.front().value->timestamp() == hist.back().value->timestamp()) {
        		printf("Timestamps equal!\n");
        	} else {
        		hist.push_front(CloningWrapper<Device>(Device::HISTORY.back().value));
        		printf("timestamps equal? %i\n",hist.front().value->timestamp() == Device::HISTORY.front().value->timestamp());
        	}
        	test_done = true;
        }
        */

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
	log_msg("Initializing ROS...");
    ros::init(argc, argv, "r2_control");
    ros::NodeHandle n;
//    rosrt::init();
    init_ravengains(n, &device0);
    saveDOFInfo();
    init_ros_topics(n,&device0);

    return 0;
}

int main(int argc, char **argv)
{
//	printf("num args %i\n",argc);
//	for (int i=0;i<argc;i++) {
//		printf("%s\n",argv[i]);
//	}
	omni_to_ros = false;
	if (argc > 1) {
		if (strcmp(argv[1],"gold")==0) {
			printf("**** GOLD ARM ONLY ****\n");
			disable_arm_id[GREEN_ARM_ID] = true;
		} else if (strcmp(argv[1],"green")==0) {
			printf("**** GREEN ARM ONLY ****\n");
			disable_arm_id[GOLD_ARM_ID] = true;
		} else if (strcmp(argv[1],"both")==0) {
			printf("**** BOTH ARMS ****\n");
			disable_arm_id[GOLD_ARM_ID] = false;
			disable_arm_id[GREEN_ARM_ID] = false;
		}
		if (strcmp(argv[1],"--omni-to-ros")==0 || (argc > 2 && strcmp(argv[2],"--omni-to-ros")==0)) {
			printf("Omni to ros\n");
			omni_to_ros = true;
		}
	}
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
    //pthread_create(&control_thread, NULL, control_process, NULL);
    pthread_create(&rt_thread, NULL, rt_process, NULL); //Start the   thread
    pthread_join(rt_thread,NULL); //Suspend main until rt thread terminates

    log_msg("\n\n\nI'm shutting down now... Please close the USB!\n\n\n");
    usleep(1e6); //Sleep for 1 second

    exit(0);
}
