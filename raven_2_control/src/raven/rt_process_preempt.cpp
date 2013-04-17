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
#include <raven/util/config.h>

#include <raven/state/initializer.h>

#include <raven/control/controller.h>
#include <raven/control/controllers/motor_position_pid.h>
#include <raven/control/controllers/end_effector_control.h>

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

bool disable_arm_id[2] = {false,false};

pthread_t rt_thread;
pthread_t control_thread;
pthread_t net_thread;
pthread_t fiforcv_thread;
pthread_t fifosend_thread;
pthread_t console_thread;

//Global Variables from globals.c
extern struct DOF_type DOF_types[];

extern USBStruct USBBoards;

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

void _outputTiming();
void _outputLoopTiming(const TimingInfo& t_info);
void _testUSBRead();
void _testClone();

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

    log_msg("Starting RT Process...");

//#define TEST_USB_READ
#ifdef TEST_USB_READ
    _testUSBRead();
#endif

    // Initializations (run here and again in init.cpp)
#ifdef USE_NEW_DEVICE
    DeviceInitializer initr;
    initr.initializeDeviceInstance();
#endif
    initDOFs(&device0);

#ifdef USE_NEW_CONTROLLER
    log_msg("Creating controllers");
#ifdef TEST_NEW_CONTROLLER
    ControllerPtr pid(new MotorPositionPID());
	ControllerPtr ee(new EndEffectorController());
#endif

	log_msg("Registering controllers");

#ifdef TEST_NEW_CONTROLLER
    Controller::registerController("motor/position",pid);
    Controller::registerController("end_effector/pose",ee);
    Controller::registerController("end_effector/grasp+pose",ee);
#endif

    Controller::registerController(Arm::ALL_ARMS,""); //init hold position controller if none exists

    log_msg("Setting controller");

#ifdef TEST_NEW_CONTROLLER
    Controller::setController("end_effector/pose");
#endif
#endif

    //testClone();

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
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &t, NULL);
        gTime++;
        LoopNumber::incrementMain();
        int loopNumber = LoopNumber::get();

        struct timespec start;
        struct timespec end;
        clock_gettime(CLOCK_REALTIME,&start);

        if (RunLevel::hasHomed()) {
        	LOOP_NUMBER_ONCE(__FILE__,__LINE__) {
        		//printf("****** Newly homed! [%i]\n",loopNumber);
        		TRACER_ON();
        	}
        }

        TimingInfo t_info;
        t_info.mark_overall_start();

#if defined USE_NEW_DEVICE && false
        LOOP_NUMBER_ONCE(__FILE__,__LINE__) {
        	Device::DEBUG_OUTPUT_TIMING = true;
        }
        if (!RunLevel::hasHomed() && LoopNumber::onlyEvery("output usb device finish update timing",300,1000)) {
        	printf("Outputting usb timing [%i]\n",loopNumber);
        	Device::DEBUG_OUTPUT_TIMING = true;
        }
        if (RunLevel::hasHomed() && LoopNumber::onlyEvery("output usb after homing device finish update timing",4,5000)) {
        	printf("Outputting usb timing [%i]\n",loopNumber);
        	Device::DEBUG_OUTPUT_TIMING = true;
        }
#endif

        t_info.mark_usb_read_start();
        //Get and Process USB Packets
        getUSBPackets(&device0); //disable usb for parport test
        t_info.mark_usb_read_end();

#ifdef USE_NEW_DEVICE
        if (Device::DEBUG_OUTPUT_TIMING) {
        	_outputTiming();
        }

        Device::DEBUG_OUTPUT_TIMING = false;
#endif


        t_info.mark_state_machine_start();
        //Run Safety State Machine
        stateMachine(&device0, &currParams, &rcvdParams);
        t_info.mark_state_machine_end();

        TRACER_OFF();

        t_info.mark_update_state_start();
        //Update Atmel Input Pins

        updateAtmelInputs(device0, currParams.runlevel);
        //Get state updates from master
        if ( getRcvdParams(&rcvdParams))
            updateDeviceState(&currParams, &rcvdParams, &device0);
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
#ifdef USE_NEW_CONTROLLER
        	/*
        	bool scope_tracer_on = false;
        	if (RunLevel::newlyHomed()) {
        		printf("***Newly homed! [%i]\n",loopNumber);
        		scope_tracer_on = true;
        	}
        	if (LoopNumber::onlyEvery("tracer control",5,1000)) {
        		//scope_tracer_on = true;
        	}
        	TRACER_ON_IN_SCOPE_IF(scope_tracer_on);
        	*/
        	if (RunLevel::newlyHomed() || LoopNumber::onlyEvery("output device finish update timing",4,5000)) {
        		Device::DEBUG_OUTPUT_TIMING = true;
        		if (Device::DEBUG_OUTPUT_TIMING) printf("Outputting controller timing [%i]\n",loopNumber);

        	}
        	int ctrl_ret = Controller::executeInProcessControl();
        	if (Device::DEBUG_OUTPUT_TIMING) {
        		outputTiming();
        	}
        	Device::DEBUG_OUTPUT_TIMING = false;
        	LOOP_NUMBER_ONCE(__FILE__,__LINE__) {
        		printf("HAS HOMED %i\n",LoopNumber::get());
        	}
#ifdef TEST_NEW_CONTROLLER
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
#endif
        }
        //////////////// END SURGICAL ROBOT CODE ///////////////////////////

        // Check for overcurrent and impose safe torque limits
        if (overdriveDetect(&device0,currParams.runlevel)) {
            log_warn("Setting soft e stop");
            RunLevel::eStop();
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

        t_info.mark_overall_end();

        clock_gettime(CLOCK_REALTIME,&end);

        int64_t start_ns = start.tv_sec * (int64_t)1000000000 + start.tv_nsec;
        int64_t end_ns = end.tv_sec * (int64_t)1000000000 + end.tv_nsec;

        bool over_time = (end_ns-start_ns) > 1000000;
        //bool over_time = t_info.overall() > ros::Duration(1./1000);
        if (over_time) {
        	TimingInfo::NUM_OVER_TIME += 1;
        }
        TimingInfo::PCT_OVER_TIME = ((float)TimingInfo::NUM_OVER_TIME) / loopNumber;

        //clock interrupt on 1ms boundaries
        int64_t t_ns;
        do {
        	t.tv_nsec+=interval; //Update timer count for next clock interrupt
        	t_ns = t.tv_sec * (int64_t)1000000000 + t.tv_nsec;
        } while (false && t_ns <= end_ns);
        tsnorm(&t);

        TimingInfo::mark_loop_end();
        TRACER_OFF();

        //_outputLoopTiming(t_info);

        //Done for this cycle
    }


    log_msg("Raven Control is shutdown");
    return 0;
}

/**Initialize by initializing usb boards, etc*/
int init_module(void)
{
    log_msg("Initializing USB I/O...");

    //Initialize USB Board
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

int init_ros(int& argc, char**& argv)
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

    rosx::Parser parser;
	parser.addGroup(Config::Options);
	parser.addArg<string>("arm");
	parser.read(argc, argv);

	omni_to_ros = false;
	if (parser.hasArg("arm")) {

		std::string arm = parser.getArg<std::string>("arm");
		if (arm == "gold") {
			printf("**** GOLD ARM ONLY ****\n");
			disable_arm_id[GREEN_ARM_ID] = true;
		} else if (arm == "green") {
			printf("**** GREEN ARM ONLY ****\n");
			disable_arm_id[GOLD_ARM_ID] = true;
		} else if (arm == "both") {
			printf("**** BOTH ARMS ****\n");
			disable_arm_id[GOLD_ARM_ID] = false;
			disable_arm_id[GREEN_ARM_ID] = false;
		}
		/*
		if (strcmp(argv[1],"--omni-to-ros")==0 || (argc > 2 && strcmp(argv[2],"--omni-to-ros")==0)) {
			printf("Omni to ros\n");
			omni_to_ros = true;
		}
		*/
	}

    pthread_create(&net_thread, NULL, network_process, NULL); //Start the network thread
//    pthread_create(&fiforcv_thread, NULL, data_fifo_rcv_process, NULL); //Start the    thread
//    pthread_create(&fifosend_thread, NULL, data_fifo_send_process, NULL); //Start the   thread
    pthread_create(&console_thread, NULL, console_process, NULL); //Start the     thread
    //pthread_create(&control_thread, NULL, control_process, NULL);
    pthread_create(&rt_thread, NULL, rt_process, NULL); //Start the   thread

    //ros::spin();

    pthread_join(rt_thread,NULL); //Suspend main until rt thread terminates

    log_msg("\n\n\nI'm shutting down now... Please close the USB!\n\n\n");
    usleep(1e6); //Sleep for 1 second

    exit(0);
}

/********************* Utility functions *************************/

void _outputTiming() {
	printf("arm_gold:\t\t\t%7lli\n",(long long int)TempTiming::arm_gold.toNSec());
	printf("\tarm_smf_gold:\t\t%7lli\n",(long long int)TempTiming::arm_smf_gold.toNSec());
	printf("\t\tnmf_iau:\t%7lli\n",(long long int)TempTiming::s_nmf_iau_gold.toNSec());
	printf("\t\tmf_iau:\t\t%7lli\n",(long long int)TempTiming::s_mf_iau_gold.toNSec());
	printf("\t\tmf_mu:\t\t%7lli\n",(long long int)TempTiming::s_mf_mu_gold.toNSec());
	printf("\t\tmf_mu_avg:\t%7lli\n",(long long int)TempTiming::s_mf_mu_avg_gold.toNSec());
	printf("\t\tmf_au:\t\t%7lli\n",(long long int)TempTiming::s_mf_au_gold.toNSec());

	printf("\tarm_cmf_gold:\t\t%7lli\n",(long long int)TempTiming::arm_cmf_gold.toNSec());
	printf("\t\tnmf_iau:\t%7lli\n",(long long int)TempTiming::c_nmf_iau_gold.toNSec());
	printf("\t\tmf_iau:\t\t%7lli\n",(long long int)TempTiming::c_mf_iau_gold.toNSec());
	printf("\t\tmf_mu:\t\t%7lli\n",(long long int)TempTiming::c_mf_mu_gold.toNSec());
	printf("\t\tmf_mu_avg:\t%7lli\n",(long long int)TempTiming::c_mf_mu_avg_gold.toNSec());
	printf("\t\tmf_au:\t\t%7lli\n",(long long int)TempTiming::c_mf_au_gold.toNSec());

	printf("\tarm_hue_gold:\t\t%7lli\n",(long long int)TempTiming::arm_hue_gold.toNSec());

	printf("arm_green:\t\t\t%7lli\n",(long long int)TempTiming::arm_green.toNSec());
	printf("\tarm_smf_green:\t\t%7lli\n",(long long int)TempTiming::arm_smf_green.toNSec());
	printf("\t\tnmf_iau:\t%7lli\n",(long long int)TempTiming::s_nmf_iau_green.toNSec());
	printf("\t\tmf_iau:\t\t%7lli\n",(long long int)TempTiming::s_mf_iau_green.toNSec());
	printf("\t\tmf_mu:\t\t%7lli\n",(long long int)TempTiming::s_mf_mu_green.toNSec());
	printf("\t\tmf_mu_avg:\t%7lli\n",(long long int)TempTiming::s_mf_mu_avg_green.toNSec());
	printf("\t\tmf_au:\t\t%7lli\n",(long long int)TempTiming::s_mf_au_green.toNSec());

	printf("\tarm_cmf_green:\t\t%7lli\n",(long long int)TempTiming::arm_cmf_green.toNSec());
	printf("\t\tnmf_iau:\t%7lli\n",(long long int)TempTiming::c_nmf_iau_green.toNSec());
	printf("\t\tmf_iau:\t\t%7lli\n",(long long int)TempTiming::c_mf_iau_green.toNSec());
	printf("\t\tmf_mu:\t\t%7lli\n",(long long int)TempTiming::c_mf_mu_green.toNSec());
	printf("\t\tmf_mu_avg:\t%7lli\n",(long long int)TempTiming::c_mf_mu_avg_green.toNSec());
	printf("\t\tmf_au:\t\t%7lli\n",(long long int)TempTiming::c_mf_au_green.toNSec());

	printf("\tarm_hue_green:\t\t%7lli\n",(long long int)TempTiming::arm_hue_green.toNSec());

	printf("dev_ifu:\t\t\t%7lli\n",(long long int)TempTiming::dev_ifu.toNSec());
}

void _testUSBRead() {
	log_warn("Testing USB board reading");

	std::vector<int> intervals;
	intervals.push_back(250 * 1000);
	intervals.push_back(500 * 1000);
	intervals.push_back(750 * 1000);
	intervals.push_back(1000 * 1000);
	intervals.push_back(1250 * 1000);
	intervals.push_back(1500 * 1000);
	intervals.push_back(1750 * 1000);
	intervals.push_back(2000 * 1000);

	std::vector<int64_t> avgs(intervals.size());
	std::vector<int64_t> mins(intervals.size());
	std::vector<int64_t> maxs(intervals.size());

	for (size_t intervalIdx=0;intervalIdx<intervals.size();intervalIdx++) {
		unsigned char buffer[512];
		struct timespec testBegin;
		struct timespec testEnd;
		struct timespec singleTestBegin;
		struct timespec singleTestEnd;

		//int interval = 1500 * 1000;
		int interval = intervals[intervalIdx];

		int boardId = USBBoards.boards[0];
		std::cout << "Testing board " << boardId << " with interval " << interval << std::endl;
		double cumulativeTime_s = 0;
		int64_t cumulativeTime_ns = 0;
		int64_t minTime_ns = 10 * (int64_t)NSEC_PER_SEC;
		int64_t maxTime_ns = 0;
		int numTests = 500;

		std::vector<int64_t> times(numTests,0);

		clock_gettime(CLOCK_REALTIME,&testBegin);
		struct timespec sleepUntil = singleTestBegin;
		sleepUntil.tv_sec += 1;
		clock_nanosleep(0, TIMER_ABSTIME, &sleepUntil, NULL);
		clock_gettime(CLOCK_REALTIME,&testBegin);
		for (int i = 0;i<numTests;i++) {
			clock_gettime(CLOCK_REALTIME,&singleTestBegin);
			usb_read(boardId,buffer,27);
			clock_gettime(CLOCK_REALTIME,&singleTestEnd);
			int64_t t1 = ((int64_t)singleTestBegin.tv_sec) * NSEC_PER_SEC + (int64_t)singleTestBegin.tv_nsec;
			int64_t t2 = ((int64_t)singleTestEnd.tv_sec) * NSEC_PER_SEC + (int64_t)singleTestEnd.tv_nsec;
			double d_s = ((double)t2-t1) / NSEC_PER_SEC;
			cumulativeTime_s += d_s;

			int64_t d = (t2-t1);
			cumulativeTime_ns += d;

			times[i] = d;

			if (d < minTime_ns) {
				minTime_ns = d;
			}
			if (d > maxTime_ns) {
				maxTime_ns = d;
			}

			write_zeros_to_board(boardId);

			//std::cout << i << std::endl;
			if (interval) {
				struct timespec sleepUntil = singleTestBegin;
				sleepUntil.tv_nsec += interval;
				clock_nanosleep(0, TIMER_ABSTIME, &sleepUntil, NULL);
			}
		}
		clock_gettime(CLOCK_REALTIME,&testEnd);

		double avg_s = cumulativeTime_s / numTests;
		int64_t avg_ns = (int64_t) (((double)cumulativeTime_ns) / numTests);

		int64_t t1 = ((int64_t)testBegin.tv_sec) * NSEC_PER_SEC + (int64_t)testBegin.tv_nsec;
		int64_t t2 = ((int64_t)testEnd.tv_sec) * NSEC_PER_SEC + (int64_t)testEnd.tv_nsec;
		double totalTime_s = ((double)t2-t1) / NSEC_PER_SEC;
		int64_t totalTime_ns = t2-t1;

		avgs[intervalIdx] = avg_ns;
		mins[intervalIdx] = minTime_ns;
		maxs[intervalIdx] = maxTime_ns;

		for (int i = 0;i<numTests;i++) {
			printf("%8lli\n",(long long int)times[i]);
		}

		std::cout << "Total time: " << totalTime_s << std::endl;
		std::cout << "Average: " << avg_s << " (" << avg_ns << " ns)" << std::endl;
		std::cout << "Min: " << minTime_ns << " max: " << maxTime_ns << std::endl;
		//printf("%5.3f: %8lli / %8lli / %8lli\n",((double)interval)/NSEC_PER_SEC,(long long int)avg_ns,(long long int)minTime_ns,(long long int)maxTime_ns);
	}

	for (size_t i=0;i<intervals.size();i++) {
		printf("%8i: %8lli / %8lli / %8lli\n",intervals[i],(long long int)avgs[i],(long long int)mins[i],(long long int)maxs[i]);
	}
	//ros::shutdown();
}

void _testClone() {
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
}

void _outputLoopTiming(const TimingInfo& t_info) {
	bool printTiming =
			LoopNumber::only("rt_proc print timing",2)
	|| RunLevel::newlyHomed()
	|| (TimingInfo::cn_overall_max_all() > ros::Duration(0.001) && LoopNumber::once("ctrl max'd"))
	|| t_info.cn_overall() > ros::Duration(0.001);

	if (printTiming) {
		cout << "TIMING FOR LOOP " << LoopNumber::get() << endl;

		cout << TimingInfo::usb_read_str_padded() << ":\t" << t_info.usb_read().toNSec() << endl;
		cout << TimingInfo::state_machine_str_padded() << ":\t" << t_info.state_machine().toNSec() << endl;
		cout << TimingInfo::update_state_str_padded() << ":\t" << t_info.update_state().toNSec() << endl;
		cout << TimingInfo::cn_get_input_str_padded() << ":\t" << t_info.cn_get_input().toNSec() << endl;

		cout << endl;

		cout << TimingInfo::cn_get_input_str_padded() << ":\t" << t_info.cn_get_input().toNSec() << endl;
		cout << TimingInfo::cn_set_input_str_padded() << ":\t" << t_info.cn_set_input().toNSec() << endl;
		cout << TimingInfo::cn_copy_device_str_padded() << ":\t" << t_info.cn_copy_device().toNSec() << endl;

		cout << TimingInfo::cn_ctrl_begin_str_padded() << ":\t" << t_info.cn_ctrl_begin().toNSec() << endl;
		cout << TimingInfo::cn_apply_ctrl_str_padded() << ":\t" << t_info.cn_apply_ctrl().toNSec() << endl;
		cout << TimingInfo::cn_ctrl_finish_str_padded() << ":\t" << t_info.cn_ctrl_finish().toNSec() << endl;
		cout << TimingInfo::cn_ctrl_overall_str_padded() << ":\t" << t_info.cn_ctrl_overall().toNSec() << endl;

		cout << TimingInfo::cn_set_output_str_padded() << ":\t" << t_info.cn_set_output().toNSec() << endl;

		cout << TimingInfo::cn_overall_str_padded() << ":\t" << t_info.cn_overall().toNSec() << endl;

		cout << endl;

		cout << TimingInfo::control_str_padded() << ":\t" << t_info.control().toNSec() << endl;

		cout << TimingInfo::usb_write_str_padded() << ":\t" << t_info.usb_write().toNSec() << endl;
		cout << TimingInfo::ros_str_padded() << ":\t" << t_info.ros().toNSec() << endl;

		cout << endl;

		cout << TimingInfo::overall_str_padded() << ":\t" << t_info.overall().toNSec() << endl;

		cout << endl;

		//			cout << TIMING_STATS(TimingInfo,cn_get_input) << endl;
		//			cout << TIMING_STATS(TimingInfo,cn_set_input) << endl;
		//			cout << TIMING_STATS(TimingInfo,cn_copy_device) << endl;
		//
		//			cout << TIMING_STATS(TimingInfo,cn_ctrl_begin) << endl;
		//			cout << TIMING_STATS(TimingInfo,cn_apply_ctrl) << endl;
		//			cout << TIMING_STATS(TimingInfo,cn_ctrl_finish) << endl;
		//			cout << TIMING_STATS(TimingInfo,cn_ctrl_overall) << endl;
		//
		//			cout << TIMING_STATS(TimingInfo,cn_set_output) << endl;
		//
		//			cout << TIMING_STATS(TimingInfo,cn_overall) << endl;
		//
		//			cout << endl;
		//
		//			cout << TIMING_STATS(USBTimingInfo,get_packet) << endl;
		//			cout << TIMING_STATS(USBTimingInfo,process_packet) << endl;
		//
		//			cout << endl;
		//
		//			cout << TIMING_STATS(ControlTiming,overall) << endl;
		//
		//			cout << TIMING_STATS(TimingInfo,overall) << endl;
		//
		//			cout << endl;
	}
}
