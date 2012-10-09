/*
 * timing.h
 *
 *  Created on: Oct 9, 2012
 *      Author: benk
 */

#ifndef TIMING_H_
#define TIMING_H_

#ifndef STRINGIFY
#define STRINGIFY(s) STRINGIFY_HELPER(s)
#define STRINGIFY_HELPER(s) #s
#endif

#include <string>
#include <iomanip>
#include <sstream>
#include <ros/ros.h>
#include <boost/circular_buffer.hpp>

#define FIELD_TO_STREAM(name,type) STRINGIFY(type) ": " << std::setw(7) << name##_##type.toNSec() << std::setw(0) << " (" << std::setw(7) << name##_##type##_all.toNSec() << std::setw(0) << ")"

#define TIMING_INFO_FIELD(name) \
	ros::Duration name; \
	ros::Time name##_start; \
	ros::Time name##_end; \
	static ros::Duration name##_min; \
	static ros::Duration name##_max; \
	static ros::Duration name##_avg; \
	static ros::Duration name##_min_all; \
	static ros::Duration name##_max_all; \
	static ros::Duration name##_avg_all; \
	void mark_##name##_start() { \
		name##_start = ros::Time::now(); \
	}\
	void mark_##name##_end() { \
		name##_end = ros::Time::now(); \
		set_##name(name##_end - name##_start); \
	}\
	void set_##name(ros::Duration d) { \
		name##_avg = ros::Duration((d.toSec() + NUM_LOOPS * name##_avg.toSec()) / (NUM_LOOPS+1)); \
		name##_avg_all = ros::Duration((d.toSec() + NUM_LOOPS_ALL * name##_avg_all.toSec()) / (NUM_LOOPS_ALL+1)); \
		if (d > name##_max || NUM_LOOPS == 0) { \
			name##_max = d; \
			if (d > name##_max_all || NUM_LOOPS_ALL == 0) { \
				name##_max_all = d; \
			} \
		} \
		if (d < name##_min || NUM_LOOPS == 0) { \
			name##_min = d; \
			if (d < name##_min_all || NUM_LOOPS_ALL == 0) { \
				name##_min_all = d; \
			} \
		} \
	} \
	static std::string name##_stats() { \
		std::stringstream ss; \
		ss << std::left << std::setw(10) << STRINGIFY(name) << std::right << std::setw(0) << "\t"; \
		ss << FIELD_TO_STREAM(name,avg); \
		ss << " "; \
		ss << FIELD_TO_STREAM(name,min); \
		ss << " "; \
		ss << FIELD_TO_STREAM(name,max); \
		return ss.str(); \
	}

struct TimingInfo;
typedef boost::circular_buffer<TimingInfo> TimingHistory;

struct TimingInfo {
	static int NUM_LOOPS;
	static int NUM_LOOPS_ALL;
	static bool RESET;

	static void mark_loop_end(TimingInfo* t_info=0) {
		if (RESET) {
			NUM_LOOPS = 0;
		} else {
			NUM_LOOPS++;
		}
		NUM_LOOPS_ALL++;
		RESET = false;
	}

	TIMING_INFO_FIELD(overall)

	TIMING_INFO_FIELD(usb_read)
	TIMING_INFO_FIELD(state_machine)
	TIMING_INFO_FIELD(update_state)
	TIMING_INFO_FIELD(control)
	TIMING_INFO_FIELD(usb_write)
	TIMING_INFO_FIELD(ros)
};


#endif /* TIMING_H_ */
