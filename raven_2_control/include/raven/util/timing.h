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

#include <ros/ros.h>
#include <boost/circular_buffer.hpp>

#include <string>
#include <iomanip>
#include <sstream>
#include <memory>

#include <raven/state/runlevel.h>

#define TIMING_FIELD_STRING_WIDTH 14

#define TIMING_STATS(StructName,name) StructName::name##_stats()

#define FIELD_TO_STREAM(name,type) STRINGIFY(type) ": " << std::setw(7) << name##_##type.toNSec() << std::setw(0) << " (" << std::setw(7) << name##_##type##_all.toNSec() << std::setw(0) << ")"

#define TIMING_STRUCT_SETUP_HEADER(StructName) \
	private: \
	static bool RESET; \
	public: \
	typedef boost::circular_buffer<StructName> History; \
	/*typedef std::auto_ptr<StructName> Ptr;*/ \
	static void clear(StructName& s) { \
		StructName newStruct; \
		s = newStruct; \
	} \
	static int NUM_LOOPS; \
	static int NUM_LOOPS_ALL; \
	static void reset() { RESET = true; } \
	static void mark_loop_end() { \
		if (RESET) { \
			NUM_LOOPS = 0; \
		} else { \
			NUM_LOOPS++; \
		} \
		NUM_LOOPS_ALL++; \
		RESET = false; \
	}

#define TIMING_STRUCT_FIELD_HEADER(name) \
	private:\
	ros::Duration name##_; \
	public:\
	inline ros::Duration name() const { \
		return name##_; \
	} \
	inline static std::string name##_str() { \
		return STRINGIFY(name); \
	} \
	inline static std::string name##_str_padded() { \
		std::string nm_str = STRINGIFY(name); \
		if (nm_str.size() > TIMING_FIELD_STRING_WIDTH) { \
			size_t first_half_width = TIMING_FIELD_STRING_WIDTH-TIMING_FIELD_STRING_WIDTH/2; \
			size_t second_half_width = TIMING_FIELD_STRING_WIDTH - first_half_width; \
			nm_str = nm_str.substr(0,first_half_width) + nm_str.substr(nm_str.size()-second_half_width); \
		} else { \
			while (nm_str.size() < TIMING_FIELD_STRING_WIDTH) { \
				nm_str += " "; \
			} \
		} \
		return nm_str; \
	} \
	ros::Time name##_start; \
	ros::Time name##_end; \
	static ros::Duration name##_min; \
	static ros::Duration name##_max; \
	static ros::Duration name##_avg; \
	static ros::Duration name##_min_all; \
	static ros::Duration name##_max_all; \
	static ros::Duration name##_avg_all; \
	inline void mark_##name##_start() { \
		name##_start = ros::Time::now(); \
	}\
	inline void mark_##name##_intermediate() { \
		name##_end = ros::Time::now(); \
		ros::Duration d = name##_end - name##_start; \
		name##_ = name##_ + d; \
	}\
	inline void mark_##name##_intermediate_final() { \
		set_##name##_stats(name##_); \
	}\
	inline void mark_##name##_end() { \
		name##_end = ros::Time::now(); \
		ros::Duration d = name##_end - name##_start; \
		name##_ = name##_ + d; \
		set_##name##_stats(d); \
	}\
	inline void set_##name(ros::Duration d) { \
		name##_ = d; \
		set_##name##_stats(d); \
	} \
	private: \
	inline void set_##name##_stats(ros::Duration d) { \
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
	public: \
	inline static std::string name##_stats() { \
		std::stringstream ss; \
		ss << std::left << std::setw(TIMING_FIELD_STRING_WIDTH) << name##_str_padded() << std::right << std::setw(0) << "\t"; \
		ss << FIELD_TO_STREAM(name,avg); \
		ss << " "; \
		ss << FIELD_TO_STREAM(name,min); \
		ss << " "; \
		ss << FIELD_TO_STREAM(name,max); \
		return ss.str(); \
	}

#define TIMING_STRUCT_SETUP_SOURCE(StructName) \
		int StructName::NUM_LOOPS = 0; \
		int StructName::NUM_LOOPS_ALL = 0; \
		bool StructName::RESET = false;

#define TIMING_STRUCT_FIELD_SOURCE(StructName,name) \
	ros::Duration StructName::name##_avg; \
	ros::Duration StructName::name##_max; \
	ros::Duration StructName::name##_min; \
	ros::Duration StructName::name##_avg_all; \
	ros::Duration StructName::name##_max_all; \
	ros::Duration StructName::name##_min_all;


/* in cpp, put these lines:
#undef TIMING_STRUCT_NAME
#define TIMING_STRUCT_NAME **StructName**
#undef TIMING_STRUCT_SETUP
#define TIMING_STRUCT_SETUP(sn) TIMING_STRUCT_SETUP_SOURCE(TIMING_STRUCT_NAME)
#undef TIMING_STRUCT_FIELD
#define TIMING_STRUCT_FIELD(name) TIMING_STRUCT_FIELD_SOURCE(TIMING_STRUCT_NAME,name)
*/

#define TIMING_STRUCT_FIELD(name) TIMING_STRUCT_FIELD_HEADER(name)

#define TIMING_STRUCT_SETUP(StructName) TIMING_STRUCT_SETUP_HEADER(StructName)

struct TimingInfo;
typedef boost::circular_buffer<TimingInfo> TimingHistory;

struct TimingInfo {
	TIMING_STRUCT_SETUP(TimingInfo)

	TIMING_STRUCT_FIELD(overall)

	TIMING_STRUCT_FIELD(usb_read)
	TIMING_STRUCT_FIELD(state_machine)
	TIMING_STRUCT_FIELD(update_state)
	TIMING_STRUCT_FIELD(control)
	TIMING_STRUCT_FIELD(usb_write)
	TIMING_STRUCT_FIELD(ros)

	TIMING_STRUCT_FIELD(cn_overall)
	TIMING_STRUCT_FIELD(cn_get_input)
	TIMING_STRUCT_FIELD(cn_set_input)
	TIMING_STRUCT_FIELD(cn_copy_device)
	TIMING_STRUCT_FIELD(cn_ctrl_overall)
	TIMING_STRUCT_FIELD(cn_ctrl_begin)
	TIMING_STRUCT_FIELD(cn_apply_ctrl)
	TIMING_STRUCT_FIELD(cn_ctrl_finish)
	TIMING_STRUCT_FIELD(cn_set_output)
};

struct USBTimingInfo {
	TIMING_STRUCT_SETUP(USBTimingInfo)

	TIMING_STRUCT_FIELD(get_packet)
	TIMING_STRUCT_FIELD(process_packet)
};

struct ControlTiming {
	TIMING_STRUCT_SETUP(ControlTiming)

	TIMING_STRUCT_FIELD(overall)
};


#endif /* TIMING_H_ */
