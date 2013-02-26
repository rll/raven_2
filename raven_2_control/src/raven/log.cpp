
/**
Generic logging function

**/
#include <stdio.h>
#include <stdarg.h>
#include <ros/console.h>
#include <map>
#include "log.h"
#include <boost/thread/mutex.hpp>

static float defaultInterval = 1;
static std::map<std::string,float> intervals;
static std::map<std::string,float> last_hits;
static std::map<std::string,bool> print;


//printf("%s\n",buf);

#define SET_BUF \
	static char buf[1024]; \
	va_list args; \
	va_start (args, fmt); \
	vsprintf(buf,fmt,args); \
	va_end(args);

int log_msg(const char* fmt,...)
{
	SET_BUF
	ROS_INFO("%s",buf);
    return 0;
}

int log_warn(const char* fmt,...)
{
    SET_BUF
    ROS_WARN("%s",buf);
    return 0;
}

int log_err(const char* fmt,...)
{
	SET_BUF
    ROS_ERROR("%s",buf);
    return 0;
}

int log_msg_throttle(float interval,const char* fmt,...) {
	static double last_hit = 0.0;
	::ros::Time now = ::ros::Time::now();
	if (ROS_UNLIKELY(last_hit + interval <= now.toSec())) {
	  last_hit = now.toSec();
	  SET_BUF
	  ROS_INFO("%s",buf);
	  return 0;
	}
	return 1;
}

int log_warn_throttle(float interval,const char* fmt,...) {
	static double last_hit = 0.0;
	::ros::Time now = ::ros::Time::now();
	if (ROS_UNLIKELY(last_hit + interval <= now.toSec())) {
	  last_hit = now.toSec();
	  SET_BUF
	  ROS_WARN("%s",buf);
	  return 0;
	}
	return 1;
}

int log_err_throttle(float interval,const char* fmt,...) {
	static double last_hit = 0.0;
	::ros::Time now = ::ros::Time::now();
	if (ROS_UNLIKELY(last_hit + interval <= now.toSec())) {
	  last_hit = now.toSec();
	  SET_BUF
	  ROS_ERROR("%s",buf);
	  return 0;
	}
	return 1;
}

int err_msg(const char* fmt,...)
{
    SET_BUF
    ROS_ERROR("%s",buf);
    return 0;
}

void set_throttle_default_interval(float interval) {
	defaultInterval = interval;
}

void set_throttle_interval(const char* name,float interval) {
	std::string nameStr(name);
	intervals[nameStr] = interval;
}

void update_log_throttler() {
	std::map<std::string,float>::iterator itr;
	::ros::Time now = ::ros::Time::now();
	for (itr=intervals.begin();itr!=intervals.end();itr++) {
		float last_hit = last_hits[itr->first];
		if (ROS_UNLIKELY(last_hit + itr->second <= now.toSec())) {
			last_hits[itr->first] = now.toSec();
			print[itr->first] = true;
		} else {
			print[itr->first] = false;
		}
	}
}

int log_msg_throttle(const char* name,const char* fmt, ...) {
	if (print[std::string(name)]) {
		SET_BUF
		ROS_INFO("%s",buf);
		return 0;
	}
	return 1;
}

const std::string Tracer::WHITESPACE("                                                                                                                                                                                 ");
boost::thread_specific_ptr<std::string> Tracer::NAME;
boost::thread_specific_ptr<int> Tracer::LEVEL;
boost::thread_specific_ptr<bool> Tracer::ON;
boost::thread_specific_ptr<time_t> Tracer::TIMESTAMP;

Tracer::ScopedTracer::ScopedTracer() {
	Tracer::enter("");
}

Tracer::ScopedTracer::ScopedTracer(const char* fmt,...) {
	SET_BUF
	Tracer::enter(buf);
}

Tracer::ScopedTracer::ScopedTracer(const std::type_info& type, const char* fmt,...) {
	if (strlen(fmt) == 0) {
		Tracer::enter("");
	} else {
		SET_BUF
		Tracer::enter("%s::%s",type.name(),buf);
	}
}

Tracer::ScopedTracer::ScopedTracer(void* p,const std::type_info& type, const char* fmt,...) {
	if (strlen(fmt) == 0) {
		Tracer::enter("");
	} else {
		SET_BUF
		Tracer::enter("%s@%p::%s",type.name(),p,buf);
	}
}


Tracer::ScopedTracer::~ScopedTracer() {
	Tracer::leave();
}

std::string
Tracer::getThreadName() {
	if (ROS_UNLIKELY(!NAME.get())) {
		NAME.reset(new std::string(""));
	}
	return *NAME;
}

void
Tracer::setThreadName(const std::string& name) {
	if (ROS_LIKELY(!NAME.get())) {
		NAME.reset(new std::string(name));
	} else {
		*NAME = name;
	}
}

bool Tracer::isOn() {
	if (ROS_UNLIKELY(!ON.get())) {
		ON.reset(new bool(false));
	}
	return *ON;
}

bool Tracer::set(bool on) {
	if (ROS_UNLIKELY(!ON.get())) {
		ON.reset(new bool(on));
		return false;
	} else {
		bool prev = *ON;
		*ON = on;
		return prev;
	}
}

bool Tracer::on() {
	if (ROS_UNLIKELY(!ON.get())) {
		ON.reset(new bool(true));
		return false;
	} else {
		bool prev = *ON;
		*ON = true;
		return prev;
	}
}
bool Tracer::off() {
	if (ROS_UNLIKELY(!ON.get())) {
		ON.reset(new bool(false));
		return false;
	} else {
		bool prev = *ON;
		*ON = false;
		return prev;
	}
}

int Tracer::level() {
	return *LEVEL;
}

int
Tracer::enter(const char* fmt,...) {
	SET_BUF
	int currLevel = printf(buf);
	if (currLevel >= 0) {
		*LEVEL = currLevel + 1;
	}
	return currLevel;
}

void
Tracer::leave() {
	if (ROS_UNLIKELY(!ON.get())) {
		ON.reset(new bool(false));
		return;
	}
	if (ROS_LIKELY(!*ON)) {
		return;
	}
	(*LEVEL) -= 1;
}

//#define PRINT_CURRENT_LEVEL

int
Tracer::printf(const char* fmt,...) {
	if (ROS_UNLIKELY(!ON.get())) {
		ON.reset(new bool(false));
		return -1;
	}
	if (ROS_LIKELY(!*ON)) {
		return -1;
	}
	int currLevel = 0;
	if (ROS_UNLIKELY(!LEVEL.get())) {
		LEVEL.reset(new int(0));
	} else {
		currLevel = *LEVEL;
	}
	std::string thread = getThreadName();
	if (strlen(fmt) != 0) {
		SET_BUF
		std::string whitespace = WHITESPACE.substr(0,currLevel*TRACER_INDENT_PER_LEVEL);
		if (thread.empty()) {
			std::cout
			<< whitespace
#ifdef PRINT_CURRENT_LEVEL
			<< currLevel
			<< ":"
#endif
			<< buf
			<< std::endl;
		} else {
			std::cout
			<< whitespace
			<< "[" << thread
#ifdef PRINT_CURRENT_LEVEL
			<< ":" << currLevel
#endif
			<< "]"
			<< buf
#ifdef PRINT_CURRENT_LEVEL
			<< " " << currLevel
#endif
			<< std::endl;
		}
	}
	return currLevel;
}

int
Tracer::printf(const std::type_info& type, const char* fmt,...) {
	if (strlen(fmt)==0) {
		return Tracer::printf("");
	} else {
		SET_BUF
		return Tracer::printf("%s::%s",type.name(),buf);
	}
}

int
Tracer::printf(void* p, const std::type_info& type, const char* fmt,...) {
	if (strlen(fmt)==0) {
		return Tracer::printf("");
	} else {
		SET_BUF
		return Tracer::printf("%s@%p::%s",type.name(),p,buf);
	}
}

time_t
Tracer::timestamp() {
	if (ROS_UNLIKELY(!TIMESTAMP.get())) {
		TIMESTAMP.reset(new time_t(0));
	}
	return *TIMESTAMP;
}
