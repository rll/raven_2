#ifndef LOG_H
#define LOG_H

#include <stdio.h>
#include <string>
#include <boost/thread/tss.hpp>


int log_msg(const char* fmt,...);
int log_warn(const char* fmt,...);
int log_err(const char* fmt,...);

int log_msg_throttle(float interval,const char* fmt,...);
int log_warn_throttle(float interval,const char* fmt,...);
int log_err_throttle(float interval,const char* fmt,...);

int err_msg(const char* fmt,...);

class Tracer {
private:
	Tracer() {}
	static const std::string WHITESPACE;
	static boost::thread_specific_ptr<std::string> NAME;
	static boost::thread_specific_ptr<int> LEVEL;
	static boost::thread_specific_ptr<bool> ON;

public:
	static std::string getThreadName();
	static void setThreadName(const std::string& name);
	static void on();
	static void off();
	static int level();
	static int enter(const std::string& name);
	static void leave();
	static int print(const std::string& stuff);
};

#define USE_TRACER

#ifdef USE_TRACER
#define TRACER_GET_THREAD_NAME() Tracer::getThreadName()
#define TRACER_SET_THREAD_NAME(name) Tracer::setThreadName(name)
#define TRACER_ON() Tracer::on()
#define TRACER_OFF() Tracer::off()
#define TRACER_LEVEL() Tracer::level()
#define TRACER_ENTER(name) Tracer::enter(name)
#define TRACER_LEAVE() Tracer::leave()
#define TRACER_PRINT(stuff) Tracer::print(stuff)
#else
#define TRACER_GET_THREAD_NAME() do {} while(0)
#define TRACER_SET_THREAD_NAME(name) do {} while(0)
#define TRACER_ON() do {} while(0)
#define TRACER_OFF() do {} while(0)
#define TRACER_LEVEL() do {} while(0)
#define TRACER_ENTER(name) do {} while(0)
#define TRACER_LEAVE() do {} while(0)
#define TRACER_PRINT(stuff) do {} while(0)
#endif

#endif // LOG_H
