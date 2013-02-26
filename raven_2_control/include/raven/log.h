#ifndef LOG_H
#define LOG_H

#include <stdio.h>
#include <string>
#include <boost/thread/tss.hpp>
#include <typeinfo>
#include <ctime>

int log_msg(const char* fmt,...);
int log_warn(const char* fmt,...);
int log_err(const char* fmt,...);

int log_msg_throttle(float interval,const char* fmt,...);
int log_warn_throttle(float interval,const char* fmt,...);
int log_err_throttle(float interval,const char* fmt,...);

int err_msg(const char* fmt,...);


//#define USE_TRACER
#define USE_TRACER_VERBOSE


#ifndef TRACER_INDENT_PER_LEVEL
#define TRACER_INDENT_PER_LEVEL 2
#endif

class Tracer {
private:
	Tracer() {}
	static const std::string WHITESPACE;
	static boost::thread_specific_ptr<std::string> NAME;
	static boost::thread_specific_ptr<int> LEVEL;
	static boost::thread_specific_ptr<bool> ON;
	static boost::thread_specific_ptr<time_t> TIMESTAMP;
public:
	struct ScopedTracer {
		ScopedTracer();
		ScopedTracer(const char* fmt,...);
		ScopedTracer(const std::type_info& type, const char* fmt,...);
		ScopedTracer(void* p,const std::type_info& type, const char* fmt,...);
		~ScopedTracer();
	};
	struct OnInScope {
		time_t start;
		bool prev;
		OnInScope(bool on = true,bool keep = true) {
			start = time(0);
			if (on || !keep) {
				prev = Tracer::set(on);
			} else {
				prev = Tracer::isOn();
			}
		}
		~OnInScope() {
			if (Tracer::timestamp() < start) {
				Tracer::set(prev);
			}
		}
	};

	static std::string getThreadName();
	static void setThreadName(const std::string& name);
	static bool isOn();
	static bool set(bool on);
	static bool on();
	static bool off();
	static int level();
	static int enter(const char* fmt,...);
	static void leave();
	static int printf(const char* fmt,...);
	static int printf(const std::type_info& type, const char* fmt,...);
	static int printf(void* p, const std::type_info& type, const char* fmt,...);

	static time_t timestamp();
};

#define TRACER_DO_NOTHING do {} while(0)

#ifdef USE_TRACER
#define TRACER_GET_THREAD_NAME() Tracer::getThreadName()
#define TRACER_SET_THREAD_NAME(name) Tracer::setThreadName(name)

#define TRACER_SET(on) Tracer::set(on)
#define TRACER_SET_AND_GET(on) Tracer::set(on)
#define TRACER_ON() Tracer::on()
#define TRACER_ON_IN_SCOPE() Tracer::OnInScope _tracer_on_in_scope_()
#define TRACER_ON_IN_SCOPE_IF(test) Tracer::OnInScope _tracer_on_in_scope_(test)
#define TRACER_ON_IN_SCOPE_ONLY_IF(test) Tracer::OnInScope _tracer_on_in_scope_(test,false)
#define TRACER_OFF() Tracer::off()
#define TRACER_LEVEL() Tracer::level()
#define TRACER_ENTER_SCOPE(...) Tracer::ScopedTracer _scoped_tracer_(__VA_ARGS__)
#define TRACER_ENTER_SCOPE_OF(obj,...) Tracer::ScopedTracer _scoped_tracer_(obj,typeid(*obj),__VA_ARGS__)
#define TRACER_ENTER(...) Tracer::enter(__VA_ARGS__)
#define TRACER_LEAVE() Tracer::leave()
#define TRACER_PRINT(...) Tracer::printf(__VA_ARGS__)
#define TRACER_PRINT_IN_(obj,...) Tracer::printf(obj,typeid(*obj),__VA_ARGS__)

#ifdef USE_TRACER_VERBOSE
#define TRACER_VERBOSE_SET(on) Tracer::set(on)
#define TRACER_VERBOSE_SET_AND_GET(on) Tracer::set(on)
#define TRACER_VERBOSE_ON() Tracer::on()
#define TRACER_VERBOSE_ON_IN_SCOPE() Tracer::OnInScope _tracer_on_in_scope_()
#define TRACER_VERBOSE_ON_IN_SCOPE_IF(test) Tracer::OnInScope _tracer_on_in_scope_(test)
#define TRACER_VERBOSE_ON_IN_SCOPE_ONLY_IF(test) Tracer::OnInScope _tracer_on_in_scope_(test,false)
#define TRACER_VERBOSE_OFF() Tracer::off()
#define TRACER_VERBOSE_ENTER_SCOPE(...) Tracer::ScopedTracer _scoped_tracer_(__VA_ARGS__)
#define TRACER_VERBOSE_ENTER_SCOPE_OF(obj,...) Tracer::ScopedTracer _scoped_tracer_(obj,typeid(*obj),__VA_ARGS__)
#define TRACER_VERBOSE_ENTER(...) Tracer::enter(__VA_ARGS__)
#define TRACER_VERBOSE_LEAVE() Tracer::leave()
#define TRACER_VERBOSE_PRINT(...) Tracer::printf(__VA_ARGS__)
#define TRACER_VERBOSE_PRINT_IN_(obj,...) Tracer::printf(obj,typeid(*obj),__VA_ARGS__)
#else
#define TRACER_VERBOSE_SET(on) TRACER_DO_NOTHING
#define TRACER_VERBOSE_SET_AND_GET(on) false
#define TRACER_VERBOSE_ON() TRACER_DO_NOTHING
#define TRACER_VERBOSE_ON_IN_SCOPE() TRACER_DO_NOTHING
#define TRACER_VERBOSE_ON_IN_SCOPE_IF(test) TRACER_DO_NOTHING
#define TRACER_VERBOSE_ON_IN_SCOPE_ONLY_IF(test) TRACER_DO_NOTHING
#define TRACER_VERBOSE_OFF() TRACER_DO_NOTHING
#define TRACER_VERBOSE_ENTER_SCOPE(...) TRACER_DO_NOTHING
#define TRACER_VERBOSE_ENTER_SCOPE_OF(obj,...) TRACER_DO_NOTHING
#define TRACER_VERBOSE_ENTER(...) TRACER_DO_NOTHING
#define TRACER_VERBOSE_LEAVE() TRACER_DO_NOTHING
#define TRACER_VERBOSE_PRINT(...) TRACER_DO_NOTHING
#define TRACER_VERBOSE_PRINT_IN_(obj,...) TRACER_DO_NOTHING
#endif

#else

#define TRACER_GET_THREAD_NAME() TRACER_DO_NOTHING
#define TRACER_SET_THREAD_NAME(name) TRACER_DO_NOTHING

#define TRACER_SET(on) TRACER_DO_NOTHING
#define TRACER_SET_AND_GET(on) false
#define TRACER_ON() TRACER_DO_NOTHING
#define TRACER_ON_IN_SCOPE() TRACER_DO_NOTHING
#define TRACER_ON_IN_SCOPE_IF(test) TRACER_DO_NOTHING
#define TRACER_ON_IN_SCOPE_ONLY_IF(test) TRACER_DO_NOTHING
#define TRACER_OFF() TRACER_DO_NOTHING
#define TRACER_LEVEL() TRACER_DO_NOTHING
#define TRACER_ENTER_SCOPE(...) TRACER_DO_NOTHING
#define TRACER_ENTER_SCOPE_OF(obj,...) TRACER_DO_NOTHING
#define TRACER_ENTER(...) TRACER_DO_NOTHING
#define TRACER_LEAVE() TRACER_DO_NOTHING
#define TRACER_PRINT(...) TRACER_DO_NOTHING
#define TRACER_PRINT_IN_(obj,...) TRACER_DO_NOTHING

#define TRACER_VERBOSE_SET(on) TRACER_DO_NOTHING
#define TRACER_VERBOSE_SET_AND_GET(on) false
#define TRACER_VERBOSE_ON() TRACER_DO_NOTHING
#define TRACER_VERBOSE_ON_IN_SCOPE() TRACER_DO_NOTHING
#define TRACER_VERBOSE_ON_IN_SCOPE_IF(test) TRACER_DO_NOTHING
#define TRACER_VERBOSE_ON_IN_SCOPE_ONLY_IF(test) TRACER_DO_NOTHING
#define TRACER_VERBOSE_OFF() TRACER_DO_NOTHING
#define TRACER_VERBOSE_ENTER_SCOPE(...) TRACER_DO_NOTHING
#define TRACER_VERBOSE_ENTER_SCOPE_OF(obj,...) TRACER_DO_NOTHING
#define TRACER_VERBOSE_ENTER(...) TRACER_DO_NOTHING
#define TRACER_VERBOSE_LEAVE() TRACER_DO_NOTHING
#define TRACER_VERBOSE_PRINT(...) TRACER_DO_NOTHING
#define TRACER_VERBOSE_PRINT_IN_(obj,...) TRACER_DO_NOTHING
#endif

#endif // LOG_H
