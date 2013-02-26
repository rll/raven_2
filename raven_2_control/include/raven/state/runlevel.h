/*
 * runlevel.h
 *
 *  Created on: Oct 14, 2012
 *      Author: benk
 */

#ifndef RUNLEVEL_H_
#define RUNLEVEL_H_

#include <string>
#include <map>
//#include <atomic>
#include <boost/thread/tss.hpp>

#ifndef STRINGIFY
#define STRINGIFY(s) STRINGIFY_HELPER(s)
#define STRINGIFY_HELPER(s) #s
#endif

#define USE_NEW_RUNLEVEL

typedef unsigned char runlevel_t;

class RunLevel {
	friend class Device;
private:
	runlevel_t value_;
	runlevel_t sublevel_;
	std::map<int,bool> armsActive_;

	static bool IS_INITED;
	static bool HAS_HOMED;
	static int FIRST_HOMED_LOOP;
	static RunLevel* PREVIOUS_RUNLEVEL;
	static RunLevel* INSTANCE;
	static bool PEDAL;
	static bool SOFTWARE_ESTOP;
	static std::map<int,bool> ARMS_ACTIVE;
public:
	//static std::atomic_uint_least32_t LOOP_NUMBER;

	static RunLevel get();
	static RunLevel previous();
	static bool changed(bool checkSublevel=false);
	static bool hasHomed();
	static bool newlyHomed();

	static void eStop();

	//static void setPedal(bool down);
	static void setArmActive(int armId,bool active=true);

	bool isEstop() const;
	bool isHardwareEstop() const;
	bool isSoftwareEstop() const;
	bool isInitSublevel(const runlevel_t& sublevel) const;
	bool isInit() const;
	bool isPedalUp() const;
	bool isPedalDown() const;
	bool isPedalUpOrDown() const { return isPedalDown() || isPedalUp(); }

	//bool isActive() const;
	bool isArmActive(int armId) const;

	std::string str() const;

	static RunLevel _E_STOP_();
	static RunLevel _INIT_(runlevel_t sublevel = 0);
	static RunLevel _PEDAL_UP_();
	static RunLevel _PEDAL_DOWN_();

	//internals
	template<typename T>
	void getNumbers(T& level,T& sublevel) {
		level = (T) value_;
		sublevel = (T) sublevel_;
	}
	static void updateRunlevel(runlevel_t level);
	static void setSublevel(runlevel_t sublevel);
	static bool getPedal(); //call from rt process
	static void setInitialized(bool value);
	static bool isInitialized();
private:
	RunLevel(runlevel_t level, runlevel_t sub = 0) : value_(level), sublevel_(sub) {}
	static RunLevel _E_STOP_HARDWARE_();
	static RunLevel _E_STOP_SOFTWARE_();
};

class LoopNumber {
public:
	struct CountInfo {
		int count;
		int loop;
	};
private:
	static std::map<std::string,int> NAMED_INTERVALS;
	static boost::thread_specific_ptr< std::map<std::string,CountInfo> > NAMED_COUNTS;

	static boost::thread_specific_ptr<int> LOOP_NUMBER;
	static int MAIN_LOOP_NUMBER;

	static int internalGet();
	static int internalGetNamedInterval(const std::string& name);
	static bool internalOnlyEvery(const std::string& name, int limit, int interval);
	static bool internalOnlyEveryAfter(const std::string& name, int min, int interval);
	static CountInfo internalGetNamedCount(const std::string& name);

	LoopNumber() {}
public:
	static int get();
	static int getMain();

	static void increment(int amt=1);
	static void incrementMain(int amt=1);

	static bool is(int loop);
	static bool after(int loop);
	static bool before(int loop);
	static bool between(int start_inclusive,int end_exclusive);
	static bool every(int interval);
	static bool everyMain(int interval);
	static inline bool always(int interval=-1) { return true; } //for convenience

	static void setNamedInterval(const std::string& name,int interval);
	static int getNamedInterval(const std::string& name);
	static bool every(const std::string& name);
	static bool everyMain(const std::string& name);
	static inline bool always(const std::string& name) { return true; } //for convenience

	static int getNamedCount(const std::string& name);

	static bool once(const std::string& name);
#define LOOP_NUMBER_ONCE(file,line) if (::LoopNumber::once(STRINGIFY(file) STRINGIFY(line))) //use: LOOP_NUMBER_ONCE(__FILE__,__LINE__) { }

	static bool only(const std::string& name, int limit);
	static bool onlyAfter(const std::string& name, int min);

	static bool onlyEvery(const std::string& name, int limit);
	static bool onlyEvery(const std::string& name, int limit, int interval);

	static bool onlyEveryAfter(const std::string& name, int min);
	static bool onlyEveryAfter(const std::string& name, int min, int interval);

};
#endif /* RUNLEVEL_H_ */
