/*
 * runlevel.h
 *
 *  Created on: Oct 14, 2012
 *      Author: benk
 */

#ifndef RUNLEVEL_H_
#define RUNLEVEL_H_

#include <string>

#define USE_NEW_RUNLEVEL

typedef unsigned char runlevel_t;

class RunLevel {
	friend class Device;
private:
	runlevel_t value_;
	runlevel_t sublevel_;

	static bool IS_INITED;
	static bool HAS_HOMED;
	static RunLevel* INSTANCE;
	static bool PEDAL;
	static bool SOFTWARE_ESTOP;
public:
	static RunLevel get();
	static void eStop();
	static void setPedal(bool down);
	static bool hasHomed();

	bool isEstop() const;
	bool isHardwareEstop() const;
	bool isSoftwareEstop() const;
	bool isInitSublevel(const runlevel_t& sublevel) const;
	bool isInit() const;
	bool isPedalUp() const;
	bool isPedalDown() const;
	bool isPedalUpOrDown() const { return isPedalDown() || isPedalUp(); }

	bool isActive() const;

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

#endif /* RUNLEVEL_H_ */
