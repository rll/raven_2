/*
 * history.h
 *
 *  Created on: Oct 9, 2012
 *      Author: benk
 */

#ifndef HISTORY_H_
#define HISTORY_H_

#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>

template<class T>
struct CloningWrapper {
	typedef boost::shared_ptr<T> ValueType;
	ValueType value;

	CloningWrapper() : value() {}
	CloningWrapper(const ValueType& theValue) : value(theValue) {}
	CloningWrapper(const CloningWrapper& other) {
		other.value->cloneInto(value);
	}

	CloningWrapper& operator=(const CloningWrapper& other) {
		other.value->cloneInto(value);
		return *this;
	}
};

template<class T>
struct History {
private:
	History() {}
public:
	typedef boost::circular_buffer<CloningWrapper<T> > Type;
};

#endif /* HISTORY_H_ */
