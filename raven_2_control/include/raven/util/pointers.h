/*
 * pointers.h
 *
 *  Created on: Oct 10, 2012
 *      Author: benk
 */

#ifndef POINTERS_H_
#define POINTERS_H_

#include <boost/shared_ptr.hpp>

#define POINTER_TYPES(Type) class Type; \
	typedef boost::shared_ptr<Type> Type##Ptr; \
	typedef boost::shared_ptr<const Type> Type##ConstPtr; \
	typedef boost::weak_ptr<Type> Type##WeakPtr;


#endif /* POINTERS_H_ */
