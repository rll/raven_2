/*
 * pointers.h
 *
 *  Created on: Oct 10, 2012
 *      Author: benk
 */

#ifndef POINTERS_H_
#define POINTERS_H_

#include <boost/shared_ptr.hpp>
#include <vector>

#define POINTER_TYPES(Type) class Type; \
	typedef boost::shared_ptr<Type> Type##Ptr; \
	typedef boost::shared_ptr<const Type> Type##ConstPtr; \
	/*typedef boost::weak_ptr<Type> Type##WeakPtr;*/ \
	template<class U> \
	inline Type##Ptr cast(boost::shared_ptr<U> const & r) { return boost::dynamic_pointer_cast<Type,U>(r); } \
	template<class U> \
	inline bool cast(boost::shared_ptr<U> const & from, Type##Ptr& to) { to = boost::dynamic_pointer_cast<Type,U>(from); return to.get(); } \
	typedef std::vector<Type##Ptr> Type##List; \
	typedef std::vector<Type##ConstPtr> Const##Type##List; \
	inline Const##Type##List constList(const Type##List& list_in) { \
		Const##Type##List list; \
		Type##List::const_iterator itr; \
		for (itr=list_in.begin();itr!=list_in.end();itr++) { \
			Type##Ptr ptr = *itr; \
			Type##ConstPtr constPtr = ptr; \
			list.push_back(constPtr); \
		} \
		return list; \
	}


#endif /* POINTERS_H_ */
