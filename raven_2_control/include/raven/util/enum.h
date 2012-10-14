///////////////////////////////////////////////////////////////////////////////
// enum.hpp: top level header for BOOST_ENUM
//
// Copyright 2005 Frank Laub
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef BOOST_ENUM_HPP
#define BOOST_ENUM_HPP

//#include <raven/util/enum/iterator.hpp>
///////////////////////////////////////////////////////////////////////////////
// iterator.hpp: defines the enum_iterator type
//
// Copyright 2005 Frank Laub
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef BOOST_ENUM_ITERATOR_HPP
#define BOOST_ENUM_ITERATOR_HPP

// MS compatible compilers support #pragma once
#if defined(_MSC_VER) && (_MSC_VER >= 1020)
# pragma once
#endif

#include <boost/iterator/iterator_facade.hpp>

namespace boost {
namespace detail {

	template <typename T>
	BOOST_DEDUCED_TYPENAME T::domain enum_cast(
		BOOST_DEDUCED_TYPENAME T::index_type index)
	{
		return static_cast<BOOST_DEDUCED_TYPENAME T::domain>(index);
	}

	template <typename T>
	class enum_iterator
		: public boost::iterator_facade
			< enum_iterator<T>
			, const T
			, boost::random_access_traversal_tag>
	{
		typedef
			boost::iterator_facade
				< enum_iterator<T>
				, const T
				, boost::random_access_traversal_tag>
			facade;

		typedef enum_iterator<T> this_type;

	public:
		enum_iterator(size_t index)
			: m_value(enum_cast<T>(index))
			, m_index(index)
		{}

	private:
		friend class boost::iterator_core_access;

		const T& dereference() const
		{
			return m_value;
		}

		void increment()
		{
			++m_index;
			m_value = enum_cast<T>(m_index);
		}

		void decrement()
		{
			--m_index;
			m_value = enum_cast<T>(m_index);
		}

		bool equal(const this_type& rhs) const
		{
			return m_index == rhs.m_index;
		}

		void advance(BOOST_DEDUCED_TYPENAME facade::difference_type n)
		{
			m_index += n;
			m_value = enum_cast<T>(m_index);
		}

		BOOST_DEDUCED_TYPENAME facade::difference_type distance_to(
			const this_type& rhs) const
		{
			return rhs.m_index - m_index;
		}

	private:
		T m_value;
		size_t m_index;
	};

} // detail
} // boost

#endif



//#include <raven/util/enum/base.hpp>
///////////////////////////////////////////////////////////////////////////////
// base.hpp: defines the enum_base type
//
// Copyright 2005 Frank Laub
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef BOOST_ENUM_BASE_HPP
#define BOOST_ENUM_BASE_HPP

// MS compatible compilers support #pragma once
#if defined(_MSC_VER) && (_MSC_VER >= 1020)
# pragma once
#endif

#include <ostream>
#include <istream>
#include <boost/operators.hpp>
#include <boost/optional.hpp>

namespace boost {
namespace detail {

	template <typename Derived, typename ValueType = int>
	class enum_base
		: private boost::totally_ordered<Derived>
	{
	public:
		typedef enum_base<Derived, ValueType> this_type;
		typedef size_t index_type;
		typedef ValueType value_type;
		typedef enum_iterator<Derived> const_iterator;
		typedef boost::optional<Derived> optional;

	public:
		enum_base() {}
		enum_base(index_type index) : m_index(index) {}

		static const_iterator begin()
		{
			return const_iterator(0);
		}

		static const_iterator end()
		{
			return const_iterator(Derived::size);
		}

		static optional get_by_value(value_type value)
		{
			for(index_type i = 0; i < Derived::size; ++i)
			{
				typedef boost::optional<value_type> optional_value;
				optional_value cur = Derived::values(enum_cast<Derived>(i));
				if(value == *cur)
					return Derived(enum_cast<Derived>(i));
			}
			return optional();
		}

		static optional get_by_index(index_type index)
		{
			if(index >= Derived::size) return optional();
			return optional(enum_cast<Derived>(index));
		}

		const char* str() const
		{
			const char* ret = Derived::names(enum_cast<Derived>(m_index));
			BOOST_ASSERT(ret);
			return ret;
		}

		value_type value() const
		{
			typedef boost::optional<value_type> optional_value;
			optional_value ret = Derived::values(enum_cast<Derived>(this->m_index));
			BOOST_ASSERT(ret);
			return *ret;
		}

		index_type index() const
		{
			return m_index;
		}

		bool operator == (const this_type& rhs) const
		{
			return m_index == rhs.m_index;
		}

		bool operator < (const this_type& rhs) const
		{
			value_type lhs_value = value();
			value_type rhs_value = rhs.value();
			if(lhs_value == rhs_value)
				return m_index < rhs.m_index;
			return lhs_value < rhs_value;
		}

	protected:
		friend class enum_iterator<Derived>;
		index_type m_index;
	};

	template <typename D, typename V>
	std::ostream& operator << (std::ostream& os, const enum_base<D, V>& rhs)
	{
		return (os << rhs.str());
	}

	template <typename D, typename V>
	std::istream& operator >> (std::istream& is, enum_base<D, V>& rhs)
	{
		std::string str;
		is >> str;
		BOOST_DEDUCED_TYPENAME D::optional ret = D::get_by_name(str.c_str());
		if(ret)
			rhs = *ret;
		else
			is.setstate(std::ios::badbit);
		return is;
	}

} // detail
} // boost

#endif



//#include <raven/util/enum/bitfield.hpp>
///////////////////////////////////////////////////////////////////////////////
// bitfield.hpp: defines the bitfield_base type
//
// Copyright 2005 Frank Laub
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef BOOST_ENUM_BITFIELD_HPP
#define BOOST_ENUM_BITFIELD_HPP

// MS compatible compilers support #pragma once
#if defined(_MSC_VER) && (_MSC_VER >= 1020)
# pragma once
#endif

#include <sstream>
#include <boost/operators.hpp>

namespace boost {
namespace detail {

	// Befriending Templates requires the need for all this mess.
	// So that we can allow the templated ostream insertion operator to access private members.
	template <typename T>
	class bitfield_base;

	template <typename T>
	std::ostream& operator << (std::ostream& os, const bitfield_base<T>& value);

	class bitfield_access
	{
# if defined(BOOST_NO_MEMBER_TEMPLATE_FRIENDS) \
	|| BOOST_WORKAROUND(__BORLANDC__, BOOST_TESTED_AT(0x551))
	// Tasteless as this may seem, making all members public allows member templates
	// to work in the absence of member template friends.
	public:
# else
		template <typename T>
		friend class bitfield_base;

		template <typename T>
		friend std::ostream& operator << (std::ostream& os, const bitfield_base<T>& value);
#endif

		template <typename T>
		static const char* names(BOOST_DEDUCED_TYPENAME T::domain index)
		{
			return T::names(index);
		}

		template <typename T>
		static BOOST_DEDUCED_TYPENAME T::optional_value values(
			BOOST_DEDUCED_TYPENAME T::domain index)
		{
			return T::values(index);
		}

		template <typename T>
		static T get_by_value(BOOST_DEDUCED_TYPENAME T::value_type value)
		{
			T ret(value, 0);
			return ret;
		}

	private:
		// objects of this class are useless
		bitfield_access(); //undefined
	};

	template <typename T>
	class bitfield_base
		: private boost::bitwise<T>
		, private boost::totally_ordered<T>
	{
	public:
		typedef bitfield_base<T> this_type;
		typedef size_t index_type;
		typedef size_t value_type;
		typedef enum_iterator<T> const_iterator;
		typedef boost::optional<T> optional;

	protected:
		bitfield_base(value_type value, int) : m_value(value) {}

	public:
		bitfield_base() : m_value(0) {}
		bitfield_base(index_type index)
		{
			optional_value value = bitfield_access::values<T>(enum_cast<T>(index));
			if(value)
				m_value = *value;
		}

		static const_iterator begin()
		{
			return const_iterator(0);
		}

		static const_iterator end()
		{
			return const_iterator(T::size);
		}

		static optional get_by_value(value_type value)
		{
			// make sure that 'value' is valid
			optional_value not_mask = bitfield_access::values<T>(T::not_mask);
			BOOST_ASSERT(not_mask);
			if(value & *not_mask)
				return optional();
			return bitfield_access::get_by_value<T>(value);
		}

		static optional get_by_index(index_type index)
		{
			if(index >= T::size) return optional();
			return optional(enum_cast<T>(index));
		}

		std::string str() const
		{
			std::stringstream ss;
			ss << *this;
			return ss.str();
		}

		value_type value() const
		{
			return m_value;
		}

		bool operator == (const this_type& rhs) const
		{
			return m_value == rhs.m_value;
		}

		bool operator < (const this_type& rhs) const
		{
			return m_value < rhs.m_value;
		}

		T& operator |= (const this_type& rhs)
		{
			m_value |= rhs.m_value;
			return static_cast<T&>(*this);
		}

		T& operator &= (const this_type& rhs)
		{
			m_value &= rhs.m_value;
			return static_cast<T&>(*this);
		}

		T& operator ^= (const this_type& rhs)
		{
			m_value ^= rhs.m_value;
			return static_cast<T&>(*this);
		}

		bool operator[] (index_type pos) const
		{
			optional element = get_by_index(pos);
			if(!element) return false;
			return operator[](*element);
		}

		bool operator[] (const this_type& rhs) const
		{
			return (m_value & rhs.m_value) != 0;
		}

		bool set(index_type pos, bool bit = true)
		{
			if(!bit) return reset(pos);
			optional element = get_by_index(pos);
			if(!element) return false;
			return set(*element, bit);
		}

		bool set(const this_type& rhs, bool bit = true)
		{
			if(!bit) return reset(rhs);
			value_type new_value = m_value | rhs.m_value;
			if(!get_by_value(new_value))
				return false;

			m_value = new_value;
			return true;
		}

		bool reset(index_type pos)
		{
			optional element = get_by_index(pos);
			if(!element) return false;
			return reset(*element);
		}

		bool reset(const this_type& rhs)
		{
			value_type new_value = m_value & ~(rhs.m_value);
			if(!get_by_value(new_value)) return false;
			m_value = new_value;
			return true;
		}

		// TO DO: implement me
		size_t count() const
		{
			return 0;
		}

		// TO DO: implement me
		bool any() const
		{
			return false;
		}

		// TO DO: implement me
		bool none() const
		{
			return false;
		}

	private:
		typedef boost::optional<value_type> optional_value;
		friend class bitfield_access;
		value_type m_value;
	};

	template <typename T>
	std::ostream& operator << (std::ostream& os, const bitfield_base<T>& rhs)
	{
		typedef BOOST_DEDUCED_TYPENAME T::value_type value_type;
		typedef BOOST_DEDUCED_TYPENAME T::index_type index_type;
		typedef boost::optional<value_type> optional_value;

		value_type remain = rhs.value();
		optional_value all_mask = bitfield_access::values<T>(T::all_mask);
		if(remain == *all_mask)
		{
			os << "all_mask";
			return os;
		}

		optional_value not_mask = bitfield_access::values<T>(T::not_mask);
		if(remain == *not_mask)
		{
			os << "not_mask";
			return os;
		}
		// FIX ME: there might be a reason the user wants to define the value 0
		//        or perhaps 0 is never legitimate for their usage
		bool isZero = (remain == 0);

		bool isFirst = true;
		for(index_type i = 0; i < T::size; ++i)
		{
			optional_value mask = bitfield_access::values<T>(enum_cast<T>(i));
			if(*mask == 0 && isZero)
			{
				const char* name = bitfield_access::names<T>(enum_cast<T>(i));
				BOOST_ASSERT(name);
				os << name;
				return os;
			}
			else if(remain & *mask)
			{
				if(isFirst)
					isFirst = false;
				else
					os << '|';

				const char* name = bitfield_access::names<T>(enum_cast<T>(i));
				BOOST_ASSERT(name);
				os << name;
				remain &= ~(*mask);
				if(remain == 0)
					return os;
			}
		}
		if(remain)
		{
			if(!isFirst)
				os << '|';
			os.fill('0');
			os.width(8);
			os << std::hex << remain;
		}
		else if(isZero)
		{
			os << "<null>";
		}
		return os;
	}

	// TO DO: this should be symmetrical to operator <<
	//       it needs to be able to take a string like A|B and
	//       return a value that represents A|B.
	template <typename T>
	std::istream& operator >> (std::istream& is, bitfield_base<T>& rhs)
	{
		std::string str;
		is >> str;
		BOOST_DEDUCED_TYPENAME T::optional ret = T::get_by_name(str.c_str());
		if(ret)
			rhs = *ret;
		else
			is.setstate(std::ios::badbit);
		return is;
	}

} // detail
} // boost

#endif



//#include <raven/util/enum/macros.hpp>
///////////////////////////////////////////////////////////////////////////////
// enum_macros.hpp: macros to generate an enum model
//
// Copyright 2005 Frank Laub
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef BOOST_ENUM_MACROS_HPP
#define BOOST_ENUM_MACROS_HPP

// MS compatible compilers support #pragma once
#if defined(_MSC_VER) && (_MSC_VER >= 1020)
# pragma once
#endif

#include <boost/preprocessor.hpp>
#include <cstring>

#define BOOST_ENUM_IS_COLUMN_2(i, k) \
	BOOST_PP_EQUAL(BOOST_PP_MOD(i, 2), k)

#define BOOST_ENUM_GET_COLUMN_2(r, data, i, elem) \
	BOOST_PP_IF(BOOST_ENUM_IS_COLUMN_2(i, data), (elem), BOOST_PP_EMPTY())

#define BOOST_ENUM_VISITOR1(_seq, _macro, _col) \
	BOOST_PP_SEQ_FOR_EACH(_macro, _, _seq)

#define BOOST_ENUM_VISITOR2(_seq, _macro, _col) \
	BOOST_PP_SEQ_FOR_EACH( \
		_macro, \
		_, \
		BOOST_PP_SEQ_FOR_EACH_I(BOOST_ENUM_GET_COLUMN_2, _col, _seq) \
	)

#define BOOST_ENUM_DOMAIN_ITEM(r, data, elem) \
	elem BOOST_PP_COMMA()

#define BOOST_ENUM_domain(_seq, _col, _colsize) \
	enum domain \
	{ \
		BOOST_PP_CAT(BOOST_ENUM_VISITOR, _colsize) \
			(_seq, BOOST_ENUM_DOMAIN_ITEM, _col) \
	}; \
	BOOST_STATIC_CONSTANT(index_type, size = BOOST_ENUM_size(_seq, _colsize));

#define BOOST_ENUM_size(_seq, _colsize) \
	BOOST_PP_DIV(BOOST_PP_SEQ_SIZE(_seq), _colsize)

#define BOOST_ENUM_PARSE_ITEM(r, data, elem) \
	if(strcmp(str, BOOST_PP_STRINGIZE(elem)) == 0) return optional(elem);

#define BOOST_ENUM_get_by_name(_name, _seq, _col, _colsize) \
	static optional get_by_name(const char* str) \
	{ \
		BOOST_PP_CAT(BOOST_ENUM_VISITOR, _colsize) \
			(_seq, BOOST_ENUM_PARSE_ITEM, _col) \
		return optional(); \
	}

#define BOOST_ENUM_CASE_STRING(r, data, elem) \
	case elem: return BOOST_PP_STRINGIZE(elem);

#define BOOST_ENUM_names(_seq, _col, _colsize) \
	static const char* names(domain index) \
	{ \
		BOOST_ASSERT(static_cast<index_type>(index) < size); \
		switch(index) \
		{ \
		BOOST_PP_CAT(BOOST_ENUM_VISITOR, _colsize) \
			(_seq, BOOST_ENUM_CASE_STRING, _col) \
		default: return NULL; \
		} \
	}

#define BOOST_ENUM_CASE_PART(elem) \
	case (elem):

#define BOOST_ENUM_VALUE_PART(elem) \
	return optional_value(elem);

#define BOOST_ENUM_VALUE_LINE(r, data, i, elem) \
	BOOST_PP_IF(BOOST_ENUM_IS_COLUMN_2(i, 0), \
		BOOST_ENUM_CASE_PART(elem), \
		BOOST_ENUM_VALUE_PART(elem) \
	)

#define BOOST_ENUM_values_identity() \
	typedef boost::optional<value_type> optional_value; \
	static optional_value values(domain index) \
	{ \
		if(static_cast<index_type>(index) < size) return optional_value(index); \
		return optional_value(); \
	}

#define BOOST_ENUM_values(_seq, _name_col, _value_col, _colsize) \
	typedef boost::optional<value_type> optional_value; \
	static optional_value values(domain index) \
	{ \
		switch(index) \
		{ \
		BOOST_PP_SEQ_FOR_EACH_I(BOOST_ENUM_VALUE_LINE, _, _seq) \
		default: return optional_value(); \
		} \
	}

#define BOOST_BITFIELD_names(_seq, _col, _colsize) \
	static const char* names(domain index) \
	{ \
		BOOST_ASSERT(static_cast<index_type>(index) < size); \
		switch(index) \
		{ \
		case all_mask: return "all_mask"; \
		BOOST_PP_CAT(BOOST_ENUM_VISITOR, _colsize) \
			(_seq, BOOST_ENUM_CASE_STRING, _col) \
		default: return NULL; \
		} \
	}

#define BOOST_BITFIELD_OR_ITEM(r, data, elem) \
	| (elem)

#define BOOST_BITFIELD_domain(_seq, _col, _colsize) \
	enum domain \
	{ \
		BOOST_PP_CAT(BOOST_ENUM_VISITOR, _colsize) \
			(_seq, BOOST_ENUM_DOMAIN_ITEM, _col) \
		all_mask, \
		not_mask \
	}; \
	BOOST_STATIC_CONSTANT(index_type, size = BOOST_ENUM_size(_seq, _colsize));

#define BOOST_BITFIELD_all_mask(_seq, _col, _colsize) \
	0 BOOST_PP_CAT(BOOST_ENUM_VISITOR, _colsize) \
		(_seq, BOOST_BITFIELD_OR_ITEM, _col)

#define BOOST_BITFIELD_get_by_name(_name, _seq, _col, _colsize) \
	static optional get_by_name(const char* str) \
	{ \
		if(strcmp(str, "all_mask") == 0) return optional(all_mask); \
		if(strcmp(str, "not_mask") == 0) return optional(not_mask); \
		BOOST_PP_CAT(BOOST_ENUM_VISITOR, _colsize) \
			(_seq, BOOST_ENUM_PARSE_ITEM, _col) \
		return optional(); \
	}

#define BOOST_BITFIELD_values(_seq, _name_col, _value_col, _colsize) \
	typedef boost::optional<value_type> optional_value; \
	static optional_value values(domain index) \
	{ \
		switch(index) \
		{ \
		BOOST_PP_SEQ_FOR_EACH_I(BOOST_ENUM_VALUE_LINE, _, _seq) \
		case all_mask: return optional_value(BOOST_BITFIELD_all_mask(_seq, _value_col, _colsize)); \
		case not_mask: return optional_value(~(BOOST_BITFIELD_all_mask(_seq, _value_col, _colsize))); \
		default: return optional_value(); \
		} \
	}

#define BOOST_ENUM(_name, _seq) \
	class _name : public boost::detail::enum_base<_name> \
	{ \
	public: \
		BOOST_ENUM_domain(_seq, 0, 1) \
		_name() {} \
		_name(domain index) : boost::detail::enum_base<_name>(index) {} \
		BOOST_ENUM_get_by_name(_name, _seq, 0, 1) \
	private: \
		friend class boost::detail::enum_base<_name>; \
		BOOST_ENUM_names(_seq, 0, 1) \
		BOOST_ENUM_values_identity() \
	};

#define BOOST_ENUM_VALUES(_name, _type, _seq) \
	class _name : public boost::detail::enum_base<_name, _type> \
	{ \
	public: \
		BOOST_ENUM_domain(_seq, 0, 2) \
		_name() {} \
		_name(domain index) : boost::detail::enum_base<_name, _type>(index) {} \
		BOOST_ENUM_get_by_name(_name, _seq, 0, 2) \
	private: \
		friend class boost::detail::enum_base<_name, _type>; \
		BOOST_ENUM_names(_seq, 0, 2) \
		BOOST_ENUM_values(_seq, 0, 1, 2) \
	};

#define BOOST_ENUM_STRINGS(_name, _seq) BOOST_ENUM_VALUES(_name, const char*, _seq)

#define BOOST_BITFIELD(_name, _seq) \
	class _name : public boost::detail::bitfield_base<_name> \
	{ \
	public: \
		BOOST_BITFIELD_domain(_seq, 0, 2) \
		_name() {} \
		_name(domain index) : boost::detail::bitfield_base<_name>(index) {} \
		BOOST_ENUM_get_by_name(_name, _seq, 0, 2) \
	private: \
		friend class boost::detail::bitfield_access; \
		_name(value_type raw, int) : boost::detail::bitfield_base<_name>(raw, 0) {} \
		BOOST_BITFIELD_names(_seq, 0, 2) \
		BOOST_BITFIELD_values(_seq, 0, 1, 2) \
	};

#endif

#endif
