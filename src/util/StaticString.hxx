// SPDX-License-Identifier: BSD-2-Clause
// author: Max Kellermann <max.kellermann@gmail.com>

#pragma once

#include "StringBuffer.hxx"
#include "StringAPI.hxx"
#include "StringUtil.hpp"
#include "StringFormat.hpp"
#include "StringCompare.hxx"
#include "UTF8.hpp"
#include "ASCII.hxx"

#include <cassert>
#include <cstddef>
#include <string_view>

bool
CopyUTF8(char *dest, size_t dest_size, const char *src) noexcept;

/**
 * A string with a maximum size known at compile time.
 */
template<typename T, size_t max>
class StaticStringBase : public BasicStringBuffer<T, max> {
	typedef BasicStringBuffer<T, max> Base;

public:
	using typename Base::value_type;
	using typename Base::reference;
	using typename Base::pointer;
	using typename Base::const_pointer;
	using typename Base::const_iterator;
	using typename Base::size_type;

	using string_view = std::basic_string_view<value_type>;

	static constexpr value_type SENTINEL = Base::SENTINEL;

	StaticStringBase() = default;
	explicit StaticStringBase(string_view value) noexcept {
		assign(value);
	}

	using Base::capacity;

	size_type length() const noexcept {
		return StringLength(c_str());
	}

	using Base::empty;

	bool full() const noexcept {
		return length() >= capacity() - 1;
	}

	/**
	 * Truncate the string to the specified length.
	 *
	 * @param new_length the new length; must be equal or smaller
	 * than the current length
	 */
	constexpr void Truncate(size_type new_length) noexcept {
		assert(new_length <= length());

		data()[new_length] = SENTINEL;
	}

	void SetASCII(std::string_view src) noexcept {
		pointer end = ::CopyASCII(data(), capacity() - 1, src);
		*end = SENTINEL;
	}

	/**
	 * Eliminate all non-ASCII characters.
	 */
	void CleanASCII() noexcept {
		CopyASCII(data(), c_str());
	}

	/**
	 * Copy from the specified UTF-8 string.
	 *
	 * @return false if #src was invalid UTF-8
	 */
	bool SetUTF8(const char *src) noexcept {
		return ::CopyUTF8(data(), capacity(), src);
	}

	bool equals(const_pointer other) const noexcept {
		assert(other != nullptr);

		return StringIsEqual(c_str(), other);
	}

	[[gnu::pure]]
	bool StartsWith(const_pointer prefix) const noexcept {
		return StringStartsWith(c_str(), prefix);
	}

	[[gnu::pure]]
	bool Contains(const_pointer needle) const noexcept {
		return StringFind(c_str(), needle) != nullptr;
	}

	using Base::data;

	/**
	 * Returns a writable buffer.
	 */
	constexpr pointer buffer() noexcept {
		return data();
	}

	/**
	 * Returns one character.  No bounds checking.
	 */
	constexpr value_type operator[](size_type i) const noexcept {
		assert(i <= length());

		return Base::operator[](i);
	}

	/**
	 * Returns one writable character.  No bounds checking.
	 */
	reference operator[](size_type i) noexcept {
		assert(i <= length());

		return Base::operator[](i);
	}

	using Base::begin;

	const_iterator end() const noexcept {
		return begin() + length();
	}

	using Base::front;

	value_type back() const noexcept {
		return end()[-1];
	}

	void assign(string_view new_value) noexcept {
		CopyString(data(), capacity(), new_value);
	}

	void append(string_view new_value) noexcept {
		size_type len = length();
		CopyString(data() + len, capacity() - len, new_value);
	}

	bool push_back(value_type ch) noexcept {
		size_t l = length();
		if (l >= capacity() - 1)
			return false;

		auto *p = data() + l;
		*p++ = ch;
		*p = SENTINEL;
		return true;
	}

	/**
	 * Append ASCII characters from the specified string without
	 * buffer boundary checks.
	 */
	void UnsafeAppendASCII(const char *p) noexcept {
		CopyASCII(data() + length(), p);
	}

	using Base::c_str;

	constexpr operator const_pointer() const noexcept {
		return c_str();
	}

	[[gnu::pure]]
	operator string_view() const noexcept {
		return c_str();
	}

	bool operator ==(const_pointer value) const noexcept {
		return equals(value);
	}

	bool operator !=(const_pointer value) const noexcept {
		return !equals(value);
	}

	template<std::size_t other_max>
	[[gnu::pure]]
	bool operator==(const StaticStringBase<T, other_max> &other) const noexcept {
		return *this == other.c_str();
	}

	template<std::size_t other_max>
	[[gnu::pure]]
	bool operator!=(const StaticStringBase<T, other_max> &other) const noexcept {
		return *this != other.c_str();
	}

	StaticStringBase<T, max> &operator=(string_view new_value) noexcept {
		assign(new_value);
		return *this;
	}

	StaticStringBase<T, max> &operator+=(string_view new_value) noexcept {
		append(new_value);
		return *this;
	}

	StaticStringBase<T, max> &operator+=(value_type ch) noexcept {
		push_back(ch);
		return *this;
	}

	/**
	 * Use snprintf() to set the value of this string.  The value
	 * is truncated if it is too long for the buffer.
	 */
	template<typename... Args>
	std::basic_string_view<T> Format(const_pointer fmt, Args&&... args) noexcept {
		int s_length = StringFormat(data(), capacity(), fmt, args...);
		if (s_length < 0)
			/* error */
			return {};

		size_type length = (size_type)s_length;
		if (length >= capacity())
			/* truncated */
			length = capacity() - 1;

		return {data(), length};
	}

	/**
	 * Use snprintf() to append to this string.  The value is
	 * truncated if it would become too long for the buffer.
	 */
	template<typename... Args>
	void AppendFormat(const_pointer fmt, Args&&... args) noexcept {
		size_t l = length();
		StringFormat(data() + l, capacity() - l, fmt, args...);
	}

	/**
	 * Use sprintf() to set the value of this string.  WARNING:
	 * this does not check if the new value fits into the buffer,
	 * and might overflow.  Use only when you are sure that the
	 * buffer is big enough!
	 */
	template<typename... Args>
	std::basic_string_view<T> UnsafeFormat(const T *fmt, Args&&... args) noexcept {
		int s_length = StringFormatUnsafe(data(), fmt, args...);
		if (s_length < 0)
			/* error */
			return {};

		size_type length = (size_type)s_length;
		return {data(), length};
	}
};

/**
 * A string with a maximum size known at compile time.
 * This is the char-based sister of the StaticString class.
 */
template<size_t max>
class StaticString: public StaticStringBase<char, max>
{
	typedef StaticStringBase<char, max> Base;

public:
	using typename Base::value_type;
	using typename Base::reference;
	using typename Base::pointer;
	using typename Base::const_pointer;
	using typename Base::const_iterator;
	using typename Base::size_type;

	using Base::Base;
	using Base::operator=;
	using Base::operator+=;

	void CropIncompleteUTF8() noexcept {
		::CropIncompleteUTF8(this->data());
	}
};

