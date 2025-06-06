// SPDX-License-Identifier: BSD-2-Clause
// author: Max Kellermann <max.kellermann@gmail.com>

#pragma once

#include <curl/curl.h>

#include <stdio.h>
#include <stdexcept>
#include <utility>

/**
 * OO wrapper for "struct curl_slist *".
 */
class CurlSlist {
	struct curl_slist *head = nullptr;

public:
	CurlSlist() noexcept = default;

	CurlSlist(CurlSlist &&src) noexcept
		:head(std::exchange(src.head, nullptr)) {}

	~CurlSlist() noexcept {
		if (head != nullptr)
			curl_slist_free_all(head);
	}

	CurlSlist &operator=(CurlSlist &&src) noexcept {
		std::swap(head, src.head);
		return *this;
	}

	struct curl_slist *Get() noexcept {
		return head;
	}

	void Clear() noexcept {
		curl_slist_free_all(head);
		head = nullptr;
	}

	void Append(const char *value) {
		auto *new_head = curl_slist_append(head, value);
		if (new_head == nullptr)
			throw std::runtime_error("curl_slist_append() failed");
		head = new_head;
	}

   /**
   * Use snprintf() to append to this string.  The value is
   * truncated if it would become too long for the buffer.
   */
  template <typename... Args>
  void AppendFormat(std::string::const_pointer fmt, Args &&...args) noexcept
  {
    char buffer[4096];
    snprintf(buffer, sizeof(buffer), fmt, args...);
    Append(buffer);
  }
};
