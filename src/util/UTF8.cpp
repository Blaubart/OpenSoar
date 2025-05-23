// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "UTF8.hpp"
#include "CharUtil.hxx"
#include "Compiler.h"

#include <algorithm>
#include <cassert>

/**
 * Is this a leading byte that is followed by 1 continuation byte?
 */
static constexpr bool
IsLeading1(unsigned char ch) noexcept
{
  return (ch & 0xe0) == 0xc0;
}

static constexpr unsigned char
MakeLeading1(unsigned char value) noexcept
{
  return 0xc0 | value;
}

/**
 * Is this a leading byte that is followed by 2 continuation byte?
 */
static constexpr bool
IsLeading2(unsigned char ch) noexcept
{
  return (ch & 0xf0) == 0xe0;
}

static constexpr unsigned char
MakeLeading2(unsigned char value) noexcept
{
  return 0xe0 | value;
}

/**
 * Is this a leading byte that is followed by 3 continuation byte?
 */
static constexpr bool
IsLeading3(unsigned char ch) noexcept
{
  return (ch & 0xf8) == 0xf0;
}

static constexpr unsigned char
MakeLeading3(unsigned char value) noexcept
{
  return 0xf0 | value;
}

/**
 * Is this a leading byte that is followed by 4 continuation byte?
 */
static constexpr bool
IsLeading4(unsigned char ch) noexcept
{
  return (ch & 0xfc) == 0xf8;
}

static constexpr unsigned char
MakeLeading4(unsigned char value) noexcept
{
  return 0xf8 | value;
}

/**
 * Is this a leading byte that is followed by 5 continuation byte?
 */
static constexpr bool
IsLeading5(unsigned char ch) noexcept
{
  return (ch & 0xfe) == 0xfc;
}

static constexpr unsigned char
MakeLeading5(unsigned char value) noexcept
{
  return 0xfc | value;
}

static constexpr bool
IsContinuation(unsigned char ch) noexcept
{
  return (ch & 0xc0) == 0x80;
}

/**
 * Generate a continuation byte of the low 6 bit.
 */
static constexpr unsigned char
MakeContinuation(unsigned char value) noexcept
{
  return 0x80 | (value & 0x3f);
}

bool
ValidateUTF8(const char *p) noexcept
{
  for (; *p != 0; ++p) {
    unsigned char ch = *p;
    if (IsASCII(ch))
      continue;

    if (IsContinuation(ch))
      /* continuation without a prefix */
      return false;

    if (IsLeading1(ch)) {
      /* 1 continuation */
      if (!IsContinuation(*++p))
        return false;
    } else if (IsLeading2(ch)) {
      /* 2 continuations */
      if (!IsContinuation(*++p) || !IsContinuation(*++p))
        return false;
    } else if (IsLeading3(ch)) {
      /* 3 continuations */
      if (!IsContinuation(*++p) || !IsContinuation(*++p) ||
          !IsContinuation(*++p))
        return false;
    } else if (IsLeading4(ch)) {
      /* 4 continuations */
      if (!IsContinuation(*++p) || !IsContinuation(*++p) ||
          !IsContinuation(*++p) || !IsContinuation(*++p))
        return false;
    } else if (IsLeading5(ch)) {
      /* 5 continuations */
      if (!IsContinuation(*++p) || !IsContinuation(*++p) ||
          !IsContinuation(*++p) || !IsContinuation(*++p) ||
          !IsContinuation(*++p))
        return false;
    } else
      return false;
  }

  return true;
}

static char
Shift(std::string_view &s) noexcept
{
  char ch = s.front();
  s.remove_prefix(1);
  return ch;
}

bool
ValidateUTF8(std::string_view p) noexcept
{
  while (!p.empty()) {
    unsigned char ch = Shift(p);
    if (IsASCII(ch))
      continue;

    if (IsContinuation(ch))
      /* continuation without a prefix */
      return false;

    if (IsLeading1(ch)) {
      /* 1 continuation */
      if (p.size() < 1 || !IsContinuation(Shift(p)))
        return false;
    } else if (IsLeading2(ch)) {
      /* 2 continuations */
      if (p.size() < 2 || !IsContinuation(Shift(p)) ||
          !IsContinuation(Shift(p)))
        return false;
    } else if (IsLeading3(ch)) {
      /* 3 continuations */
      if (p.size() < 3 || !IsContinuation(Shift(p)) ||
          !IsContinuation(Shift(p)) ||
          !IsContinuation(Shift(p)))
        return false;
    } else if (IsLeading4(ch)) {
      /* 4 continuations */
      if (p.size() < 4 || !IsContinuation(Shift(p)) ||
          !IsContinuation(Shift(p)) ||
          !IsContinuation(Shift(p)) || !IsContinuation(Shift(p)))
        return false;
    } else if (IsLeading5(ch)) {
      /* 5 continuations */
      if (p.size() < 5 || !IsContinuation(Shift(p)) ||
          !IsContinuation(Shift(p)) || !IsContinuation(Shift(p)) ||
          !IsContinuation(Shift(p)) || !IsContinuation(Shift(p)))
        return false;
    } else
      return false;
  }

  return true;
}

std::size_t
SequenceLengthUTF8(char ch) noexcept
{
  if (IsASCII(ch))
    return 1;
  else if (IsLeading1(ch))
    /* 1 continuation */
    return 2;
  else if (IsLeading2(ch))
    /* 2 continuations */
    return 3;
  else if (IsLeading3(ch))
    /* 3 continuations */
    return 4;
  else if (IsLeading4(ch))
    /* 4 continuations */
    return 5;
  else if (IsLeading5(ch))
    /* 5 continuations */
    return 6;
  else
    /* continuation without a prefix or some other illegal
       start byte */
    return 0;
}

template<std::size_t L>
struct CheckSequenceUTF8 {
  [[gnu::pure]]
  bool operator()(const char *p) const noexcept {
    return IsContinuation(*p) && CheckSequenceUTF8<L-1>()(p + 1);
  }
};

template<>
struct CheckSequenceUTF8<0u> {
  constexpr bool operator()(const char *) const noexcept {
    return true;
  }
};

template<std::size_t L>
[[gnu::pure]]
static std::size_t
InnerSequenceLengthUTF8(const char *p) noexcept
{
  return CheckSequenceUTF8<L>()(p)
    ? L + 1
    : 0u;
}

std::size_t
SequenceLengthUTF8(const char *p) noexcept
{
  const unsigned char ch = *p++;

  if (IsASCII(ch))
    return 1;
  else if (IsLeading1(ch))
    /* 1 continuation */
    return InnerSequenceLengthUTF8<1>(p);
  else if (IsLeading2(ch))
    /* 2 continuations */
    return InnerSequenceLengthUTF8<2>(p);
  else if (IsLeading3(ch))
    /* 3 continuations */
    return InnerSequenceLengthUTF8<3>(p);
  else if (IsLeading4(ch))
    /* 4 continuations */
    return InnerSequenceLengthUTF8<4>(p);
  else if (IsLeading5(ch))
    /* 5 continuations */
    return InnerSequenceLengthUTF8<5>(p);
  else
    /* continuation without a prefix or some other illegal
       start byte */
    return 0;
}

static const char *
FindNonASCIIOrZero(const char *p) noexcept
{
  while (*p != 0 && IsASCII(*p))
    ++p;
  return p;
}

char *
Latin1ToUTF8(unsigned char ch, char *buffer) noexcept
{
  if (IsASCII(ch)) {
    *buffer++ = ch;
  } else {
    *buffer++ = MakeLeading1(ch >> 6);
    *buffer++ = MakeContinuation(ch);
  }

  return buffer;
}

const char *
Latin1ToUTF8(const char *gcc_restrict src,
             std::span<char> buffer) noexcept
{
  const char *p = FindNonASCIIOrZero(src);
  if (*p == 0)
    /* everything is plain ASCII, we don't need to convert anything */
    return src;

  if ((std::size_t)(p - src) >= buffer.size())
    /* buffer too small */
    return nullptr;

  const char *const end = buffer.data() + buffer.size();
  char *q = std::copy(src, p, buffer.data());

  while (*p != 0) {
    unsigned char ch = *p++;

    if (IsASCII(ch)) {
      *q++ = ch;

      if (q >= end)
        /* buffer too small */
        return nullptr;
    } else {
      if (q + 2 >= end)
        /* buffer too small */
        return nullptr;

      *q++ = MakeLeading1(ch >> 6);
      *q++ = MakeContinuation(ch);
    }
  }

  *q = 0;
  return buffer.data();
}

static const char *
FindNonASCII(std::string_view p) noexcept
{
  for (const auto &i : p)
    if (!IsASCII(i))
      return &i;
  return nullptr;
}

std::string_view
Latin1ToUTF8(std::string_view src,
             std::span<char> buffer) noexcept
{
  const char *p = FindNonASCII(src);
  if (p == nullptr)
    /* everything is plain ASCII, we don't need to convert anything */
    return src;

  if ((std::size_t)(p - src.data()) > buffer.size())
    /* buffer too small */
    return {};

  const char *const end = buffer.data() + buffer.size();
  char *q = std::copy(src.data(), p, buffer.data());

  const char *const src_end = src.data() + src.size();
  while (p < src_end) {
    unsigned char ch = *p++;

    if (IsASCII(ch)) {
      *q++ = ch;

      if (q >= end)
        /* buffer too small */
        return {};
    } else {
      if (q + 2 >= end)
        /* buffer too small */
        return {};

      *q++ = MakeLeading1(ch >> 6);
      *q++ = MakeContinuation(ch);
    }
  }

  return {buffer.data(), std::size_t(q - buffer.data())};
}

char *
UnicodeToUTF8(unsigned ch, char *q) noexcept
{
  if (ch < 0x80) [[likely]] {
    *q++ = (char)ch;
  } else if (ch < 0x800) [[likely]] {
    *q++ = MakeLeading1(ch >> 6);
    *q++ = MakeContinuation(ch);
  } else if (ch < 0x10000) {
    *q++ = MakeLeading2(ch >> 12);
    *q++ = MakeContinuation(ch >> 6);
    *q++ = MakeContinuation(ch);
  } else if (ch < 0x200000) {
    *q++ = MakeLeading3(ch >> 18);
    *q++ = MakeContinuation(ch >> 12);
    *q++ = MakeContinuation(ch >> 6);
    *q++ = MakeContinuation(ch);
  } else if (ch < 0x4000000) {
    *q++ = MakeLeading4(ch >> 24);
    *q++ = MakeContinuation(ch >> 18);
    *q++ = MakeContinuation(ch >> 12);
    *q++ = MakeContinuation(ch >> 6);
    *q++ = MakeContinuation(ch);
  } else if (ch < 0x80000000) {
    *q++ = MakeLeading5(ch >> 30);
    *q++ = MakeContinuation(ch >> 24);
    *q++ = MakeContinuation(ch >> 18);
    *q++ = MakeContinuation(ch >> 12);
    *q++ = MakeContinuation(ch >> 6);
    *q++ = MakeContinuation(ch);
  } else {
    // error
  }

  return q;
}

std::size_t
LengthUTF8(const char *p) noexcept
{
  /* this is a very naive implementation: it does not do any
     verification, it just counts the bytes that are not a UTF-8
     continuation  */

  std::size_t n = 0;
  for (; *p != 0; ++p)
    if (!IsContinuation(*p))
      ++n;
  return n;
}

/**
 * Find the null terminator.
 */
[[gnu::pure]]
static char *
FindTerminator(char *p) noexcept
{
  assert(p != nullptr);

  while (*p != 0)
    ++p;

  return p;
}

/**
 * Find the leading byte for the given continuation byte.
 */
[[gnu::pure]]
static char *
FindLeading([[maybe_unused]] char *const begin, char *i) noexcept
{
  assert(i > begin);
  assert(IsContinuation(*i));

  while (IsContinuation(*--i)) {
    assert(i > begin);
  }

  return i;
}

char *
CropIncompleteUTF8(char *const p) noexcept
{
  char *const end = FindTerminator(p);
  if (end == p)
    return end;

  char *const last = end - 1;
  if (!IsContinuation(*last)) {
    char *result = end;
    if (!IsASCII(*last)) {
      *last = 0;
      result = last;
    }

    assert(ValidateUTF8(p));
    return result;
  }

  char *const leading = FindLeading(p, last);
  const std::size_t n_continuations = last - leading;
  assert(n_continuations > 0);

  const unsigned char ch = *leading;

  unsigned expected_continuations;
  if (IsLeading1(ch))
    expected_continuations = 1;
  else if (IsLeading2(ch))
    expected_continuations = 2;
  else if (IsLeading3(ch))
    expected_continuations = 3;
  else if (IsLeading4(ch))
    expected_continuations = 4;
  else if (IsLeading5(ch))
    expected_continuations = 5;
  else {
    assert(n_continuations == 0);
    gcc_unreachable();
  }

  assert(n_continuations <= expected_continuations);

  char *result = end;

  if (n_continuations < expected_continuations) {
    /* this continuation is incomplete: truncate here */
    *leading = 0;
    result = leading;
  }

  /* now the string must be completely valid */
  assert(ValidateUTF8(p));

  return result;
}

std::size_t
TruncateStringUTF8(std::string_view s, std::size_t max_chars) noexcept
{
  assert(ValidateUTF8(s));

  std::size_t result = 0;
  while (!s.empty() && max_chars > 0) {
    std::size_t sequence = SequenceLengthUTF8(s.front());
    if (sequence > s.size())
      break;

    result += sequence;
    s.remove_prefix(sequence);
    --max_chars;
  }

  return result;
}

std::size_t
TruncateStringUTF8(const char *p,
                   std::size_t max_chars, std::size_t max_bytes) noexcept
{
#ifndef __clang__
  /* disabled on clang due to -Wtautological-pointer-compare */
  assert(p != nullptr);
#endif
  assert(ValidateUTF8(p));

  std::size_t result = 0;
  while (max_chars > 0 && *p != '\0') {
    std::size_t sequence = SequenceLengthUTF8(*p);
    if (sequence > max_bytes)
      break;

    result += sequence;
    max_bytes -= sequence;
    p += sequence;
    --max_chars;
  }

  return result;
}

char *
CopyTruncateStringUTF8(std::span<char> dest,
                       const char *src, std::size_t truncate) noexcept
{
  assert(dest.data() != nullptr);
  assert(!dest.empty());
  assert(src != nullptr);
  assert(ValidateUTF8(src));

  std::size_t copy = TruncateStringUTF8(src, truncate, dest.size() - 1);
  auto *p = std::copy_n(src, copy, dest.data());
  *p = '\0';
  return p;
}

std::pair<unsigned, const char *>
NextUTF8(const char *p) noexcept
{
  unsigned char a = *p++;
  if (a == 0)
    return std::make_pair(0u, nullptr);

  if (IsASCII(a))
    return std::make_pair(unsigned(a), p);

  assert(!IsContinuation(a));

  if (IsLeading1(a)) {
    /* 1 continuation */
    unsigned char b = *p++;
    assert(IsContinuation(b));

    return std::make_pair(((a & 0x1f) << 6) | (b & 0x3f), p);
  } else if (IsLeading2(a)) {
    /* 2 continuations */
    unsigned char b = *p++;
    assert(IsContinuation(b));
    unsigned char c = *p++;
    assert(IsContinuation(c));

    return std::make_pair(((a & 0xf) << 12) | ((b & 0x3f) << 6) | (c & 0x3f),
                          p);
  } else if (IsLeading3(a)) {
    /* 3 continuations */
    unsigned char b = *p++;
    assert(IsContinuation(b));
    unsigned char c = *p++;
    assert(IsContinuation(c));
    unsigned char d = *p++;
    assert(IsContinuation(d));

    return std::make_pair(((a & 0x7) << 18) | ((b & 0x3f) << 12)
                          | ((c & 0x3f) << 6) | (d & 0x3f),
                          p);
  } else if (IsLeading4(a)) {
    /* 4 continuations */
    unsigned char b = *p++;
    assert(IsContinuation(b));
    unsigned char c = *p++;
    assert(IsContinuation(c));
    unsigned char d = *p++;
    assert(IsContinuation(d));
    unsigned char e = *p++;
    assert(IsContinuation(e));

    return std::make_pair(((a & 0x3) << 24) | ((b & 0x3f) << 18)
                          | ((c & 0x3f) << 12) | ((d & 0x3f) << 6)
                          | (e & 0x3f),
                          p);
  } else if (IsLeading5(a)) {
    /* 5 continuations */
    unsigned char b = *p++;
    unsigned char c = *p++;
    unsigned char d = *p++;
    unsigned char e = *p++;
    unsigned char f = *p++;
    assert(IsContinuation(b));
    assert(IsContinuation(c));
    assert(IsContinuation(d));
    assert(IsContinuation(e));
    assert(IsContinuation(f));

    return std::make_pair(((a & 0x1) << 30) | ((b & 0x3f) << 24)
                          | ((c & 0x3f) << 18) | ((d & 0x3f) << 12)
                          | ((e & 0x3f) << 6) | (f & 0x3f),
                          p);
  } else {
    assert(false);
    gcc_unreachable();
  }
}

#ifdef _WIN32
#include <stringapiset.h>

std::wstring 
UTF8ToWide(const std::string_view s)
{
  int length = MultiByteToWideChar(CP_UTF8, 0, s.data(), s.size() + 1, nullptr, 0);
  std::wstring w(length + 1, L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.data(), s.size() + 1, w.data(), length);
  return w;
}

#endif
