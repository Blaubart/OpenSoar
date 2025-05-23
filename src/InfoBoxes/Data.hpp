// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "lib/fmt/ToBuffer.hxx"
#include "util/StaticString.hxx"
#include "Units/Unit.hpp"
#include "time/FloatDuration.hxx"

#if FMT_VERSION < 90000
#include <fmt/format.h> // for the fmt::buffer::flush() implementation
#endif

#include <chrono>
#include <cstdint>
// #include <string>

class Angle;

struct InfoBoxData {
  static constexpr unsigned COLOR_COUNT = 6;

  /**
   * If non-zero, then custom painting is enabled via
   * InfoBoxContent::OnCustomPaint().  The integer value can be used
   * to detect changes.
   */
  uint64_t custom;

  unsigned content_serial;

  StaticString<32> title;
  StaticString<32> value;
  StaticString<32> comment;

  Unit value_unit;

  uint8_t title_color, value_color, comment_color;

  void Clear() noexcept;

  /**
   * Enable custom painting via InfoBoxContent::OnCustomPaint().
   *
   * @param _custom a non-zero value which should change each time a
   * different content will be drawn
   */
  void SetCustom(uint64_t _custom) noexcept {
    custom = _custom;
    value.clear();
  }

  uint64_t GetCustom() const noexcept {
    return custom;
  }

  /**
   * Resets value to --- and unassigns the unit
   */
  void SetValueInvalid() noexcept;

  /**
   * Clears comment
   */
  void SetCommentInvalid() noexcept {
    comment.clear();
  }

  /**
   * calls SetValueInvalid() then SetCommentInvalid()
   */
  void SetInvalid() noexcept;

  /**
   * Sets the InfoBox title to the given Value
   *
   * @param title New value of the InfoBox title
   */
  void SetTitle(const char *title) noexcept;

  const char *GetTitle() const {
    return title;
  };

  void VFmtTitle(fmt::string_view format_str, fmt::format_args args) noexcept {
    auto [p, _] = fmt::vformat_to_n(title.begin(), title.capacity() - 1,
                                    format_str, args);
    *p = 0;
    title.CropIncompleteUTF8();
  }

  template<typename S, typename... Args>
  void FmtTitle(const S &format_str, Args&&... args) noexcept {
#if FMT_VERSION >= 90000
    return VFmtTitle(format_str,
                     fmt::make_format_args<fmt::format_context>(args...));
#else
    return VFmtTitle(fmt::to_string_view(format_str),
                     fmt::make_args_checked<Args...>(format_str, args...));
#endif
  }

  /**
   * Sets the InfoBox value to the given Value
   * @param Value New value of the InfoBox value
   */
  void SetValue(const char *value) noexcept;

  void VFmtValue(fmt::string_view format_str, fmt::format_args args) noexcept {
    auto [p, _] = fmt::vformat_to_n(value.begin(), value.capacity() - 1,
                                    format_str, args);
    *p = 0;
    value.CropIncompleteUTF8();
  }

  template<typename S, typename... Args>
  void FmtValue(const S &format_str, Args&&... args) noexcept {
#if FMT_VERSION >= 90000
    return VFmtValue(format_str,
                     fmt::make_format_args<fmt::format_context>(args...));
#else
    return VFmtValue(fmt::to_string_view(format_str),
                     fmt::make_args_checked<Args...>(format_str, args...));
#endif
  }

  /**
   * Sets the InfoBox value to the given angle.
   */
  void SetValue(Angle value, const char *suffix="") noexcept;

  void SetValueFromBearingDifference(Angle delta) noexcept;

  /**
   * Set the InfoBox value to the specified glide ratio.
   */
  void SetValueFromGlideRatio(double gr) noexcept;

  /**
   * Set the InfoBox value to the specified distance.
   */
  void SetValueFromDistance(double value) noexcept;

  /**
   * Set the InfoBox value to the specified altitude.
   */
  void SetValueFromAltitude(double value) noexcept;

  /**
   * Set the InfoBox value to the specified arrival altitude.
   */
  void SetValueFromArrival(double value) noexcept;

  /**
   * Set the InfoBox value to the specified horizontal speed.
   */
  void SetValueFromSpeed(double value, bool precision=true) noexcept;

  /**
   * Set the InfoBox value to the specified task speed.
   */
  void SetValueFromTaskSpeed(double value, bool precision=true) noexcept;

  /**
   * Set the InfoBox value to the specified percentage value.
   */
  void SetValueFromPercent(double value) noexcept;

  /**
   * Set the InfoBox value to the specified voltage value.
   */
  void SetValueFromVoltage(double value) noexcept;

  /**
   * Sets the InfoBox comment to the given Value
   * @param Value New value of the InfoBox comment
   */
  void SetComment(const char *comment) noexcept;

  void VFmtComment(fmt::string_view format_str, fmt::format_args args) noexcept {
    auto [p, _] = fmt::vformat_to_n(comment.begin(), comment.capacity() - 1,
                                    format_str, args);
    *p = 0;
    comment.CropIncompleteUTF8();
  }

  template<typename S, typename... Args>
  void FmtComment(const S &format_str, Args&&... args) noexcept {
#if FMT_VERSION >= 90000
    return VFmtComment(format_str,
                       fmt::make_format_args<fmt::format_context>(args...));
#else
    return VFmtComment(fmt::to_string_view(format_str),
                       fmt::make_args_checked<Args...>(format_str, args...));
#endif
  }

  /**
   * Sets the InfoBox comment to the given angle.
   */
  void SetComment(Angle comment, const char *suffix="") noexcept;

  void SetCommentFromDistance(double value) noexcept;

  void SetCommentFromBearingDifference(Angle delta) noexcept;

  /**
   * Set the InfoBox comment to the specified horizontal speed.
   */
  void SetCommentFromSpeed(double value, bool precision=true) noexcept;

  /**
   * Set the InfoBox comment to the specified task speed.
   */
  void SetCommentFromTaskSpeed(double value, bool precision=true) noexcept;

  /**
   * Set the InfoBox value to the specified altitude in the alternate
   * altitude unit.
   */
  void SetCommentFromAlternateAltitude(double value) noexcept;

  /**
   * Set the InfoBox comment value to the specified vertical speed.
   */
  void SetCommentFromVerticalSpeed(double value, bool include_sign=true) noexcept;

  /**
   * Set the InfoBox value to time HH:MM and SS
   */
  void SetValueFromTimeTwoLines(std::chrono::seconds dd) noexcept;

  void SetValueFromTimeTwoLines(FloatDuration dd) noexcept {
    SetValueFromTimeTwoLines(std::chrono::duration_cast<std::chrono::seconds>(dd));
  }

  /**
   * Set the InfoBox comment to the specified percentage value.
   */
  void SetCommentFromPercent(double value) noexcept;

  /**
   * Sets the unit of the InfoBox value
   *
   * @param Value New unit of the InfoBox value
   */
  void SetValueUnit(Unit _value_unit) noexcept {
    value_unit = _value_unit;
  }

  /**
   * Sets the color of the InfoBox value to the given value
   * @param value New color of the InfoBox value
   */
  void SetValueColor(unsigned _color) noexcept {
    assert(_color < COLOR_COUNT);

    value_color = _color;
  }

  /**
   * Sets the color of the InfoBox comment to the given value
   * @param value New color of the InfoBox comment
   */
  void SetCommentColor(unsigned _color) noexcept {
    assert(_color < COLOR_COUNT);

    comment_color = _color;
  }

  /**
   * Sets the color of the InfoBox title to the given value
   * @param value New color of the InfoBox title
   */
  void SetTitleColor(unsigned _color) noexcept {
    assert(_color < COLOR_COUNT);

    title_color = _color;
  }

  void SetAllColors(unsigned color) noexcept;

  bool CompareCustom(const InfoBoxData &other) const noexcept {
    return content_serial == other.content_serial &&
      custom == other.custom;
  }

  bool CompareTitle(const InfoBoxData &other) const noexcept;
  bool CompareValue(const InfoBoxData &other) const noexcept;
  bool CompareComment(const InfoBoxData &other) const noexcept;
};
