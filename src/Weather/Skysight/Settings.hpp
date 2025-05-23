// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "util/StaticString.hxx"

struct SkysightSettings {
  StaticString<64> email;
  StaticString<64> password;
  StaticString<64> region;

  bool IsDefined() const {
    return !email.empty() && !password.empty();
  }

  void SetDefaults() {
    email.clear();
    password.clear();
    region.clear();
  }

};
