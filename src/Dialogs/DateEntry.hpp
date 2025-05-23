// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once


struct BrokenDate;

bool
DateEntryDialog(const char *caption, BrokenDate &value,
                bool nullable=false);
