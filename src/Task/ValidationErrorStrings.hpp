// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Engine/Task/Factory/ValidationError.hpp"


[[gnu::const]]
const char *
getTaskValidationErrors(const TaskValidationErrorSet v);
