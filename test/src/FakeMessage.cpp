// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "Message.hpp"

#include <stdio.h>

void
Message::AddMessage(const char *text,
                    [[maybe_unused]] const char *data) noexcept
{
  fprintf(stderr, "%s\n", text);
}
