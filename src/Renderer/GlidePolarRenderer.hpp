// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once


#include <cstddef>

struct PixelRect;
class Canvas;
struct ChartLook;
class ClimbHistory;
class GlidePolar;

void
GlidePolarCaption(char *buffer, const size_t length, const GlidePolar &glide_polar);

void
RenderGlidePolar(Canvas &canvas, const PixelRect rc,
                 const ChartLook &chart_look,
                 const ClimbHistory &climb_history,
                 const GlidePolar &glide_polar);
