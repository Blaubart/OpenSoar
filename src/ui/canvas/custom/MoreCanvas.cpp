// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "ui/canvas/Canvas.hpp"
#include "util/StringAPI.hxx"

#ifndef NDEBUG
#include "util/UTF8.hpp"
#endif

#include <algorithm>
#include <cassert>

#include <limits.h>
#include <string.h>
#include <winuser.h>

void
Canvas::DrawRaisedEdge(PixelRect &rc) noexcept
{
  Pen bright(1, Color(240, 240, 240));
  Select(bright);
  DrawTwoLinesExact({rc.left, rc.bottom - 2},
                    {rc.left, rc.top},
                    {rc.right - 1, rc.top});

  Pen dark(1, Color(128, 128, 128));
  Select(dark);
  DrawTwoLinesExact({rc.left, rc.bottom - 1},
                    {rc.right - 1, rc.bottom - 1},
                    {rc.right - 1, rc.top + 1});

  ++rc.left;
  ++rc.top;
  --rc.right;
  --rc.bottom;
}

unsigned
Canvas::DrawFormattedText(const PixelRect r, const std::string_view text,
                          const unsigned format) noexcept
{
  assert(ValidateUTF8(text));
  if (font == nullptr)
    return 0;

  const unsigned skip = font->GetLineSpacing();
  const unsigned max_lines = (format & DT_CALCRECT)
    ? UINT_MAX
    : (r.GetHeight() + skip - 1) / skip;

  char *const duplicated = new char[text.size() + 1], *p = duplicated;
  unsigned lines = 1;
  for (char ch : text) {
    if (ch == '\n') {
      /* explicit line break */

      if (++lines > max_lines)
        break;

      ch = '\0';
    } else if (ch == '\r')
      /* skip */
      continue;
    else if ((unsigned)ch < 0x20)
      /* replace non-printable characters */
      ch = ' ';

    *p++ = ch;
  }

  *p = '\0';
  const size_t len = p - duplicated;

  // simple wordbreak algorithm. looks for single spaces only, no tabs,
  // no grouping of multiple spaces
  for (size_t i = 0; i < len; i += strlen(duplicated + i) + 1) {
    PixelSize sz = CalcTextSize(duplicated + i);
    char *prev_p = nullptr;

    // remove words from behind till line fits or no more space is found
    while (sz.width > r.GetWidth() &&
           (p = StringFindLast(duplicated + i, ' ')) != nullptr) {
      if (prev_p)
        *prev_p = ' ';
      *p = '\0';
      prev_p = p;
      sz = CalcTextSize(duplicated + i);
    }

    if (prev_p) {
      lines++;
      if (lines >= max_lines)
        break;
    }
  }

  if (format & DT_CALCRECT) {
    delete[] duplicated;
    return lines * skip;
  }

  int y = (format & DT_VCENTER) && lines < max_lines
    ? (r.top + r.bottom - lines * skip) / 2
    : r.top;
  for (size_t i = 0; i < len; i += strlen(duplicated + i) + 1) {
    if (duplicated[i] != '\0') {
      int x;
      if (format & (DT_RIGHT | DT_CENTER)) {
        PixelSize sz = CalcTextSize(duplicated + i);
        x = (format & DT_CENTER)
          ? (r.left + r.right - (int)sz.width) / 2
          : r.right - (int)sz.width;  // DT_RIGHT
      } else {  // default is DT_LEFT
        x = r.left;
      }

      TextAutoClipped({x, y}, duplicated + i);

      if (format & DT_UNDERLINE)
        DrawHLine(x, x + CalcTextWidth(duplicated + i),
                  y + font->GetAscentHeight() + 1, text_color);
    }
    y += skip;
    if (y >= r.bottom)
      break;
  }

  delete[] duplicated;
  return lines * skip;
}

void
Canvas::DrawOpaqueText(PixelPoint p, const PixelRect &rc,
                       std::string_view text) noexcept
{
  DrawFilledRectangle(rc, background_color);
  DrawTransparentText(p, text);
}
