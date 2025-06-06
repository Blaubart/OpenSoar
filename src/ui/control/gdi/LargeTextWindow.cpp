// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "../LargeTextWindow.hpp"

#include <memory>

#include <commctrl.h>
#include <windowsx.h>

void
LargeTextWindow::Create(ContainerWindow &parent, PixelRect rc,
                        const LargeTextWindowStyle style)
{
  Window::Create(&parent, WC_EDIT, nullptr, rc, style);
}

void
LargeTextWindow::SetText(const char *text)
{
  // Replace \n by \r\r\n to enable usage of line-breaks in edit control
  unsigned size = strlen(text);

  const std::unique_ptr<char[]> allocation(new char[size * 3 + 1]);
  char *buffer = allocation.get();

  const char* p2 = text;
  char* p3 = buffer;
  for (; *p2 != '\0'; p2++) {
    if (*p2 == '\n') {
      *p3 = '\r';
      p3++;
      *p3 = '\r';
      p3++;
      *p3 = '\n';
    } else if (*p2 == '\r') {
      continue;
    } else {
      *p3 = *p2;
    }
    p3++;
  }
  *p3 = '\0';

  ::SetWindowText(hWnd, buffer);
}

void
LargeTextWindow::ScrollVertically(int delta_lines)
{
  Edit_Scroll(*this, delta_lines, 0);
}
