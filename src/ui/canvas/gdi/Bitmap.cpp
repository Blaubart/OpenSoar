// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "ui/canvas/Bitmap.hpp"
#include "ui/canvas/gdi/GdiPlusBitmap.hpp"
#include "ui/canvas/custom/UncompressedImage.hpp"

#include "Screen/Debug.hpp"
#include "system/Path.hpp"

#include <wingdi.h>
#include <winuser.h>

#include <cassert>
#include <utility>

Bitmap::Bitmap(Bitmap &&src) noexcept
  :bitmap(std::exchange(src.bitmap, nullptr))
{
}

Bitmap &Bitmap::operator=(Bitmap &&src) noexcept
{
  using std::swap;
  swap(bitmap, src.bitmap);
  return *this;
}

bool
Bitmap::LoadFile(Path path)
{
  bitmap = GdiLoadImage(path.c_str());
  return IsDefined();
}

bool 
Bitmap::Load(UncompressedImage &&uncompressed, [[maybe_unused]] Type type)
{
  Reset();

  bitmap = GdiLoadImage(std::move(uncompressed));

  return IsDefined();
}


void
Bitmap::Reset() noexcept
{
  if (bitmap != nullptr) {
    assert(IsScreenInitialized());

#ifndef NDEBUG
    bool success =
#endif
      ::DeleteObject(bitmap);
    assert(success);

    bitmap = nullptr;
  }
}

PixelSize
Bitmap::GetSize() const noexcept
{
#if defined(_DEBUG)
  if (!IsDefined())
    return { 0, 0 };
#endif
  assert(IsDefined());

  BITMAP bm;
  ::GetObject(bitmap, sizeof(bm), &bm);
  const PixelSize size = { bm.bmWidth, bm.bmHeight };
  return size;
}
