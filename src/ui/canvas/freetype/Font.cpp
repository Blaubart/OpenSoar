// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "ui/canvas/Font.hpp"
#include "Screen/Debug.hpp"
#include "ui/canvas/custom/Files.hpp"
#include "Look/FontDescription.hpp"
#include "Init.hpp"
#include "Asset.hpp"
#include "lib/fmt/RuntimeError.hxx"
#include "system/Path.hpp"

#ifndef ENABLE_OPENGL
#include "thread/Mutex.hxx"
#endif

#include "util/UTF8.hpp"

#if defined(__clang__) && defined(__arm__)
/* work around warning: 'register' storage class specifier is
   deprecated */
#define register
#endif

#include <ft2build.h>
#include FT_FREETYPE_H

#include <algorithm>

#include <cassert>
#include <concepts>
#include <cstdint>

#ifndef ENABLE_OPENGL
/**
 * libfreetype is not thread-safe; this global Mutex is used to
 * protect libfreetype from multi-threaded access.
 */
static Mutex freetype_mutex;
#endif

static FT_Int32 load_flags = FT_LOAD_DEFAULT;
static FT_Render_Mode render_mode = FT_RENDER_MODE_NORMAL;

static AllocatedPath font_path;
static AllocatedPath bold_font_path;
static AllocatedPath italic_font_path;
static AllocatedPath bold_italic_font_path;
static AllocatedPath monospace_font_path;

[[gnu::const]]
static inline bool
IsMono() noexcept
{
#ifdef KOBO
  /* on the Kobo, "mono" mode can be set at runtime; the shutdown
     screen renders in greyscale mode */
  return FreeType::mono;
#else
  return IsDithered();
#endif
}

static constexpr FT_Long
FT_FLOOR(FT_Long x) noexcept
{
  return (x & -64) / 64;
}

static constexpr FT_Long
FT_CEIL(FT_Long x) noexcept
{
  return FT_FLOOR(x + 63);
}

static unsigned
NextChar(std::string_view &s) noexcept
{
  assert(!s.empty());

  auto n = NextUTF8(s.data());
  s.remove_prefix(n.second - s.data());
  return n.first;
}

void
Font::Initialise()
{
  FreeType::Initialise();

  if (IsMono()) {
    /* disable anti-aliasing */
    load_flags |= FT_LOAD_TARGET_MONO;
    render_mode = FT_RENDER_MODE_MONO;
  }

  font_path = FindDefaultFont();
  bold_font_path = FindDefaultBoldFont();
  italic_font_path = FindDefaultItalicFont();
  bold_italic_font_path = FindDefaultBoldItalicFont();
  monospace_font_path = FindDefaultMonospaceFont();
}

void
Font::Deinitialise() noexcept
{
  FreeType::Deinitialise();
}

[[gnu::pure]]
static unsigned
GetCapitalHeight(FT_Face face) noexcept
{
#ifndef ENABLE_OPENGL
  const std::lock_guard lock{freetype_mutex};
#endif

  FT_UInt i = FT_Get_Char_Index(face, 'M');
  if (i == 0)
    return 0;

  FT_Error error = FT_Load_Glyph(face, i, load_flags);
  if (error)
    return 0;

  return FT_CEIL(face->glyph->metrics.height);
}

void
Font::LoadFile(const char *file, unsigned ptsize, [[maybe_unused]] bool bold, [[maybe_unused]] bool italic)
{
  assert(IsScreenInitialized());

  Destroy();

  FT_Face new_face = FreeType::Load(file);

  FT_Error error = ::FT_Set_Pixel_Sizes(new_face, 0, ptsize);
  if (error) {
    ::FT_Done_Face(new_face);
    throw FmtRuntimeError("Failed to initialise font {}: {}",
                          file, FT_Error_String(error));
  }

  const FT_Fixed y_scale = new_face->size->metrics.y_scale;

  height = FT_CEIL(FT_MulFix(new_face->height, y_scale));
  ascent_height = FT_CEIL(FT_MulFix(new_face->ascender, y_scale));

  capital_height = ::GetCapitalHeight(new_face);
  if (capital_height == 0)
    capital_height = height;

  // TODO: handle bold/italic

  face = new_face;
}

void
Font::Load(const FontDescription &d)
{
  assert(IsScreenInitialized());

  bool bold = d.IsBold();
  bool italic = d.IsItalic();
  Path path = nullptr;

  /* check for presence of "real" font and clear the bold or italic
   * flags if found so that freetype does not apply them again to
   * produce a "synthetic" bold or italic version of the font */
  if (italic && bold && bold_italic_font_path != nullptr) {
    path = bold_italic_font_path;
    bold = false;
    italic = false;
  } else if (italic && italic_font_path != nullptr) {
    path = italic_font_path;
    italic = false;
  } else if (d.IsMonospace() && monospace_font_path != nullptr) {
    path = monospace_font_path;
  } else if (bold && bold_font_path != nullptr) {
    path = bold_font_path;
    bold = false;
  } else {
    path = font_path;
  }

  assert(path != nullptr);

  LoadFile(path.c_str(), d.GetHeight(), bold, italic);
}

void
Font::Destroy() noexcept
{
  if (!IsDefined())
    return;

  assert(IsScreenInitialized());

  ::FT_Done_Face(face);
  face = nullptr;
}

static void
ForEachChar(std::string_view text, std::invocable<unsigned> auto f)
{
  assert(ValidateUTF8(text));

  while (!text.empty()) {
    const unsigned ch = NextChar(text);
    f(ch);
  }
}

template<typename T>
static void
ForEachGlyph(const FT_Face face, unsigned ascent_height, T &&text,
             std::invocable<int, int, FT_GlyphSlot> auto f) noexcept
{
  const bool use_kerning = FT_HAS_KERNING(face);

  int x = 0;
  unsigned prev_index = 0;

#ifndef ENABLE_OPENGL
  const std::lock_guard lock{freetype_mutex};
#endif

  ForEachChar(std::forward<T>(text),
              [face, ascent_height, &f, use_kerning,
               &x, &prev_index](unsigned ch){
      const FT_UInt i = FT_Get_Char_Index(face, ch);
      if (i == 0)
        return;

      const FT_Error error = FT_Load_Glyph(face, i, load_flags);
      if (error)
        return;

      const FT_GlyphSlot glyph = face->glyph;
      const FT_Glyph_Metrics &metrics = glyph->metrics;

      if (use_kerning) {
        if (prev_index != 0 && i != 0) {
          FT_Vector delta;
          FT_Get_Kerning(face, prev_index, i, ft_kerning_default,
                         &delta);
          x += delta.x >> 6;
        }

        prev_index = i;
      }

      f(x + FT_FLOOR(metrics.horiBearingX),
        ascent_height - FT_FLOOR(metrics.horiBearingY),
        glyph);

      x += FT_CEIL(metrics.horiAdvance);
    });
}

PixelSize
Font::TextSize(std::string_view text) const noexcept
{
  int maxx = 0;

  ForEachGlyph(face, ascent_height, text,
               [&maxx](int x, [[maybe_unused]] int y, const FT_GlyphSlot glyph){
      const FT_Glyph_Metrics &metrics = glyph->metrics;
      const int glyph_minx = FT_FLOOR(metrics.horiBearingX);
      const int glyph_maxx = glyph_minx + FT_CEIL(metrics.width);

      int z = x + glyph_maxx;
      if (z > maxx)
        maxx = z;
    });

  return PixelSize{unsigned(maxx), height};
}

static void
MixLine(uint8_t *dest, const uint8_t *src, size_t n) noexcept
{
  for (size_t i = 0; i != n; ++i)
    /* bit-wise "OR" should be good enough for kerning */
    dest[i] = dest[i] | src[i];
}

static void
RenderGlyph(uint8_t *buffer, unsigned buffer_width, unsigned buffer_height,
            const FT_Bitmap &bitmap, int x, int y) noexcept
{
  const uint8_t *src = (const uint8_t *)bitmap.buffer;
  int width = bitmap.width, height = bitmap.rows;
  int pitch = bitmap.pitch;

  if (x < 0) {
    src -= x;
    width += x;
    x = 0;
  }

  if (unsigned(x) >= buffer_width || width <= 0)
    return;

  if (unsigned(x + width) > buffer_width)
    width = buffer_width - x;

  if (y < 0) {
    src -= y * pitch;
    height += y;
    y = 0;
  }

  if (unsigned(y) >= buffer_height || height <= 0)
    return;

  if (unsigned(y + height) > buffer_height)
    height = buffer_height - y;

  buffer += unsigned(y) * buffer_width + unsigned(x);
  for (const uint8_t *end = src + height * pitch;
       src != end; src += pitch, buffer += buffer_width)
    MixLine(buffer, src, width);
}

static void
ConvertMono(unsigned char *dest, const unsigned char *src, unsigned n) noexcept
{
  for (; n >= 8; n -= 8, ++src) {
    for (unsigned i = 0x80; i != 0; i >>= 1)
      *dest++ = (*src & i) ? 0xff : 0x00;
  }

  for (unsigned i = 0x80; n > 0; i >>= 1, --n)
    *dest++ = (*src & i) ? 0xff : 0x00;
}

static void
ConvertMono(FT_Bitmap &dest, const FT_Bitmap &src) noexcept
{
  dest = src;
  dest.pitch = dest.width;
  dest.buffer = new unsigned char[dest.pitch * dest.rows];

  unsigned char *d = dest.buffer, *s = src.buffer;
  for (unsigned y = 0; y < unsigned(dest.rows);
       ++y, d += dest.pitch, s += src.pitch)
    ConvertMono(d, s, dest.width);
}

static void
RenderGlyph(uint8_t *buffer, size_t width, size_t height,
            FT_GlyphSlot glyph, int x, int y) noexcept
{
  FT_Error error = FT_Render_Glyph(glyph, render_mode);
  if (error)
    return;

  if (IsMono()) {
    /* with anti-aliasing disabled, FreeType writes each pixel in one
       bit; hack: convert it to 1 byte per pixel and then render it */
    FT_Bitmap bitmap;
    ConvertMono(bitmap, glyph->bitmap);
    RenderGlyph(buffer, width, height, bitmap, x, y);
    delete[] bitmap.buffer;
  } else
    RenderGlyph(buffer, width, height, glyph->bitmap, x, y);
}

void
Font::Render(std::string_view text, const PixelSize size,
             void *_buffer) const noexcept
{
  uint8_t *buffer = (uint8_t *)_buffer;
  std::fill_n(buffer, BufferSize(size), 0);

  ForEachGlyph(face, ascent_height, text,
               [size, buffer](int x, int y, const FT_GlyphSlot glyph){
      RenderGlyph(buffer, size.width, size.height, glyph,
                  x, y);
    });
}
