// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "FlarmTrafficWindow.hpp"
#include "FLARM/Traffic.hpp"
#include "FLARM/Friends.hpp"
#include "ui/canvas/Canvas.hpp"
#include "Screen/Layout.hpp"
#include "Formatter/UserUnits.hpp"
#include "Units/Units.hpp"
#include "Math/Screen.hpp"
#include "Language/Language.hpp"
#include "util/Macros.hpp"
#include "Look/FlarmTrafficLook.hpp"
#include "Renderer/TextInBox.hpp"

#include <algorithm>

#include <cassert>
#include <stdio.h>

#ifdef ENABLE_OPENGL
#include "ui/canvas/opengl/Scope.hpp"
#endif


FlarmTrafficWindow::FlarmTrafficWindow(const FlarmTrafficLook &_look,
                                       unsigned _h_padding,
                                       unsigned _v_padding,
                                       bool _small) noexcept
  :look(_look),
   radar_renderer(_h_padding, _v_padding),
   small(_small)
{
  data.Clear();
}

bool
FlarmTrafficWindow::WarningMode() const noexcept
{
  assert(warning < (int)data.list.size());
  assert(warning < 0 || data.list[warning].IsDefined());
  assert(warning < 0 || data.list[warning].HasAlarm());

  return warning >= 0;
}

void
FlarmTrafficWindow::OnResize(PixelSize new_size) noexcept
{
  PaintWindow::OnResize(new_size);

  radar_renderer.UpdateLayout(PixelRect{new_size});
}

void
FlarmTrafficWindow::SetTarget(int i) noexcept
{
  assert(i < (int)data.list.size());
  assert(i < 0 || data.list[i].IsDefined());

  if (selection == i)
    return;

  selection = i;
  Invalidate();
}

/**
 * Tries to select the next target, if impossible selection = -1
 */
void
FlarmTrafficWindow::NextTarget() noexcept
{
  // If warning is displayed -> prevent selector movement
  if (WarningMode())
    return;

  assert(selection < (int)data.list.size());

  const FlarmTraffic *traffic;
  if (selection >= 0)
    traffic = data.NextTraffic(&data.list[selection]);
  else
    traffic = NULL;

  if (traffic == NULL)
    traffic = data.FirstTraffic();

  SetTarget(traffic);
}

/**
 * Tries to select the previous target, if impossible selection = -1
 */
void
FlarmTrafficWindow::PrevTarget() noexcept
{
  // If warning is displayed -> prevent selector movement
  if (WarningMode())
    return;

  assert(selection < (int)data.list.size());

  const FlarmTraffic *traffic;
  if (selection >= 0)
    traffic = data.PreviousTraffic(&data.list[selection]);
  else
    traffic = NULL;

  if (traffic == NULL)
    traffic = data.LastTraffic();

  SetTarget(traffic);
}

/**
 * Checks whether the selection is still on the valid target and if not tries
 * to select the next one
 */
void
FlarmTrafficWindow::UpdateSelector(const FlarmId id,
                                   const PixelPoint pt) noexcept
{
  // Update #selection
  if (!id.IsDefined())
    SetTarget(-1);
  else
    SetTarget(id);

  // If we don't have a valid selection and we can't find
  // a target close to to the PixelPoint we select the next one
  // on the internal list
  if (selection < 0 && (
      pt.x < 0 || pt.y < 0 ||
      !SelectNearTarget(pt, radar_renderer.GetRadius() * 2)) )
    NextTarget();
}

/**
 * Iterates through the traffic array, finds the target with the highest
 * alarm level and saves it to "warning".
 */
void
FlarmTrafficWindow::UpdateWarnings() noexcept
{
  const FlarmTraffic *alert = data.FindMaximumAlert();
  warning = alert != NULL
    ? (int)data.TrafficIndex(alert)
    : - 1;
}

/**
 * This should be called when the radar needs to be repainted
 */
void
FlarmTrafficWindow::Update(Angle new_direction, const TrafficList &new_data,
                           const TeamCodeSettings &new_settings) noexcept
{
  static constexpr Angle min_heading_delta = Angle::Degrees(2);
  if (new_data.modified == data.modified &&
      (heading - new_direction).Absolute() < min_heading_delta)
    /* no change - don't redraw */
    return;

  FlarmId selection_id;
  PixelPoint pt;
  if (!small && selection >= 0) {
    selection_id = data.list[selection].id;
    pt = sc[selection];
  } else {
    selection_id.Clear();
    pt.x = -100;
    pt.y = -100;
  }

  heading = new_direction;
  fr = -heading;
  fir = heading;
  data = new_data;
  settings = new_settings;

  UpdateWarnings();
  UpdateSelector(selection_id, pt);

  Invalidate();
}

/**
 * Returns the distance to the own plane in pixels
 * @param d Distance in meters to the own plane
 */
double
FlarmTrafficWindow::RangeScale(double d) const noexcept
{
  d /= distance;
  return std::min(d, 1.) * radar_renderer.GetRadius();
}

/**
 * Paints a "No Traffic" sign on the given canvas
 * @param canvas The canvas to paint on
 */
void
FlarmTrafficWindow::PaintRadarNoTraffic(Canvas &canvas) const noexcept
{
  if (small)
    return;

  const char* str = _("No Traffic");
  canvas.Select(look.no_traffic_font);
  PixelSize ts = canvas.CalcTextSize(str);
  canvas.SetTextColor(look.default_color);
  canvas.DrawText(
      radar_renderer.GetCenter() -
          PixelSize{ts.width / 2, radar_renderer.GetRadius() -
                                      radar_renderer.GetRadius() / 4},
      str);
}

[[gnu::const]]
static const Pen *
FlarmColorPen(const FlarmTrafficLook &look, FlarmColor color) noexcept
{
  switch (color) {
  case FlarmColor::NONE:
    break;

  case FlarmColor::GREEN:
    return &look.team_pen_green;

  case FlarmColor::BLUE:
    return &look.team_pen_blue;

  case FlarmColor::YELLOW:
    return &look.team_pen_yellow;

  case FlarmColor::MAGENTA:
    return &look.team_pen_magenta;

  case FlarmColor::COUNT:
    assert(false);
    gcc_unreachable();
  }

  return nullptr;
}

/**
 * Paints the traffic symbols on the given canvas
 * @param canvas The canvas to paint on
 */
void
FlarmTrafficWindow::PaintRadarTarget(Canvas &canvas,
                                     const FlarmTraffic &traffic,
                                     unsigned i) noexcept
{
  // Save relative East/North
  DoublePoint2D p(traffic.relative_east, -traffic.relative_north);

  // Calculate the distance in pixels
  double scale = RangeScale(traffic.distance);

  // Don't display distracting, far away targets in WarningMode
  if (WarningMode() && !traffic.HasAlarm() && scale == radar_renderer.GetRadius())
    return;

  // x and y are not between 0 and 1 (distance will be handled via scale)
  if (!traffic.distance.IsZero()) {
    p.x /= traffic.distance;
    p.y /= traffic.distance;
  } else {
    p.x = 0;
    p.y = 0;
  }

  if (!enable_north_up) {
    // Rotate x and y to have a track up display
    p = fr.Rotate(p);
  }

  // Calculate screen coordinates
  const auto radar_mid = radar_renderer.GetCenter();
  sc[i].x = radar_mid.x + iround(p.x * scale);
  sc[i].y = radar_mid.y + iround(p.y * scale);

  const Color *text_color;
  const Pen *target_pen, *circle_pen;
  const Brush *target_brush = nullptr, *arrow_brush;
  unsigned circles = 0;

  // Set the arrow color depending on alarm level
  switch (traffic.alarm_level) {
  case FlarmTraffic::AlarmType::LOW:
  case FlarmTraffic::AlarmType::INFO_ALERT:
    text_color = &look.default_color;
    target_pen = circle_pen = &look.warning_pen;
    target_brush = &look.warning_brush;
    arrow_brush = &look.default_brush;
    circles = 1;
    break;
  case FlarmTraffic::AlarmType::IMPORTANT:
  case FlarmTraffic::AlarmType::URGENT:
    text_color = &look.default_color;
    target_pen = circle_pen = &look.alarm_pen;
    target_brush = &look.alarm_brush;
    arrow_brush = &look.default_brush;
    circles = 2;
    break;
  case FlarmTraffic::AlarmType::NONE:
  default:
    if (WarningMode()) {
      text_color = &look.passive_color;
      target_pen = &look.passive_pen;
      arrow_brush = &look.passive_brush;
    } else {
      // Search for team color
      const FlarmColor team_color = FlarmFriends::GetFriendColor(traffic.id);

      // If team color found -> draw a colored circle around the target
      if (team_color != FlarmColor::NONE) {
        circle_pen = FlarmColorPen(look, team_color);
        circles = 1;
      } else {
        // unnecessary - prevents "may be used uninitialized" compiler warning
        circle_pen = &look.default_pen;
      }
      // same colours of FLARM targets as in map display
      text_color = &look.default_color;
      target_pen = &look.radar_pen;
      arrow_brush = &look.default_brush;

      if (traffic.relative_altitude > (const RoughAltitude)50) {
        target_brush = &look.safe_above_brush;
      } else if (traffic.relative_altitude > (const RoughAltitude)-50) {
        target_brush = &look.warning_in_altitude_range_brush;
      } else {
        target_brush = &look.safe_below_brush;
      }

      if (!small && static_cast<unsigned> (selection) == i)
        target_pen = &look.default_pen;
    }
    break;
  }

  if (circles > 0) {
    canvas.SelectHollowBrush();
    canvas.Select(*circle_pen);
    canvas.DrawCircle(sc[i], Layout::FastScale(small ? 8 : 16));
    if (circles == 2)
      canvas.DrawCircle(sc[i], Layout::FastScale(small ? 10 : 19));
  }

  // Create an arrow polygon
  BulkPixelPoint Arrow[4];
  if (small) {
    Arrow[0].x = -3;
    Arrow[0].y = 4;
    Arrow[1].x = 0;
    Arrow[1].y = -5;
    Arrow[2].x = 3;
    Arrow[2].y = 4;
    Arrow[3].x = 0;
    Arrow[3].y = 2;
  } else {
    Arrow[0].x = -6;
    Arrow[0].y = 8;
    Arrow[1].x = 0;
    Arrow[1].y = -10;
    Arrow[2].x = 6;
    Arrow[2].y = 8;
    Arrow[3].x = 0;
    Arrow[3].y = 5;
  }

  // Rotate and shift the arrow
  PolygonRotateShift(Arrow, sc[i],
                     traffic.track - (enable_north_up ?
                                      Angle::Zero() : heading),
                     Layout::Scale(100u));

  // Select pen and brush
  canvas.Select(*target_pen);
  if (target_brush == nullptr)
    canvas.SelectHollowBrush();
  else
    canvas.Select(*target_brush);

  // Draw the polygon
  canvas.DrawPolygon(Arrow, 4);

  if (small) {
    if (!WarningMode() || traffic.HasAlarm())
      PaintTargetInfoSmall(canvas, traffic, i, *text_color, *arrow_brush);

    return;
  }

  // if warning exists -> don't draw vertical speeds
  if (WarningMode())
    return;

  // if vertical speed to small or negative -> skip this one
  if (side_display_type == SideInfoType::VARIO &&
      (!traffic.climb_rate_avg30s_available ||
       traffic.climb_rate_avg30s < 0.5 ||
       traffic.IsPowered()))
      return;

  // Select font
  canvas.SetBackgroundTransparent();
  canvas.Select(look.side_info_font);

  // Format string
  char tmp[10];

  if (side_display_type == SideInfoType::VARIO)
    FormatUserVerticalSpeed(traffic.climb_rate_avg30s, tmp, false);
  else
    FormatRelativeUserAltitude(traffic.relative_altitude, tmp, true);

  PixelSize sz = canvas.CalcTextSize(tmp);
  const PixelPoint tp{
    sc[i].x + int(Layout::FastScale(11u)),
    sc[i].y - int(sz.height / 2),
  };

  // Select color
  canvas.SetTextColor(*text_color);

  // Draw vertical speed
  canvas.DrawText(tp, tmp);
}

void
FlarmTrafficWindow::PaintTargetInfoSmall(Canvas &canvas,
                                         const FlarmTraffic &traffic,
                                         unsigned i,
                                         const Color &text_color,
                                         const Brush &arrow_brush) noexcept
{
  const short relalt =
      iround(Units::ToUserAltitude(traffic.relative_altitude) / 100);

  // if (relative altitude is other than zero)
  if (relalt == 0)
    return;

  // Write the relativ altitude devided by 100 to the Buffer
  StaticString<10> buffer;
  const auto relalt_s = buffer.Format("%d", abs(relalt));

  // Select font
  canvas.SetBackgroundTransparent();
  canvas.Select(look.side_info_font);
  canvas.SetTextColor(text_color);

  // Calculate size of the output string
  PixelSize tsize = canvas.CalcTextSize(relalt_s);

  unsigned dist = Layout::FastScale(traffic.HasAlarm() ? 12 : 8);

  // Draw string
  canvas.DrawText(sc[i].At(dist, -int(tsize.height / 2)), relalt_s);

  // Set target_brush for the up/down arrow
  canvas.Select(arrow_brush);
  canvas.SelectNullPen();

  // Prepare the triangular polygon
  BulkPixelPoint triangle[4];
  triangle[0].x = 0;
  triangle[0].y = -4;
  triangle[1].x = 3;
  triangle[1].y = 0;
  triangle[2].x = -3;
  triangle[2].y = 0;

  // Flip = -1 for arrow pointing downwards
  short flip = 1;
  if (relalt < 0)
    flip = -1;

  // Shift the arrow to the right position
  for (int j = 0; j < 3; j++) {
    triangle[j].x = Layout::FastScale(triangle[j].x);
    triangle[j].y = Layout::FastScale(triangle[j].y);

    triangle[j] = sc[i].At(dist + triangle[j].x + int(tsize.width / 2),
                           flip * (triangle[j].y - int(tsize.height / 2)));
  }
  triangle[3].x = triangle[0].x;
  triangle[3].y = triangle[0].y;

  // Draw the arrow
  canvas.DrawTriangleFan(triangle, 4);

}

/**
 * Paints the traffic symbols on the given canvas
 * @param canvas The canvas to paint on
 */
void
FlarmTrafficWindow::PaintRadarTraffic(Canvas &canvas) noexcept
{
  if (data.IsEmpty()) {
    PaintRadarNoTraffic(canvas);
    return;
  }

  // Iterate through the traffic (normal traffic)
  for (unsigned i = 0; i < data.list.size(); ++i) {
    const FlarmTraffic &traffic = data.list[i];

    if (!traffic.HasAlarm() &&
        static_cast<unsigned> (selection) != i)
      PaintRadarTarget(canvas, traffic, i);
  }

  if (selection >= 0) {
    const FlarmTraffic &traffic = data.list[selection];

    if (!traffic.HasAlarm())
      PaintRadarTarget(canvas, traffic, selection);
  }

  if (!WarningMode())
    return;

  // Iterate through the traffic (alarm traffic)
  for (unsigned i = 0; i < data.list.size(); ++i) {
    const FlarmTraffic &traffic = data.list[i];

    if (traffic.HasAlarm())
      PaintRadarTarget(canvas, traffic, i);
  }
}

/**
 * Paint a plane symbol in the middle of the radar on the given canvas
 * @param canvas The canvas to paint on
 */
void
FlarmTrafficWindow::PaintRadarPlane(Canvas &canvas) const noexcept
{
  const auto radar_mid = radar_renderer.GetCenter();

  canvas.Select(look.plane_pen);

  PixelPoint p1(Layout::FastScale(small ? 5 : 10),
                -Layout::FastScale(small ? 1 : 2));
  PixelPoint p2(-p1.x, p1.y);

  if (enable_north_up) {
    p1 = fir.Rotate(p1);
    p2 = fir.Rotate(p2);
  }

  canvas.DrawLine(radar_mid + p1, radar_mid + p2);

  p2 = { 0, Layout::FastScale(small ? 3 : 6) };
  p1 = { 0, -p2.y };

  if (enable_north_up) {
    p1 = fir.Rotate(p1);
    p2 = fir.Rotate(p2);
  }

  canvas.DrawLine(radar_mid + p1, radar_mid + p2);

  p1.x = Layout::FastScale(small ? 2 : 4);
  p1.y = p1.x;
  p2 = { -p1.x, p1.y };

  if (enable_north_up) {
    p1 = fir.Rotate(p1);
    p2 = fir.Rotate(p2);
  }

  canvas.DrawLine(radar_mid + p1, radar_mid + p2);
}

[[gnu::const]]
static PixelPoint
iround(DoublePoint2D p) noexcept
{
  return {iround(p.x), iround(p.y)};
}

/**
 * Paints the radar circle on the given canvas
 * @param canvas The canvas to paint on
 */
void
FlarmTrafficWindow::PaintNorth(Canvas &canvas) const noexcept
{
  DoublePoint2D p(0, -1);
  if (!enable_north_up) {
    p = fr.Rotate(p);
  }

  canvas.SetTextColor(look.background_color);
  canvas.Select(look.radar_pen);
  canvas.Select(look.radar_brush);
  canvas.SetBackgroundTransparent();
  canvas.Select(look.label_font);

  const auto radar_mid = radar_renderer.GetCenter();
  const PixelPoint q = radar_mid + iround(p * radar_renderer.GetRadius());

  PixelSize s = canvas.CalcTextSize("N");
  canvas.DrawCircle(q, s.height * 0.65);
  canvas.DrawText(q - s / 2u, "N");
}

static void
DrawCircleLabel(Canvas &canvas, PixelPoint p,
                std::string_view text) noexcept
{
  const auto size = canvas.CalcTextSize(text);
  p.x -= size.width / 2;
  p.y -= size.height * 3 / 4;

  canvas.DrawText(p, text);
}

/**
 * Paints the radar circle on the given canvas
 * @param canvas The canvas to paint on
 */
void
FlarmTrafficWindow::PaintRadarBackground(Canvas &canvas) const noexcept
{
  canvas.SelectHollowBrush();
  canvas.Select(look.radar_pen);
  canvas.SetTextColor(look.radar_color);

  // Paint circles
  const unsigned radius = radar_renderer.GetRadius();
  radar_renderer.DrawCircle(canvas, radius);
  radar_renderer.DrawCircle(canvas, radius / 2);

  PaintRadarPlane(canvas);

  if (small)
    return;

  // Paint zoom strings
  canvas.Select(look.label_font);
  canvas.SetBackgroundOpaque();
  canvas.SetBackgroundColor(look.background_color);

  const auto radar_mid = radar_renderer.GetCenter();

  DrawCircleLabel(canvas, radar_mid + PixelSize{0u, radius},
                  FormatUserDistanceSmart(distance, true, 1000).c_str());

  DrawCircleLabel(canvas, radar_mid + PixelSize{0u, radius / 2},
                  FormatUserDistanceSmart(distance / 2, true, 1000).c_str());

  canvas.SetBackgroundTransparent();

  PaintNorth(canvas);
}

/**
 * This function paints the TrafficRadar onto the given canvas
 * @param canvas The canvas to paint on
 */
void
FlarmTrafficWindow::Paint(Canvas &canvas) noexcept
{
  assert(selection < (int)data.list.size());
  assert(selection < 0 || data.list[selection].IsDefined());
  assert(warning < (int)data.list.size());
  assert(warning < 0 || data.list[warning].IsDefined());
  assert(warning < 0 || data.list[warning].HasAlarm());

  PaintRadarBackground(canvas);
  PaintRadarTraffic(canvas);
}

/**
 * This function is called when the Radar needs repainting.
 * @param canvas The canvas to paint on
 */
void
FlarmTrafficWindow::OnPaint(Canvas &canvas) noexcept
{
#ifdef ENABLE_OPENGL
  if (small) {
    const ScopeAlphaBlend alpha_blend;

    canvas.SelectBlackPen();
    canvas.Select(Brush(look.background_color.WithAlpha(0xd0)));
    radar_renderer.DrawCircle(canvas, radar_renderer.GetRadius());

  } else
#endif
    canvas.Clear(look.background_color);

  Paint(canvas);
}

bool
FlarmTrafficWindow::SelectNearTarget(PixelPoint p, int max_distance) noexcept
{
  int min_distance = 99999;
  int min_id = -1;

  for (unsigned i = 0; i < data.list.size(); ++i) {
    // If FLARM target does not exist -> next one
    if (!data.list[i].IsDefined())
      continue;

    int distance_sq = (p - sc[i]).MagnitudeSquared();

    if (distance_sq > min_distance
        || distance_sq > max_distance * max_distance)
      continue;

    min_distance = distance_sq;
    min_id = i;
  }

  if (min_id >= 0)
    SetTarget(min_id);

  return min_id >= 0;
}
