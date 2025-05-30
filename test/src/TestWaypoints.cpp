// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "Waypoint/Waypoints.hpp"
#include "Geo/GeoVector.hpp"
#include "test_debug.hpp"

#include <functional>

#include <stdio.h>

extern "C" {
#include "tap.h"
}

class WaypointPredicateCounter
{
public:
  typedef std::function<bool(const Waypoint &wp)> Predicate;

private:
  Predicate predicate;
  unsigned count;

public:
  WaypointPredicateCounter(const Predicate &_predicate)
    :predicate(_predicate), count(0) {}

  void Visit(const WaypointPtr &wp) noexcept {
    if (predicate(*wp))
      count++;
  }

  unsigned GetCounter() const {
    return count;
  }
};

static void
AddSpiralWaypoints(Waypoints &waypoints,
                   const GeoPoint &center = GeoPoint(Angle::Degrees(51.4),
                                                     Angle::Degrees(7.85)),
                   Angle angle_start = Angle::Degrees(0),
                   Angle angle_step = Angle::Degrees(15),
                   double distance_start = 0,
                   double distance_step = 1000,
                   double distance_max = 150000)
{
  assert(distance_step > 0);

  for (unsigned i = 0;; ++i) {
    GeoVector vector;
    vector.distance = distance_start + distance_step * i;
    if (vector.distance > distance_max)
      break;

    vector.bearing = angle_start + angle_step * i;

    Waypoint waypoint{vector.EndPoint(center)};
    waypoint.original_id = i;
    waypoint.elevation = i * 10 - 500;
    waypoint.has_elevation = true;

    StaticString<256> buffer;

    if (i % 7 == 0) {
      buffer = "Airfield";
      waypoint.type = Waypoint::Type::AIRFIELD;
    } else if (i % 3 == 0) {
      buffer = "Field";
      waypoint.type = Waypoint::Type::OUTLANDING;
    } else
      buffer = "Waypoint";

    buffer.AppendFormat(" #%d", i + 1);
    waypoint.name = buffer;

    waypoints.Append(std::move(waypoint));
  }

  waypoints.Optimise();
}

static void
TestLookups(const Waypoints &waypoints, const GeoPoint &center)
{
  WaypointPtr waypoint;

  ok1((waypoint = waypoints.LookupId(0)) == NULL);
  ok1((waypoint = waypoints.LookupId(1)) != NULL);
  ok1(waypoint->original_id == 0);
  ok1((waypoint = waypoints.LookupId(151)) != NULL);
  ok1(waypoint->original_id == 150);
  ok1((waypoint = waypoints.LookupId(152)) == NULL);
  ok1((waypoint = waypoints.LookupId(160)) == NULL);

  ok1((waypoint = waypoints.LookupLocation(center, 0)) != NULL);
  ok1(waypoint->original_id == 0);

  ok1((waypoint = waypoints.LookupName("Waypoint #5")) != NULL);
  ok1(waypoint->original_id == 4);

  ok1((waypoint = waypoints.LookupLocation(waypoint->location, 10000)) != NULL);
  ok1(waypoint->original_id == 4);
}

class BeginsWith
{
  const char *prefix;

public:
  BeginsWith(const char *_prefix):prefix(_prefix) {}

  bool operator()(const Waypoint &waypoint) {
    return StringStartsWith(waypoint.name.c_str(), prefix);
  }
};

static void
TestNamePrefixVisitor(const Waypoints &waypoints, const char *prefix,
                      unsigned expected_results)
{
  WaypointPredicateCounter::Predicate predicate = BeginsWith(prefix);
  WaypointPredicateCounter prefix_counter(predicate);
  waypoints.VisitNamePrefix(prefix, [&](const auto &wp){ prefix_counter.Visit(wp); });
  ok1(prefix_counter.GetCounter() == expected_results);
}

static void
TestNamePrefixVisitor(const Waypoints &waypoints)
{
  TestNamePrefixVisitor(waypoints, "", 151);
  TestNamePrefixVisitor(waypoints, "Foo", 0);
  TestNamePrefixVisitor(waypoints, "a", 0);
  TestNamePrefixVisitor(waypoints, "A", 22);
  TestNamePrefixVisitor(waypoints, "Air", 22);
  TestNamePrefixVisitor(waypoints, "Field", 51 - 8);
}

class CloserThan
{
  double distance;
  GeoPoint location;

public:
  CloserThan(double _distance, const GeoPoint &_location)
    :distance(_distance), location(_location) {}

  bool operator()(const Waypoint &waypoint) {
    return location.Distance(waypoint.location) < distance;
  }
};

static void
TestRangeVisitor(const Waypoints &waypoints, const GeoPoint &location,
                 double distance, unsigned expected_results)
{
  WaypointPredicateCounter::Predicate predicate = CloserThan(distance, location);
  WaypointPredicateCounter distance_counter(predicate);
  waypoints.VisitWithinRange(location, distance, [&](const auto &wp){ distance_counter.Visit(wp); });
  ok1(distance_counter.GetCounter() == expected_results);
}

static void
TestRangeVisitor(const Waypoints &waypoints, const GeoPoint &center)
{
  TestRangeVisitor(waypoints, center, 1, 1);
  TestRangeVisitor(waypoints, center, 999, 1);
  TestRangeVisitor(waypoints, center, 1300, 2);
  TestRangeVisitor(waypoints, center, 10500, 11);
  TestRangeVisitor(waypoints, center, 1000000, 151);
}

static bool
OriginalIDAbove5(const Waypoint &waypoint) {
  return waypoint.original_id > 5;
}

static void
TestGetNearest(const Waypoints &waypoints, const GeoPoint &center)
{
  WaypointPtr waypoint;
  GeoPoint _near = GeoVector(250, Angle::Degrees(15)).EndPoint(center);
  GeoPoint _far = GeoVector(750, Angle::Degrees(15)).EndPoint(center);
  GeoPoint further = GeoVector(4200, Angle::Degrees(48)).EndPoint(center);

  ok1((waypoint = waypoints.GetNearest(center, 1)) != NULL);
  ok1(waypoint->original_id == 0);

  ok1((waypoint = waypoints.GetNearest(center, 10000)) != NULL);
  ok1(waypoint->original_id == 0);

  ok1((waypoint = waypoints.GetNearest(_near, 1)) == NULL);

  ok1((waypoint = waypoints.GetNearest(_near, 10000)) != NULL);
  ok1(waypoint->original_id == 0);

  ok1((waypoint = waypoints.GetNearest(_far, 1)) == NULL);

  ok1((waypoint = waypoints.GetNearest(_far, 10000)) != NULL);
  ok1(waypoint->original_id == 1);

  ok1((waypoint = waypoints.GetNearestLandable(center, 1)) != NULL);
  ok1(waypoint->original_id == 0);

  ok1((waypoint = waypoints.GetNearestLandable(center, 10000)) != NULL);
  ok1(waypoint->original_id == 0);

  ok1((waypoint = waypoints.GetNearestLandable(further, 1)) == NULL);

  ok1((waypoint = waypoints.GetNearestLandable(further, 10000)) != NULL);
  ok1(waypoint->original_id == 3);

  ok1((waypoint = waypoints.GetNearestIf(center, 1, OriginalIDAbove5)) == NULL);

  ok1((waypoint = waypoints.GetNearestIf(center, 10000, OriginalIDAbove5)) != NULL);
  ok1(waypoint->original_id == 6);
}

static void
TestIterator(const Waypoints &waypoints)
{
  unsigned count = 0;
  for (const auto &i : waypoints) {
    count++;
    (void)i;
  }

  ok1(count == 151);
}

static unsigned
TestCopy(Waypoints& waypoints)
{
  const WaypointPtr wp = waypoints.LookupId(5);
  if (!wp)
    return false;

  unsigned size_old = waypoints.size();
  Waypoint wp_copy = *wp;
  wp_copy.id = waypoints.size() + 1;
  waypoints.Append(std::move(wp_copy));
  waypoints.Optimise();
  unsigned size_new = waypoints.size();
  return (size_new == size_old + 1);
}

static bool
TestErase(Waypoints& waypoints, unsigned id)
{
  waypoints.Optimise();
  auto wp = waypoints.LookupId(id);
  if (wp == NULL)
    return false;

  waypoints.Erase(std::move(wp));
  waypoints.Optimise();

  wp = waypoints.LookupId(id);
  return wp == NULL;
}

static bool
TestReplace(Waypoints& waypoints, unsigned id)
{
  auto wp = waypoints.LookupId(id);
  if (wp == NULL)
    return false;

  std::string oldName = wp->name;

  Waypoint copy = *wp;
  copy.name = "Fred";
  waypoints.Replace(wp, std::move(copy));
  waypoints.Optimise();

  wp = waypoints.LookupId(id);
  return wp != NULL && wp->name != oldName && wp->name == "Fred";
}

int
main(int argc, char** argv)
{
  if (!ParseArgs(argc, argv))
    return 0;

  plan_tests(52);

  Waypoints waypoints;
  GeoPoint center(Angle::Degrees(51.4), Angle::Degrees(7.85));

  // AddSpiralWaypoints creates 151 waypoints from
  // 0km to 150km distance in 1km steps
  AddSpiralWaypoints(waypoints, center);

  ok1(!waypoints.IsEmpty());
  ok1(waypoints.size() == 151);

  TestLookups(waypoints, center);
  TestNamePrefixVisitor(waypoints);
  TestRangeVisitor(waypoints, center);
  TestGetNearest(waypoints, center);
  TestIterator(waypoints);

  ok(TestCopy(waypoints), "waypoint copy", 0);
  ok(TestErase(waypoints, 3), "waypoint erase", 0);
  ok(TestReplace(waypoints, 4), "waypoint replace", 0);

  // test clear
  waypoints.Clear();
  ok1(waypoints.IsEmpty());
  ok1(waypoints.size() == 0);

  return exit_status();
}
