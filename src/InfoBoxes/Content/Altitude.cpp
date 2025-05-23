// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "InfoBoxes/Content/Altitude.hpp"
#include "Factory.hpp"
#include "InfoBoxes/Data.hpp"
#include "InfoBoxes/Panel/Panel.hpp"
#include "InfoBoxes/Panel/AltitudeInfo.hpp"
#include "InfoBoxes/Panel/AltitudeSimulator.hpp"
#include "InfoBoxes/Panel/AltitudeSetup.hpp"
#include "Units/Units.hpp"
#include "Interface.hpp"
#include "Language/Language.hpp"


/*
 * Subpart callback function pointers
 */

#ifdef __clang__
/* gcc gives "redeclaration differs in 'constexpr'" */
constexpr
#endif
const InfoBoxPanel altitude_infobox_panels[] = {
  { N_("Simulator"), LoadAltitudeSimulatorPanel },
  { N_("Info"), LoadAltitudeInfoPanel },
  { N_("Setup"), LoadAltitudeSetupPanel },
  { nullptr, nullptr }
};

const InfoBoxPanel *
InfoBoxContentAltitude::GetDialogContent() noexcept
{
  return altitude_infobox_panels;
}

void
UpdateInfoBoxAltitudeNav(InfoBoxData &data) noexcept
{
  const MoreData &basic = CommonInterface::Basic();

  if (!basic.NavAltitudeAvailable()) {
    data.SetInvalid();

    if (basic.pressure_altitude_available)
      data.SetComment(_("no QNH"));

    return;
  }

  const ComputerSettings &settings_computer = CommonInterface::GetComputerSettings();

  if (basic.baro_altitude_available &&
      settings_computer.features.nav_baro_altitude_enabled)
    data.SetTitle(InfoBoxFactory::GetCaption(InfoBoxFactory::e_H_Baro));
  else
    data.SetTitle(InfoBoxFactory::GetCaption(InfoBoxFactory::e_HeightGPS));

  data.SetValueFromAltitude(basic.nav_altitude);
  data.SetCommentFromAlternateAltitude(basic.nav_altitude);
}

void
InfoBoxContentAltitudeGPS::Update(InfoBoxData &data) noexcept
{
  const NMEAInfo &basic = CommonInterface::Basic();

  if (!basic.gps_altitude_available) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromAltitude(basic.gps_altitude);
  data.SetCommentFromAlternateAltitude(basic.gps_altitude);
}

void
UpdateInfoBoxAltitudeAGL(InfoBoxData &data) noexcept
{
  const DerivedInfo &calculated = CommonInterface::Calculated();

  if (!calculated.altitude_agl_valid) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromAltitude(calculated.altitude_agl);
  data.SetCommentFromAlternateAltitude(calculated.altitude_agl);

  // Set Color (red/black)
  data.SetValueColor(calculated.altitude_agl <
      CommonInterface::GetComputerSettings().task.route_planner.safety_height_terrain ? 1 : 0);
}

void
UpdateInfoBoxAltitudeBaro(InfoBoxData &data) noexcept
{
  const NMEAInfo &basic = CommonInterface::Basic();

  if (!basic.baro_altitude_available) {
    data.SetInvalid();

    if (basic.pressure_altitude_available)
      data.SetComment(_("no QNH"));

    return;
  }

  data.SetValueFromAltitude(basic.baro_altitude);
  data.SetCommentFromAlternateAltitude(basic.baro_altitude);
}

void
UpdateInfoBoxAltitudeQFE(InfoBoxData &data) noexcept
{
  const NMEAInfo &basic = CommonInterface::Basic();
  const auto &calculated = CommonInterface::Calculated();

  const auto any_altitude = basic.GetAnyAltitude();
  if (!any_altitude) {
    data.SetInvalid();
    return;
  }

  if (!calculated.flight.HasTakenOff()) {
    data.SetInvalid();
    data.SetComment(_("Not flying"));
    return;
  }

  const double value = *any_altitude - calculated.flight.takeoff_altitude;
  data.SetValueFromAltitude(value);
  data.SetCommentFromAlternateAltitude(value);
}

void
UpdateInfoBoxAltitudeFlightLevel(InfoBoxData &data) noexcept
{
  const NMEAInfo &basic = CommonInterface::Basic();
  const ComputerSettings &settings_computer =
    CommonInterface::GetComputerSettings();

  if (basic.pressure_altitude_available) {
    auto Altitude = Units::ToUserUnit(basic.pressure_altitude, Unit::FEET);

    // Title color black
    data.SetTitleColor(0);

    // Set Value
    data.FmtValue("{:03}", iround(Altitude / 100));

    // Set Comment
    data.FmtComment("{}ft", iround(Altitude));

  } else if (basic.gps_altitude_available &&
             settings_computer.pressure_available) {
    // Take gps altitude as baro altitude. This is inaccurate but still fits our needs.
    const AtmosphericPressure &qnh = settings_computer.pressure;
    auto Altitude = Units::ToUserUnit(qnh.QNHAltitudeToPressureAltitude(basic.gps_altitude), Unit::FEET);

    // Title color red
    data.SetTitleColor(1);

    // Set Value
    data.FmtValue("{:03}", iround(Altitude / 100));

    // Set Comment
    data.FmtComment("{}ft", iround(Altitude));

  } else if ((basic.baro_altitude_available || basic.gps_altitude_available) &&
             !settings_computer.pressure_available) {
    data.SetInvalid();
    data.SetComment(_("no QNH"));
  } else {
    data.SetInvalid();
  }
}
