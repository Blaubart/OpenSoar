// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

/**
 * @file
 * The FLARM Traffic Details dialog displaying extended information about
 * the FLARM targets from the FLARMnet database
 * @todo Button that opens the Waypoint details dialog of the
 * home airport (if found in FLARMnet and local waypoint database)
 */

#include "TrafficDialogs.hpp"
#include "Look/TrafficLook.hpp"
#include "Dialogs/TextEntry.hpp"
#include "Dialogs/Message.hpp"
#include "Dialogs/WidgetDialog.hpp"
#include "Widget/RowFormWidget.hpp"
#include "FLARM/FlarmNetRecord.hpp"
#include "FLARM/Traffic.hpp"
#include "FLARM/Details.hpp"
#include "FLARM/Friends.hpp"
#include "FLARM/Glue.hpp"
#include "Renderer/ColorButtonRenderer.hpp"
#include "UIGlobals.hpp"
#include "Components.hpp"
#include "Formatter/UserUnits.hpp"
#include "Formatter/AngleFormatter.hpp"
#include "util/StringBuilder.hxx"
#include "util/StringCompare.hxx"
#include "util/Macros.hpp"
#include "Language/Language.hpp"
#include "Interface.hpp"
#include "Blackboard/LiveBlackboard.hpp"
#include "Blackboard/BlackboardListener.hpp"
#include "TeamActions.hpp"

class FlarmTrafficDetailsWidget final
  : public RowFormWidget, NullBlackboardListener {
  enum Controls {
    CALLSIGN,
    CHANGE_CALLSIGN_BUTTON,
    SPACER1,
    DISTANCE,
    ALTITUDE,
    VARIO,
    SPACER2,
    PILOT,
    AIRPORT,
    RADIO,
    PLANE,
  };

  WndForm &dialog;

  const FlarmId target_id;

public:
  FlarmTrafficDetailsWidget(WndForm &_dialog, FlarmId _target_id)
    :RowFormWidget(_dialog.GetLook()), dialog(_dialog),
     target_id(_target_id) {}

  void CreateButtons(WidgetDialog &buttons);

  /* virtual methods from Widget */
  void Prepare(ContainerWindow &parent, const PixelRect &rc) noexcept override;
  void Show(const PixelRect &rc) noexcept override;
  void Hide() noexcept override;

private:
  void UpdateChanging(const MoreData &basic);
  void Update();

  void OnCallsignClicked();
  void OnTeamClicked();
  void OnFriendColorClicked(FlarmColor color);

  /* virtual methods from BlackboardListener */
  void OnGPSUpdate(const MoreData &basic) override {
    UpdateChanging(basic);
  }
};

inline void
FlarmTrafficDetailsWidget::CreateButtons(WidgetDialog &buttons)
{
  const ButtonLook &button_look = buttons.GetButtonLook();

  buttons.AddButton(std::make_unique<ColorButtonRenderer>(button_look,
                                                          TrafficLook::team_color_green),
                    [this](){ OnFriendColorClicked(FlarmColor::GREEN); });

  buttons.AddButton(std::make_unique<ColorButtonRenderer>(button_look,
                                                          TrafficLook::team_color_blue),
                    [this](){ OnFriendColorClicked(FlarmColor::BLUE); });

  buttons.AddButton(std::make_unique<ColorButtonRenderer>(button_look,
                                                          TrafficLook::team_color_yellow),
                    [this](){ OnFriendColorClicked(FlarmColor::YELLOW); });

  buttons.AddButton(std::make_unique<ColorButtonRenderer>(button_look,
                                                          TrafficLook::team_color_magenta),
                    [this](){ OnFriendColorClicked(FlarmColor::MAGENTA); });

  buttons.AddButton(_("Clear"), [this](){ OnFriendColorClicked(FlarmColor::NONE); });
  buttons.AddButton(_("Team"), [this](){ OnTeamClicked(); });
}

void
FlarmTrafficDetailsWidget::Prepare([[maybe_unused]] ContainerWindow &parent,
                                   [[maybe_unused]] const PixelRect &rc) noexcept
{
  AddReadOnly(_("Callsign"));
  AddButton(_("Change callsign"), [this](){ OnCallsignClicked(); });
  AddSpacer();
  AddReadOnly(_("Distance"));
  AddReadOnly(_("Altitude"));
  AddReadOnly(_("Vario"));
  AddSpacer();
  AddReadOnly(_("Pilot"));
  AddReadOnly(_("Airport"));
  AddReadOnly(_("Radio frequency"));
  AddReadOnly(_("Plane type"));

  Update();
}

void
FlarmTrafficDetailsWidget::Show(const PixelRect &rc) noexcept
{
  RowFormWidget::Show(rc);
  Update();
  CommonInterface::GetLiveBlackboard().AddListener(*this);
}

void
FlarmTrafficDetailsWidget::Hide() noexcept
{
  CommonInterface::GetLiveBlackboard().RemoveListener(*this);
  RowFormWidget::Hide();
}

/**
 * Updates all the dialogs fields, that are changing frequently.
 * e.g. climb speed, distance, height
 */
void
FlarmTrafficDetailsWidget::UpdateChanging(const MoreData &basic)
{
  char tmp[40];
  const char *value;

  const FlarmTraffic* target =
    basic.flarm.traffic.FindTraffic(target_id);

  bool target_ok = target && target->IsDefined();

  // Fill distance/direction field
  if (target_ok) {
    FormatUserDistanceSmart(target->distance, tmp, true, 20, 1000);
    char *p = tmp + strlen(tmp);
    *p++ = ' ';
    FormatAngleDelta(p, 20, target->Bearing() - basic.track);
    value = tmp;
  } else
    value = "--";

  SetText(DISTANCE, value);

  // Fill altitude field
  if (target_ok) {
    char *p = tmp;
    if (target->altitude_available) {
      FormatUserAltitude(target->altitude, p);
      p += strlen(p);
      *p++ = ' ';
    }

    Angle dir = Angle::FromXY(target->distance, target->relative_altitude);
    FormatVerticalAngleDelta(p, 20, dir);

    value = tmp;
  } else
    value = "--";

  SetText(ALTITUDE, value);

  // Fill climb speed field
  if (target_ok && target->climb_rate_avg30s_available) {
    FormatUserVerticalSpeed(target->climb_rate_avg30s, tmp);
    value = tmp;
  } else
    value = "--";

  SetText(VARIO, value);
}

/**
 * Updates all the dialogs fields.
 * Should be called on dialog opening as it closes the dialog when the
 * target does not exist.
 */
void
FlarmTrafficDetailsWidget::Update()
{
  char tmp[200], tmp_id[7];
  const char *value;

  // Set the dialog caption
  StringFormatUnsafe(tmp, "%s (%s)",
                     _("FLARM Traffic Details"), target_id.Format(tmp_id));
  dialog.SetCaption(tmp);

  // Try to find the target in the FLARMnet database
  /// @todo: make this code a little more usable
  const FlarmNetRecord *record = FlarmDetails::LookupRecord(target_id);
  if (record) {
    // Fill the pilot name field
    SetText(PILOT, record->pilot);

    // Fill the frequency field
    if (!StringIsEmpty(record->frequency))
      value = UnsafeBuildString(tmp, record->frequency.c_str(), " MHz");
    else
      value = "--";
    SetText(RADIO, value);

    // Fill the home airfield field
    SetText(AIRPORT, record->airfield);

    // Fill the plane type field
    SetText(PLANE, record->plane_type);
  } else {
    // Fill the pilot name field
    SetText(PILOT, "--");

    // Fill the frequency field
    SetText(RADIO, "--");

    // Fill the home airfield field
    SetText(AIRPORT, "--");

    // Fill the plane type field
    const FlarmTraffic* target =
      CommonInterface::Basic().flarm.traffic.FindTraffic(target_id);

    const char* actype;
    if (target == nullptr ||
        (actype = FlarmTraffic::GetTypeString(target->type)) == nullptr)
      actype = "--";

    SetText(PLANE, actype);
  }

  // Fill the callsign field (+ registration)
  // note: don't use target->Name here since it is not updated
  //       yet if it was changed
  const char* cs = FlarmDetails::LookupCallsign(target_id);
  if (cs != nullptr && cs[0] != 0) {
    try {
      BasicStringBuilder<char> builder(tmp, ARRAY_SIZE(tmp));
      builder.Append(cs);
      if (record)
        builder.Append(" (", record->registration.c_str(), ")");
      value = tmp;
    } catch (BasicStringBuilder<char>::Overflow) {
      value = cs;
    }
  } else
    value = "--";
  SetText(CALLSIGN, value);

  // Update the frequently changing fields too
  UpdateChanging(CommonInterface::Basic());
}

/**
 * This event handler is called when the "Team" button is pressed
 */
inline void
FlarmTrafficDetailsWidget::OnTeamClicked()
{
  // Ask for confirmation
  if (ShowMessageBox(_("Do you want to set this FLARM contact as your new teammate?"),
                  _("New Teammate"), MB_YESNO) != IDYES)
    return;

  TeamActions::TrackFlarm(target_id);

  // Close the dialog
  dialog.SetModalResult(mrOK);
}

/**
 * This event handler is called when the "Change Callsign" button is pressed
 */
inline void
FlarmTrafficDetailsWidget::OnCallsignClicked()
{
  StaticString<21> newName;
  newName.clear();

  // pre-fill the callsign from flarmnet database or userfile
  const char* cs = FlarmDetails::LookupCallsign(target_id);
  if (cs != nullptr && cs[0] != 0)
    newName = cs;

  if (TextEntryDialog(newName, _("Callsign")) &&
      FlarmDetails::AddSecondaryItem(target_id, newName))
    SaveFlarmNames();

  Update();
}

void
FlarmTrafficDetailsWidget::OnFriendColorClicked(FlarmColor color)
{
  FlarmFriends::SetFriendColor(target_id, color);
  dialog.SetModalResult(mrOK);
}

/**
 * The function opens the FLARM Traffic Details dialog
 */
void
dlgFlarmTrafficDetailsShowModal(FlarmId id)
{
  const DialogLook &look = UIGlobals::GetDialogLook();

  WidgetDialog dialog(WidgetDialog::Full{}, UIGlobals::GetMainWindow(),
                      look, _("FLARM Traffic Details"));

  FlarmTrafficDetailsWidget *widget =
    new FlarmTrafficDetailsWidget(dialog, id);
  widget->CreateButtons(dialog);
  dialog.AddButton(_("Close"), mrCancel);
  dialog.FinishPreliminary(widget);
  dialog.ShowModal();
}
