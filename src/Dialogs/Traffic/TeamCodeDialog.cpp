// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "TrafficDialogs.hpp"
#include "Dialogs/WidgetDialog.hpp"
#include "Dialogs/TextEntry.hpp"
#include "Dialogs/Waypoint/WaypointDialogs.hpp"
#include "Dialogs/Message.hpp"
#include "Widget/RowFormWidget.hpp"
#include "UIGlobals.hpp"
#include "FLARM/Details.hpp"
#include "FLARM/Glue.hpp"
#include "Computer/Settings.hpp"
#include "Profile/Profile.hpp"
#include "Engine/Waypoint/Waypoint.hpp"
#include "Formatter/AngleFormatter.hpp"
#include "Formatter/UserUnits.hpp"
#include "Interface.hpp"
#include "Blackboard/BlackboardListener.hpp"
#include "Language/Language.hpp"
#include "TeamActions.hpp"
#include "util/StringCompare.hxx"
#include "util/StringStrip.hxx"
#include "util/TruncateString.hpp"
#include "util/Macros.hpp"
#include "Components.hpp"
#include "DataComponents.hpp"

class TeamCodeWidget final
  : public RowFormWidget, NullBlackboardListener {
  enum Controls {
    OWN_CODE,
    MATE_CODE,
    RANGE,
    BEARING,
    RELATIVE_BEARING,
    FLARM_LOCK,
  };

public:
  TeamCodeWidget(const DialogLook &look)
    :RowFormWidget(look) {}

  void CreateButtons(WidgetDialog &buttons);
  void Update(const MoreData &basic, const DerivedInfo &calculated);

private:
  void OnCodeClicked();
  void OnSetWaypointClicked();
  void OnFlarmLockClicked();

  /* virtual methods from class Widget */
  void Prepare(ContainerWindow &parent,
               const PixelRect &rc) noexcept override;
  void Show(const PixelRect &rc) noexcept override;
  void Hide() noexcept override;

  /* virtual methods from class BlackboardListener */
  virtual void OnCalculatedUpdate(const MoreData &basic,
                                  const DerivedInfo &calculated) override;
};

inline void
TeamCodeWidget::CreateButtons(WidgetDialog &buttons)
{
  buttons.AddButton(_("Set code"), [this](){ OnCodeClicked(); });
  buttons.AddButton(_("Set WP"), [this](){ OnSetWaypointClicked(); });
  buttons.AddButton(_("Flarm Lock"), [this](){ OnFlarmLockClicked(); });
}

void
TeamCodeWidget::Prepare([[maybe_unused]] ContainerWindow &parent,
                        [[maybe_unused]] const PixelRect &rc) noexcept
{
  AddReadOnly(_("Own code"));
  AddReadOnly(_("Mate code"));
  AddReadOnly(_("Range"));
  AddReadOnly(_("Bearing"));
  AddReadOnly(_("Rel. bearing"));
  AddReadOnly(_("Flarm lock"));
}

void
TeamCodeWidget::Show(const PixelRect &rc) noexcept
{
  Update(CommonInterface::Basic(), CommonInterface::Calculated());
  CommonInterface::GetLiveBlackboard().AddListener(*this);
  RowFormWidget::Show(rc);
}

void
TeamCodeWidget::Hide() noexcept
{
  RowFormWidget::Hide();
  CommonInterface::GetLiveBlackboard().RemoveListener(*this);
}

void
TeamCodeWidget::Update(const MoreData &basic, const DerivedInfo &calculated)
{
  const TeamInfo &teamcode_info = calculated;
  const TeamCodeSettings &settings =
    CommonInterface::GetComputerSettings().team_code;

  SetText(RELATIVE_BEARING,
          teamcode_info.teammate_available && basic.track_available
          ? FormatAngleDelta(teamcode_info.teammate_vector.bearing - basic.track).c_str()
          : "---");

  if (teamcode_info.teammate_available) {
    SetText(BEARING,
            FormatBearing(teamcode_info.teammate_vector.bearing).c_str());

    SetText(RANGE,
            FormatUserDistanceSmart(teamcode_info.teammate_vector.distance));
  }

  SetText(OWN_CODE, teamcode_info.own_teammate_code.GetCode());
  SetText(MATE_CODE, settings.team_code.GetCode());
  SetText(FLARM_LOCK,
          settings.team_flarm_id.IsDefined()
          ? settings.team_flarm_callsign.c_str()
          : "");
}

void
TeamCodeWidget::OnCalculatedUpdate(const MoreData &basic,
                                   const DerivedInfo &calculated)
{
  Update(basic, calculated);
}

inline void
TeamCodeWidget::OnSetWaypointClicked()
{
  const auto wp =
    ShowWaypointListDialog(*data_components->waypoints, CommonInterface::Basic().location);
  if (wp != nullptr) {
    CommonInterface::SetComputerSettings().team_code.team_code_reference_waypoint = wp->id;
    Profile::Set(ProfileKeys::TeamcodeRefWaypoint, wp->id);
  }
}

inline void
TeamCodeWidget::OnCodeClicked()
{
  char newTeammateCode[10];

  CopyTruncateString(newTeammateCode, ARRAY_SIZE(newTeammateCode),
                     CommonInterface::GetComputerSettings().team_code.team_code.GetCode());

  if (!TextEntryDialog(newTeammateCode, 7))
    return;

  StripRight(newTeammateCode);

  TeamCodeSettings &settings =
    CommonInterface::SetComputerSettings().team_code;
  settings.team_code.Update(newTeammateCode);
  if (settings.team_code.IsDefined())
    settings.team_flarm_id.Clear();
}

inline void
TeamCodeWidget::OnFlarmLockClicked()
{
  TeamCodeSettings &settings =
    CommonInterface::SetComputerSettings().team_code;
  char newTeamFlarmCNTarget[decltype(settings.team_flarm_callsign)::capacity()];
  strcpy(newTeamFlarmCNTarget, settings.team_flarm_callsign.c_str());

  if (!TextEntryDialog(newTeamFlarmCNTarget, 4))
    return;

  if (StringIsEmpty(newTeamFlarmCNTarget)) {
    settings.team_flarm_id.Clear();
    settings.team_flarm_callsign.clear();
    return;
  }

  LoadFlarmDatabases();

  FlarmId ids[30];
  unsigned count =
    FlarmDetails::FindIdsByCallSign(newTeamFlarmCNTarget, ids, 30);

  if (count == 0) {
    ShowMessageBox(_("Unknown Competition Number"),
                   _("Not Found"), MB_OK | MB_ICONINFORMATION);
    return;
  }

  const FlarmId id = PickFlarmTraffic(_("Set new teammate"), ids, count);
  if (!id.IsDefined())
    return;

  TeamActions::TrackFlarm(id, newTeamFlarmCNTarget);
}

void
dlgTeamCodeShowModal()
{
  const DialogLook &look = UIGlobals::GetDialogLook();
  TWidgetDialog<TeamCodeWidget>
    dialog(WidgetDialog::Auto{}, UIGlobals::GetMainWindow(),
           look, _("Team Code"));
  dialog.SetWidget(look);
  dialog.GetWidget().CreateButtons(dialog);
  dialog.AddButton(_("Close"), mrOK);
  dialog.ShowModal();
}
