// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "AirspaceWarningDialog.hpp"
#include "Airspace.hpp"
#include "Dialogs/WidgetDialog.hpp"
#include "Form/Button.hpp"
#include "Look/DialogLook.hpp"
#include "Formatter/UserUnits.hpp"
#include "Renderer/TwoTextRowsRenderer.hpp"
#include "ui/canvas/Canvas.hpp"
#include "Screen/Layout.hpp"
#include "ui/event/PeriodicTimer.hpp"
#include "Airspace/AirspaceWarning.hpp"
#include "Airspace/ProtectedAirspaceWarningManager.hpp"
#include "Airspace/AirspaceWarningManager.hpp"
#include "Formatter/AirspaceFormatter.hpp"
#include "Engine/Airspace/AbstractAirspace.hpp"
#include "util/Macros.hpp"
#include "Interface.hpp"
#include "ActionInterface.hpp"
#include "Language/Language.hpp"
#include "Widget/ListWidget.hpp"
#include "UIGlobals.hpp"
#include "Audio/Sound.hpp"

#include <algorithm>
#include <cassert>
#include <vector>

#include <stdio.h>

class AirspaceWarningListWidget final
  : public ListWidget {

  ProtectedAirspaceWarningManager &airspace_warnings;

  UI::PeriodicTimer update_list_timer{[this]{ UpdateList(); }};

  Button *ack_button;
  Button *ack_day_button;
  Button *enable_button;
  Button *radio_button;

  std::vector<AirspaceWarning> warning_list;

  /**
   * Current list cursor airspace.
   */
  ConstAirspacePtr selected_airspace;

  TwoTextRowsRenderer row_renderer;

  /**
   * Airspace repetitive warning sound interval counter.
   */
  unsigned sound_interval_counter;

public:
  AirspaceWarningListWidget(ProtectedAirspaceWarningManager &aw)
    :airspace_warnings(aw),
     sound_interval_counter(1)
  {}

  void CreateButtons(WidgetDialog &buttons) {
    ack_button = buttons.AddButton(_("ACK"), [this](){ Ack(); });
    ack_day_button = buttons.AddButton(_("ACK Day"), [this](){ AckDay(); });
    enable_button = buttons.AddButton(_("Enable"), [this](){ Enable(); });
    radio_button = buttons.AddButton(_("Radio"), [this](){ Radio(); });
  }

  void CopyList();
  void UpdateList();
  void UpdateButtons();

  [[gnu::pure]]
  const AbstractAirspace *GetSelectedAirspace() const;

  [[gnu::pure]]
  bool HasWarning() const;

  void Ack();
  void AckDay();
  void Enable();
  void Radio() noexcept;

  /* virtual methods from Widget */
  void Prepare(ContainerWindow &parent, const PixelRect &rc) noexcept override;
  void Show(const PixelRect &rc) noexcept override;
  void Hide() noexcept override;

  /* virtual methods from ListItemRenderer */
  void OnPaintItem(Canvas &canvas, const PixelRect rc,
                   unsigned idx) noexcept override;

  /* virtual methods from ListCursorHandler */
  void OnCursorMoved(unsigned index) noexcept override;

  bool CanActivateItem([[maybe_unused]] unsigned index) const noexcept override {
    return true;
  }

  void OnActivateItem(unsigned index) noexcept override;
};

static WndForm *dialog = NULL;
static AirspaceWarningListWidget *list;

static constexpr Color inside_color(254,50,50);
static constexpr Color near_color(254,254,50);
static constexpr Color inside_ack_color(254,100,100);
static constexpr Color near_ack_color(254,254,100);
static bool auto_close = true;


const AbstractAirspace *
AirspaceWarningListWidget::GetSelectedAirspace() const
{
  return selected_airspace.get();
}

void
AirspaceWarningListWidget::UpdateButtons()
{
  auto &airspace = selected_airspace;
  if (airspace == NULL) {
    ack_button->SetEnabled(false);
    ack_day_button->SetEnabled(false);
    enable_button->SetEnabled(false);
    radio_button->SetEnabled(false);
    return;
  }

  bool ack_expired, ack_day;

  {
    ProtectedAirspaceWarningManager::ExclusiveLease lease(airspace_warnings);
    const AirspaceWarning &warning = lease->GetWarning(airspace);
    ack_expired = warning.IsAckExpired();
    ack_day = warning.GetAckDay();
  }

  ack_button->SetEnabled(ack_expired);
  ack_day_button->SetEnabled(!ack_day);
  enable_button->SetEnabled(!ack_expired);
  radio_button->SetEnabled(airspace->GetRadioFrequency().IsDefined());
}

void
AirspaceWarningListWidget::Prepare(ContainerWindow &parent,
                                   const PixelRect &rc) noexcept
{
  const auto &look = UIGlobals::GetDialogLook();

  CreateList(parent, look, rc,
             row_renderer.CalculateLayout(*look.list.font, *look.list.font));
}

void
AirspaceWarningListWidget::OnCursorMoved(unsigned i) noexcept
{
  selected_airspace = i < warning_list.size()
    ? warning_list[i].GetAirspacePtr()
    : nullptr;

  UpdateButtons();
}

void
AirspaceWarningListWidget::Show(const PixelRect &rc) noexcept
{
  sound_interval_counter = 0;
  ListWidget::Show(rc);
  UpdateList();
  update_list_timer.Schedule(std::chrono::milliseconds(500));
}

void
AirspaceWarningListWidget::Hide() noexcept
{
  update_list_timer.Cancel();
  ListWidget::Hide();
}

void
AirspaceWarningListWidget::OnActivateItem([[maybe_unused]] unsigned i) noexcept
{
  if (selected_airspace != nullptr)
    dlgAirspaceDetails(selected_airspace, &airspace_warnings);
}

bool
AirspaceWarningListWidget::HasWarning() const
{
  ProtectedAirspaceWarningManager::Lease lease(airspace_warnings);
  return std::any_of(lease->begin(), lease->end(),
                     [](const auto &i){ return i.IsActive(); });
}

static void
Hide()
{
  dialog->Hide();
  dialog->SetModalResult(mrOK);
}

static void
AutoHide()
{
  // Close the dialog if no warning exists and AutoClose is set
  if (!list->HasWarning() && auto_close)
    Hide();
}

void
AirspaceWarningListWidget::Ack()
{
  const auto &airspace = selected_airspace;
  if (airspace != NULL) {
    airspace_warnings.Acknowledge(airspace);
    UpdateList();
    AutoHide();
  }
}

void
AirspaceWarningListWidget::AckDay()
{
  const auto &airspace = selected_airspace;
  if (airspace != NULL) {
    airspace_warnings.AcknowledgeDay(airspace, true);
    UpdateList();
    AutoHide();
  }
}

void
AirspaceWarningListWidget::Enable()
{
  const auto &airspace = selected_airspace;
  if (airspace == NULL)
    return;

  {
    ProtectedAirspaceWarningManager::ExclusiveLease lease(airspace_warnings);
    AirspaceWarning *warning = lease->GetWarningPtr(*airspace);
    if (warning == NULL)
      return;

    warning->AcknowledgeInside(false);
    warning->AcknowledgeWarning(false);
    warning->AcknowledgeDay(false);
  }

  UpdateList();
}

inline void
AirspaceWarningListWidget::Radio() noexcept
{
  if (selected_airspace != nullptr &&
      selected_airspace->GetRadioFrequency().IsDefined())
    ActionInterface::SetActiveFrequency(selected_airspace->GetRadioFrequency(),
                                        selected_airspace->GetName());
}

void
AirspaceWarningListWidget::OnPaintItem(Canvas &canvas,
                                       const PixelRect paint_rc,
                                       unsigned i) noexcept
{
  char buffer[128];

  // This constant defines the margin that should be respected
  // for renderring within the paint_rc area.
  const unsigned padding = Layout::GetTextPadding();

  if (i == 0 && warning_list.empty()) {
    /* the warnings were emptied between the opening of the dialog and
       this refresh, so only need to display "No Warnings" for top
       item, otherwise exit immediately */
    row_renderer.DrawFirstRow(canvas, paint_rc, _("No Warnings"));
    return;
  }

  assert(i < warning_list.size());

  const auto &warning = warning_list[i];
  const AbstractAirspace &airspace = warning.GetAirspace();

  // word "inside" is used as the etalon, because it is longer than "near" and
  // currently (9.4.2011) there is no other possibility for the status text.
  const int status_width = canvas.CalcTextWidth("inside");
  // "1888" is used in order to have enough space for 4-digit heights with "AGL"
  const int altitude_width = canvas.CalcTextWidth("1888 m AGL");

  // Dynamic columns scaling - "name" column is flexible, altitude and state
  // columns are fixed-width.
  auto [text_altitude_rc, status_rc] =
    paint_rc.VerticalSplit(paint_rc.right - (2 * padding + status_width));
  auto text_rc =
    text_altitude_rc.VerticalSplit(text_altitude_rc.right - (padding + altitude_width)).first;
  text_rc.right -= padding;

  if (!warning.IsActive())
    canvas.SetTextColor(COLOR_GRAY);

  { // name, altitude info
    StringFormat(buffer, ARRAY_SIZE(buffer), "%s %s",
                 airspace.GetName(),
                 AirspaceFormatter::GetClass(airspace));

    row_renderer.DrawFirstRow(canvas, text_rc, buffer);

    AirspaceFormatter::FormatAltitudeShort(buffer, airspace.GetTop());
    row_renderer.DrawRightFirstRow(canvas, text_altitude_rc, buffer);

    AirspaceFormatter::FormatAltitudeShort(buffer, airspace.GetBase());
    row_renderer.DrawRightSecondRow(canvas, text_altitude_rc, buffer);
  }

  if (const auto &solution = warning.GetSolution();
      warning.IsWarning() && !warning.IsInside() && solution.IsValid()) {

    snprintf(buffer, sizeof(buffer), "%d secs",
              (int)solution.elapsed_time.count());

    if (solution.distance > 0)
      snprintf(buffer + strlen(buffer), sizeof(buffer), " dist %d m",
                (int)solution.distance);
    else {
      /* the airspace is right above or below us - show the vertical
         distance */
      strcat(buffer, " vertical ");

      auto delta = solution.altitude - CommonInterface::Basic().nav_altitude;
      FormatRelativeUserAltitude(delta, buffer + strlen(buffer), true);
    }

    row_renderer.DrawSecondRow(canvas, text_rc, buffer);
  }

  /* draw the warning state indicator */

  Color state_color;
  const char *state_text;

  if (warning.IsInside()) {
    state_color = warning.IsActive() ? inside_color : inside_ack_color;
    state_text = "inside";
  } else if (warning.IsWarning()) {
    state_color = warning.IsActive() ? near_color : near_ack_color;
    state_text = "near";
  } else {
    state_color = COLOR_WHITE;
    state_text = NULL;
  }

  const PixelSize state_text_size =
    canvas.CalcTextSize(state_text != NULL ? state_text : "W");

  if (state_color != COLOR_WHITE) {
    /* colored background */
    PixelRect rc = status_rc;
    rc.top += padding;
    rc.right -= padding;
    rc.bottom -= padding;

    canvas.DrawFilledRectangle(rc, state_color);

    /* on this background we just painted, we must use black color for
       the state text; our caller might have selected a different
       color, override it here */
    canvas.SetTextColor(COLOR_BLACK);
  }

  if (state_text != NULL) {
    // -- status text will be centered inside its table cell:
    canvas.DrawText(status_rc.CenteredTopLeft(state_text_size), state_text);
  }
}

inline void
AirspaceWarningListWidget::CopyList()
{
  const ProtectedAirspaceWarningManager::Lease lease(airspace_warnings);
  warning_list = {lease->begin(), lease->end()};
}

void
AirspaceWarningListWidget::UpdateList()
{
  CopyList();

  if (!warning_list.empty()) {
    GetList().SetLength(warning_list.size());

    int i = -1;
    if (selected_airspace != NULL) {
      auto it = std::find_if(warning_list.begin(), warning_list.end(),
                             [this](const auto &i){
                               return &i.GetAirspace() == selected_airspace.get();
                             });
      if (it != warning_list.end()) {
        i = std::distance(warning_list.begin(), it);
        GetList().SetCursorIndex(i);
      }
    }

    if (i < 0)
      /* the selection may have changed, update CursorAirspace */
      OnCursorMoved(GetList().GetCursorIndex());

    // Process repetitive sound warnings if they are enabled in config
    const AirspaceWarningConfig &warning_config =
      CommonInterface::GetComputerSettings().airspace.warnings;
    if (warning_config.repetitive_sound) {
      FloatDuration tt_closest_airspace{1000};
      for (const auto &i : warning_list) {
        /* Find smallest time to nearest aispace (cannot always rely
           on fact that closest airspace should be in the beginning of
           the list) */
        if (!i.IsInside())
          tt_closest_airspace = std::min(tt_closest_airspace,
                                         i.GetSolution().elapsed_time);
        else
          tt_closest_airspace = {};
      }

      const unsigned sound_interval =
        ((tt_closest_airspace * 3 / warning_config.warning_time) + 1) * 2;
      if (sound_interval_counter >= sound_interval) {
        PlayResource("IDR_WAV_BEEPBWEEP");
        sound_interval_counter = 1;
      } else
        ++sound_interval_counter;
    }
  } else {
    GetList().SetLength(1);
    selected_airspace = NULL;
    sound_interval_counter = 0;
  }

  GetList().Invalidate();
  UpdateButtons();
  AutoHide();
}

bool
dlgAirspaceWarningVisible()
{
  return (dialog != NULL);
}

void
dlgAirspaceWarningsShowModal(ProtectedAirspaceWarningManager &_warnings,
                             bool _auto_close)
{
  if (dlgAirspaceWarningVisible())
    return;

  auto_close = _auto_close;

  list = new AirspaceWarningListWidget(_warnings);

  WidgetDialog dialog2(WidgetDialog::Full{}, UIGlobals::GetMainWindow(),
                       UIGlobals::GetDialogLook(),
                       _("Airspace Warnings"), list);
  list->CreateButtons(dialog2);
  dialog2.AddButton(_("Close"), mrOK);
  dialog2.EnableCursorSelection();

  dialog = &dialog2;

  dialog2.ShowModal();

  // Needed for dlgAirspaceWarningVisible()
  dialog = NULL;
}
