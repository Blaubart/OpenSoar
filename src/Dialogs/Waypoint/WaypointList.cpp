// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "WaypointDialogs.hpp"
#include "Dialogs/WidgetDialog.hpp"
#include "Widget/ListWidget.hpp"
#include "Widget/TwoWidgets.hpp"
#include "Widget/RowFormWidget.hpp"
#include "ui/event/KeyCode.hpp"
#include "Form/Edit.hpp"
#include "Form/DataField/Listener.hpp"
#include "Form/DataField/Prefix.hpp"
#include "Profile/Current.hpp"
#include "Profile/Map.hpp"
#include "Profile/Keys.hpp"
#include "Waypoint/LastUsed.hpp"
#include "Waypoint/WaypointList.hpp"
#include "Waypoint/WaypointListBuilder.hpp"
#include "Waypoint/WaypointFilter.hpp"
#include "Waypoint/Waypoints.hpp"
#include "Form/DataField/Enum.hpp"
#include "util/StringPointer.hxx"
#include "util/AllocatedString.hxx"
#include "UIGlobals.hpp"
#include "Look/MapLook.hpp"
#include "Look/DialogLook.hpp"
#include "util/Macros.hpp"
#include "Renderer/WaypointListRenderer.hpp"
#include "Renderer/TwoTextRowsRenderer.hpp"
#include "Units/Units.hpp"
#include "Formatter/AngleFormatter.hpp"
#include "Formatter/UserUnits.hpp"
#include "Interface.hpp"
#include "Blackboard/BlackboardListener.hpp"
#include "Language/Language.hpp"
#include "Components.hpp"
#include "DataComponents.hpp"

#include <algorithm>
#include <list>

#include <cassert>
#include <stdio.h>

enum Controls {
  NAME,
  DISTANCE,
  DIRECTION,
  TYPE,
};

static constexpr unsigned distance_filter_items[] = {
  0, 25, 50, 75, 100, 150, 250, 500, 1000
};

static constexpr int direction_filter_items[] = {
  -1, -1, 0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330
};

static const char *const type_filter_items[] = {
  "*", "Airport", "Landable",
  "Turnpoint", 
  "Start", 
  "Finish", 
  "Left FAI Triangle",
  "Right FAI Triangle",
  "Custom",
  "File 1", "File 2",
  "Map file",
  "Recently Used",
  nullptr
};

struct WaypointListDialogState
{
  StaticString<WaypointFilter::NAME_LENGTH + 1> name;

  int distance_index;
  int direction_index;
  TypeFilter type_index;

  bool IsDefined() const {
    return !name.empty() || distance_index > 0 ||
      direction_index > 0 || type_index != TypeFilter::ALL;
  }

  void ToFilter(WaypointFilter &filter, Angle heading) const {
    filter.name = name;
    filter.distance =
      Units::ToSysDistance(distance_filter_items[distance_index]);
    filter.type_index = type_index;

    if (direction_index != 1)
      filter.direction = Angle::Degrees(direction_filter_items[direction_index]);
    else
      filter.direction = heading;
  }
};

class WaypointFilterWidget;

class WaypointListWidget final
  : public ListWidget, public DataFieldListener,
    NullBlackboardListener {
  Waypoints &way_points;

  WndForm &dialog;

  WaypointFilterWidget &filter_widget;

  WaypointList items;

  TwoTextRowsRenderer row_renderer;

  const GeoPoint location;
  Angle last_heading;

  OrderedTask *const ordered_task;
  const unsigned ordered_task_index;

public:
  WaypointListWidget(Waypoints &_way_points, WndForm &_dialog,
                     WaypointFilterWidget &_filter_widget,
                     GeoPoint _location, Angle _heading,
                     OrderedTask *_ordered_task,
                     unsigned _ordered_task_index)
    :way_points(_way_points), dialog(_dialog),
     filter_widget(_filter_widget),
     location(_location), last_heading(_heading),
     ordered_task(_ordered_task),
     ordered_task_index(_ordered_task_index) {}

  void UpdateList();

  void OnWaypointListEnter();

  WaypointPtr GetCursorObject() const {
    return items.empty()
      ? nullptr
      : items[GetList().GetCursorIndex()].waypoint;
  }

  /* virtual methods from class Widget */
  void Prepare(ContainerWindow &parent, const PixelRect &rc) noexcept override;

  void Show(const PixelRect &rc) noexcept override {
    ListWidget::Show(rc);
    UpdateList();
    CommonInterface::GetLiveBlackboard().AddListener(*this);
  }

  void Hide() noexcept override {
    CommonInterface::GetLiveBlackboard().RemoveListener(*this);

    ListWidget::Hide();
  }

  /* virtual methods from ListItemRenderer */
  void OnPaintItem(Canvas &canvas, const PixelRect rc,
                   unsigned idx) noexcept override;

  /* virtual methods from ListCursorHandler */
  bool CanActivateItem([[maybe_unused]] unsigned index) const noexcept override {
    return true;
  }

  void OnActivateItem([[maybe_unused]] unsigned index) noexcept override;

  /* virtual methods from DataFieldListener */
  void OnModified(DataField &df) noexcept override;

private:
  /* virtual methods from BlackboardListener */
  void OnGPSUpdate([[maybe_unused]] const MoreData &basic) override;
};

class WaypointFilterWidget : public RowFormWidget {
  Angle last_heading;

  DataFieldListener *listener;

public:
  WaypointFilterWidget(const DialogLook &look, Angle _heading)
    :RowFormWidget(look, true), last_heading(_heading) {}

  void SetListener(DataFieldListener *_listener) {
    listener = _listener;
  }

  void Update(Angle last_heading);

  /* virtual methods from class Widget */
  void Prepare(ContainerWindow &parent,
                       const PixelRect &rc) noexcept override;
};

class WaypointListButtons : public RowFormWidget {
  WndForm &dialog;
  WaypointListWidget *list;

public:
  WaypointListButtons(const DialogLook &look, WndForm &_dialog)
    :RowFormWidget(look), dialog(_dialog) {}

  void SetList(WaypointListWidget *_list) {
    list = _list;
  }

  /* virtual methods from class Widget */
  void Prepare([[maybe_unused]] ContainerWindow &parent, [[maybe_unused]] const PixelRect &rc) noexcept override {
    AddButton(_("Details"), [this](){
      list->OnWaypointListEnter();
    });

    AddButton(_("Close"), dialog.MakeModalResultCallback(mrCancel));
  }
};

static WaypointListDialogState dialog_state;

static const char *
GetDirectionData(char *buffer, size_t size, int direction_filter_index,
                 Angle heading)
{
  if (direction_filter_index == 0)
    return "*";
  else if (direction_filter_index == 1)
    StringFormatUnsafe(buffer, "HDG(%s)",
                       FormatBearing(heading).c_str());
  else
    FormatBearing(buffer, size, direction_filter_items[direction_filter_index]);

  return buffer;
}

void
WaypointFilterWidget::Update(Angle _last_heading)
{
  last_heading = _last_heading;

  WndProperty &direction_control = GetControl(DIRECTION);
  DataFieldEnum &direction_df = *(DataFieldEnum *)
    direction_control.GetDataField();

  char buffer[22];
  direction_df.replaceEnumText(1, GetDirectionData(buffer, ARRAY_SIZE(buffer),
                                                   1, last_heading));
  direction_control.RefreshDisplay();
}

static void
FillList(WaypointList &list, const Waypoints &src,
         GeoPoint location, Angle heading, const WaypointListDialogState &state,
         OrderedTask *ordered_task, unsigned ordered_task_index)
{
  if (!state.IsDefined() && src.size() >= 500)
    return;

  WaypointFilter filter;
  state.ToFilter(filter, heading);

  WaypointListBuilder builder(filter, location, list,
                              ordered_task, ordered_task_index);
  builder.Visit(src);

  if (filter.distance > 0 || !filter.direction.IsNegative())
    list.SortByDistance(location);
  else
    list.SortByName();

  list.MakeUnique();
}

static void
FillLastUsedList(WaypointList &list,
                 const WaypointIDList &last_used_ids,
                 const Waypoints &waypoints)
{
  for (auto it = last_used_ids.rbegin(); it != last_used_ids.rend(); it++) {
    auto waypoint = waypoints.LookupId(*it);
    if (waypoint == nullptr)
      continue;

    list.emplace_back(std::move(waypoint));
  }
}

void
WaypointListWidget::UpdateList()
{
  items.clear();

  if (dialog_state.type_index == TypeFilter::LAST_USED)
    FillLastUsedList(items, LastUsedWaypoints::GetList(),
                     way_points);
  else
    FillList(items, way_points, location, last_heading,
             dialog_state,
             ordered_task, ordered_task_index);

  auto &list = GetList();
  list.SetLength(std::max(1u, (unsigned)items.size()));
  list.SetOrigin(0);
  list.SetCursorIndex(0);
  list.Invalidate();
}

void
WaypointListWidget::Prepare(ContainerWindow &parent,
                            const PixelRect &rc) noexcept
{
  const DialogLook &look = UIGlobals::GetDialogLook();
  CreateList(parent, look, rc,
             row_renderer.CalculateLayout(*look.list.font_bold,
                                          look.small_font));
  UpdateList();
}

static DataField *
CreateNameDataField(Waypoints &waypoints, DataFieldListener *listener)
{
  return new PrefixDataField("", [&waypoints](const char *prefix){
    static char buffer[256];
    return waypoints.SuggestNamePrefix(prefix, buffer, ARRAY_SIZE(buffer));
  }, listener);
}

static DataField *
CreateDistanceDataField(DataFieldListener *listener)
{
  DataFieldEnum *df = new DataFieldEnum(listener);
  df->addEnumText("*");

  for (unsigned i = 1; i < ARRAY_SIZE(distance_filter_items); i++) {
    df->addEnumText(FormatUserDistance(Units::ToSysDistance(distance_filter_items[i])));
  }

  df->SetValue(dialog_state.distance_index);
  return df;
}

static DataField *
CreateDirectionDataField(DataFieldListener *listener, Angle last_heading)
{
  char buffer[22];
  DataFieldEnum *df = new DataFieldEnum(listener);
  for (unsigned i = 0; i < ARRAY_SIZE(direction_filter_items); i++)
    df->addEnumText(GetDirectionData(buffer, ARRAY_SIZE(buffer), i,
                                     last_heading));

  df->SetValue(dialog_state.direction_index);
  return df;
}

static void
ReplaceProfilePathBase(DataFieldEnum &df, unsigned i,
                       std::string_view profile_key)
{
  const auto p = Profile::map.GetPathBase(profile_key);
  if (p != nullptr)
    df.replaceEnumText(i, p.c_str());
}

static DataField *
CreateTypeDataField(DataFieldListener *listener)
{
  DataFieldEnum *df = new DataFieldEnum(listener);
  df->addEnumTexts(type_filter_items);

  ReplaceProfilePathBase(*df, (unsigned)TypeFilter::FILE_1,
                         ProfileKeys::WaypointFile);
  ReplaceProfilePathBase(*df, (unsigned)TypeFilter::FILE_2,
                         ProfileKeys::AdditionalWaypointFile);
  ReplaceProfilePathBase(*df, (unsigned)TypeFilter::MAP,
                         ProfileKeys::MapFile);

  df->SetValue(dialog_state.type_index);
  return df;
}

void
WaypointFilterWidget::Prepare([[maybe_unused]] ContainerWindow &parent,
                              [[maybe_unused]] const PixelRect &rc) noexcept
{
  Add(_("Name"), nullptr, CreateNameDataField(*data_components->waypoints, listener));
  Add(_("Distance"), nullptr, CreateDistanceDataField(listener));
  Add(_("Direction"), nullptr, CreateDirectionDataField(listener, last_heading));
  Add(_("Type"), nullptr, CreateTypeDataField(listener));
}

void
WaypointListWidget::OnModified(DataField &df) noexcept
{
  if (filter_widget.IsDataField(NAME, df)) {
    dialog_state.name = df.GetAsString();

    /* pass the focus to the list so the user can use the up/down keys
       to select an item right away after the text input dialog has
       been closed; however if the value was changed by
       incrementing/decrementing the first letter (cursor left/right),
       don't move the focus; we don't know for sure how the value was
       changed, but if the filter has only one letter, it's most
       likely changed by left/right */
    if (dialog_state.name.length() > 1)
      GetList().SetFocus();
  } else if (filter_widget.IsDataField(DISTANCE, df)) {
    const DataFieldEnum &dfe = (const DataFieldEnum &)df;
    dialog_state.distance_index = dfe.GetValue();
  } else if (filter_widget.IsDataField(DIRECTION, df)) {
    const DataFieldEnum &dfe = (const DataFieldEnum &)df;
    dialog_state.direction_index = dfe.GetValue();
  } else if (filter_widget.IsDataField(TYPE, df)) {
    const DataFieldEnum &dfe = (const DataFieldEnum &)df;
    dialog_state.type_index = (TypeFilter)dfe.GetValue();
  }

  UpdateList();
}

void
WaypointListWidget::OnPaintItem(Canvas &canvas, const PixelRect rc,
                                unsigned i) noexcept
{
  if (items.empty()) {
    assert(i == 0);

    const auto *text = dialog_state.IsDefined() || way_points.IsEmpty()
      ? _("No Match!")
      : _("Choose a filter or click here");
    row_renderer.DrawFirstRow(canvas, rc, text);
    return;
  }

  assert(i < items.size());

  const struct WaypointListItem &info = items[i];

  WaypointListRenderer::Draw(canvas, rc, *info.waypoint,
                             info.GetVector(location),
                             row_renderer,
                             UIGlobals::GetMapLook().waypoint,
                             CommonInterface::GetMapSettings().waypoint);
}

void
WaypointListWidget::OnWaypointListEnter()
{
  if (!items.empty())
    dialog.SetModalResult(mrOK);
  else
    filter_widget.GetControl(NAME).BeginEditing();
}

void
WaypointListWidget::OnActivateItem([[maybe_unused]] unsigned index) noexcept
{
  OnWaypointListEnter();
}

void
WaypointListWidget::OnGPSUpdate([[maybe_unused]] const MoreData &basic)
{
  if (dialog_state.direction_index == 1 &&
      !CommonInterface::Calculated().circling) {
    const Angle heading = basic.attitude.heading;
    Angle a = last_heading - heading;
    if (a.AsDelta().Absolute() >= Angle::Degrees(60)) {
      last_heading = heading;
      filter_widget.Update(last_heading);
      UpdateList();
    }
  }
}

WaypointPtr
ShowWaypointListDialog(Waypoints &waypoints, const GeoPoint &_location,
                       OrderedTask *_ordered_task, unsigned _ordered_task_index)
{
  const DialogLook &look = UIGlobals::GetDialogLook();

  const Angle heading = CommonInterface::Basic().attitude.heading;

  dialog_state.name.clear();

  WidgetDialog dialog(WidgetDialog::Full{}, UIGlobals::GetMainWindow(),
                      look, _("Select Waypoint"));

  auto left_widget =
    std::make_unique<TwoWidgets>(std::make_unique<WaypointFilterWidget>(look, heading),
                                 std::make_unique<WaypointListButtons>(look, dialog),
                                 true);

  auto &filter_widget = (WaypointFilterWidget &)left_widget->GetFirst();
  auto &buttons_widget = (WaypointListButtons &)left_widget->GetSecond();

  auto list_widget =
    std::make_unique<WaypointListWidget>(waypoints, dialog, filter_widget,
                                         _location, heading,
                                         _ordered_task, _ordered_task_index);
  const auto &list_widget_ = *list_widget;

  filter_widget.SetListener(list_widget.get());
  buttons_widget.SetList(list_widget.get());

  TwoWidgets *widget = new TwoWidgets(std::move(left_widget),
                                      std::move(list_widget),
                                      false);

  dialog.FinishPreliminary(widget);
  return dialog.ShowModal() == mrOK
    ? list_widget_.GetCursorObject()
    : nullptr;
}
