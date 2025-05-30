// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "TrafficDialogs.hpp"
#include "Dialogs/WidgetDialog.hpp"
#include "Widget/ListWidget.hpp"
#include "Widget/TwoWidgets.hpp"
#include "Widget/RowFormWidget.hpp"
#include "Renderer/TwoTextRowsRenderer.hpp"
#include "ui/canvas/Canvas.hpp"
#include "Screen/Layout.hpp"
#include "Form/DataField/Prefix.hpp"
#include "Form/DataField/Listener.hpp"
#include "FLARM/FlarmNetRecord.hpp"
#include "FLARM/Details.hpp"
#include "FLARM/Id.hpp"
#include "FLARM/Global.hpp"
#include "FLARM/TrafficDatabases.hpp"
#include "util/StaticString.hxx"
#include "Language/Language.hpp"
#include "UIGlobals.hpp"
#include "Look/DialogLook.hpp"
#include "Look/Look.hpp"
#include "Interface.hpp"
#include "Formatter/UserUnits.hpp"
#include "Formatter/AngleFormatter.hpp"
#include "Blackboard/BlackboardListener.hpp"
#include "Tracking/SkyLines/Data.hpp"
#include "Tracking/TrackingGlue.hpp"
#include "Engine/Waypoint/Waypoints.hpp"
#include "Components.hpp"
#include "NetComponents.hpp"
#include "DataComponents.hpp"
#include "Pan.hpp"

using namespace std::chrono;

enum Controls {
  CALLSIGN,
};

enum Buttons {
  DETAILS,
  MAP,
};

class TrafficListButtons;

class TrafficListWidget : public ListWidget, public DataFieldListener,
                          NullBlackboardListener {
  struct Item {
    /**
     * The FLARM traffic id.  If this is "undefined", then this object
     * does not refer to FLARM traffic.
     */
    FlarmId id;

#ifdef HAVE_SKYLINES_TRACKING
    /**
     * The SkyLines account id.
     */
    uint32_t skylines_id = 0;

    SkyLinesTracking::Data::Time time_of_day;
#endif

    /**
     * The color that was assigned by the user to this FLARM peer.  It
     * is FlarmColor::COUNT if the color has not yet been determined.
     */
    FlarmColor color = FlarmColor::COUNT;

    /**
     * Were the attributes below already lazy-loaded from the
     * database?  We can't use nullptr for this, because both will be
     * nullptr after a failed lookup.
     */
    bool loaded = false;

    const FlarmNetRecord *record;
    const char *callsign;

    /**
     * This object's location.  Check GeoPoint::IsValid().
     */
    GeoPoint location = GeoPoint::Invalid();

    /**
     * The vector from the current aircraft location to this object's
     * location (if known).  Check GeoVector::IsValid().
     */
    GeoVector vector = GeoVector::Invalid();

    /**
     * The display name of the SkyLines account.
     */
    std::string name;

#ifdef HAVE_SKYLINES_TRACKING
    StaticString<20> near_name;
    double near_distance;

    int altitude;
#endif

    explicit Item(FlarmId _id)
      :id(_id) {
      assert(id.IsDefined());
      assert(IsFlarm());

#ifdef HAVE_SKYLINES_TRACKING
      near_name.clear();
#endif
    }

#ifdef HAVE_SKYLINES_TRACKING
    explicit Item(uint32_t _id, SkyLinesTracking::Data::Time _time_of_day,
                  const GeoPoint &_location, int _altitude,
                  std::string &&_name)
      :id(FlarmId::Undefined()), skylines_id(_id),
       time_of_day(_time_of_day),
       color(FlarmColor::COUNT),
       loaded(false),
       location(_location),
       vector(GeoVector::Invalid()), name(std::move(_name)),
       altitude(_altitude) {
      assert(IsSkyLines());

      near_name.clear();
    }
#endif

    /**
     * Does this object describe a FLARM?
     */
    bool IsFlarm() const {
      return id.IsDefined();
    }

#ifdef HAVE_SKYLINES_TRACKING
    /**
     * Does this object describe data from SkyLines live tracking?
     */
    bool IsSkyLines() const {
      return skylines_id != 0;
    }
#endif

    void Load() {
      if (IsFlarm()) {
        record = traffic_databases->flarm_net.FindRecordById(id);
        callsign = traffic_databases->FindNameById(id);
#ifdef HAVE_SKYLINES_TRACKING
      } else if (IsSkyLines()) {
        record = nullptr;
        callsign = nullptr;
#endif
      } else {
        gcc_unreachable();
      }

      loaded = true;
    }

    void AutoLoad() {
      if (IsFlarm() && color == FlarmColor::COUNT)
        color = traffic_databases->GetColor(id);

      if (!loaded)
        Load();
    }
  };

  typedef std::vector<Item> ItemList;

  WndForm &dialog;

  const RowFormWidget *const filter_widget;

  TrafficListButtons *const buttons;

  ItemList items;

  /**
   * The time stamp of the newest #FlarmTraffic object.  This is used
   * to check whether the list needs to be redrawn.
   */
  Validity last_update;

  TwoTextRowsRenderer row_renderer;

public:
  TrafficListWidget(WndForm &_dialog,
                    const FlarmId *array, size_t count)
    :dialog(_dialog), filter_widget(nullptr),
     buttons(nullptr) {
    items.reserve(count);

    for (unsigned i = 0; i < count; ++i)
      items.emplace_back(array[i]);
  }

  TrafficListWidget(WndForm &_dialog,
                    const RowFormWidget &_filter_widget,
                    TrafficListButtons &_buttons)
    :dialog(_dialog), filter_widget(&_filter_widget),
     buttons(&_buttons) {
  }

  [[gnu::pure]]
  FlarmId GetCursorId() const {
    return items.empty()
      ? FlarmId::Undefined()
      : items[GetList().GetCursorIndex()].id;
  }

private:
  /**
   * Find an existing item by its FLARM id.  This is a simple linear
   * search that doesn't scale well with a large list.
   */
  [[gnu::pure]]
  ItemList::iterator FindItem(FlarmId id) {
    assert(id.IsDefined());

    return std::find_if(items.begin(), items.end(),
                        [id](const Item &item) { return item.id == id; });
  }

  /**
   * Add a new item to the list, unless the given FLARM id already
   * exists.
   */
  Item &AddItem(FlarmId id) {
    auto existing = FindItem(id);
    if (existing != items.end())
      return *existing;

    items.emplace_back(id);
    return items.back();
  }

  void UpdateList();

  /**
   * Update volatile data on existing items (e.g. their current
   * positions).
   */
  void UpdateVolatile();

  void UpdateButtons();

  void OpenDetails(unsigned index);

  void OpenMap(unsigned index);

public:
  void OpenDetails() noexcept {
    OpenDetails(GetList().GetCursorIndex());
  }

  void OpenMap() noexcept {
    OpenMap(GetList().GetCursorIndex());
  }

  /* virtual methods from class Widget */

  void Prepare(ContainerWindow &parent,
               const PixelRect &rc) noexcept override;

  void Show(const PixelRect &rc) noexcept override {
    ListWidget::Show(rc);

    if (filter_widget != nullptr)
      UpdateList();

    CommonInterface::GetLiveBlackboard().AddListener(*this);
  }

  void Hide() noexcept override {
    CommonInterface::GetLiveBlackboard().RemoveListener(*this);

    ListWidget::Hide();
  }

  /* virtual methods from ListItemRenderer */
  virtual void OnPaintItem(Canvas &canvas, const PixelRect rc,
                           unsigned idx) noexcept override;

  /* virtual methods from ListCursorHandler */
  virtual void OnCursorMoved([[maybe_unused]] unsigned index) noexcept override {
    UpdateButtons();
  }

  virtual bool CanActivateItem([[maybe_unused]] unsigned index) const noexcept override {
    return true;
  }

  virtual void OnActivateItem(unsigned index) noexcept override;

  /* virtual methods from DataFieldListener */
  void OnModified([[maybe_unused]] DataField &df) noexcept override {
    UpdateList();
  }

private:
  /* virtual methods from BlackboardListener */
  virtual void OnGPSUpdate([[maybe_unused]] const MoreData &basic) override {
    UpdateVolatile();
  }
};

class TrafficFilterWidget : public RowFormWidget {
  DataFieldListener *listener;

public:
  TrafficFilterWidget(const DialogLook &look)
    :RowFormWidget(look, true) {}

  void SetListener(DataFieldListener *_listener) {
    listener = _listener;
  }

  void Prepare([[maybe_unused]] ContainerWindow &parent,
               [[maybe_unused]] const PixelRect &rc) noexcept override {
    PrefixDataField *callsign_df = new PrefixDataField("", listener);
    Add(_("Competition ID"), nullptr, callsign_df);
  }
};

class TrafficListButtons : public RowFormWidget {
  WndForm &dialog;
  TrafficListWidget *list;

public:
  TrafficListButtons(const DialogLook &look, WndForm &_dialog)
    :RowFormWidget(look), dialog(_dialog) {}

  void SetList(TrafficListWidget *_list) noexcept {
    list = _list;
  }

  void Prepare([[maybe_unused]] ContainerWindow &parent,
               [[maybe_unused]] const PixelRect &rc) noexcept override {
    AddButton(_("Details"), [this](){ list->OpenDetails(); });
    AddButton(_("Map"), [this](){ list->OpenMap(); });
    AddButton(_("Close"), dialog.MakeModalResultCallback(mrCancel));
  }
};

void
TrafficListWidget::UpdateList()
{
  assert(filter_widget != nullptr);

  items.clear();
  last_update.Clear();

  const char *callsign = filter_widget->GetValueString(CALLSIGN);
  if (!StringIsEmpty(callsign)) {
    FlarmId ids[30];
    unsigned count = FlarmDetails::FindIdsByCallSign(callsign, ids, 30);

    for (unsigned i = 0; i < count; ++i)
      AddItem(ids[i]);
  } else {
    /* if no filter was set, show a list of current traffic and known
       traffic */

    /* add live FLARM traffic */
    for (const auto &i : CommonInterface::Basic().flarm.traffic.list) {
      AddItem(i.id);
    }

    /* add FLARM peers that have a user-defined color */
    for (const auto &i : traffic_databases->flarm_colors) {
      Item &item = AddItem(i.first);
      item.color = i.second;
    }

    /* add FLARM peers that have a user-defined name */
    for (const auto &i : traffic_databases->flarm_names) {
      AddItem(i.id);
    }

#ifdef HAVE_SKYLINES_TRACKING
    /* show SkyLines traffic unless this is a FLARM traffic picker
       dialog (from dlgTeamCode) */
    if (buttons != nullptr && net_components != nullptr &&
        net_components->tracking) {
      const auto &data = net_components->tracking->GetSkyLinesData();
      const std::lock_guard lock{data.mutex};
      for (const auto &i : data.traffic) {
        const auto name_i = data.user_names.find(i.first);
        std::string name = name_i != data.user_names.end()
          ? name_i->second
          : std::string();

        items.emplace_back(i.first, i.second.time_of_day,
                           i.second.location, i.second.altitude,
                           std::move(name));
        Item &item = items.back();

        if (i.second.location.IsValid()) {
          if (CommonInterface::Basic().location_available)
            item.vector = GeoVector(CommonInterface::Basic().location,
                                    i.second.location);

          const auto wp = data_components->waypoints->GetNearestLandable(i.second.location,
                                                                         20000);
          if (wp != nullptr) {
            item.near_name = wp->name.c_str();
            item.near_distance = wp->location.DistanceS(i.second.location);
          }
        }
      }
    }
#endif
  }

  GetList().SetLength(items.size());

  UpdateVolatile();
  UpdateButtons();
}

void
TrafficListWidget::UpdateVolatile()
{
  const TrafficList &live_list = CommonInterface::Basic().flarm.traffic;

  bool modified = false;

  /* determine the most recent time stamp in the #TrafficList; this is
     used to set the new last_update value */
  Validity max_time;
  max_time.Clear();

  for (auto &i : items) {
    if (i.IsFlarm()) {
      const FlarmTraffic *live = live_list.FindTraffic(i.id);

      if (live != nullptr) {
        if (live->valid.Modified(last_update))
          /* if this #FlarmTraffic is newer than #last_update, then we
             need to redraw the list */
          modified = true;

        if (live->valid.Modified(max_time))
          /* update max_time (and last_update) for the next
             UpdateVolatile() call */
          max_time = live->valid;

        i.location = live->location;
        i.vector = GeoVector(live->distance, live->track);
      } else {
        if (i.location.IsValid() || i.vector.IsValid())
          /* this item has disappeared from our FLARM: redraw the
             list */
          modified = true;

        i.location.SetInvalid();
        i.vector.SetInvalid();
      }
#ifdef HAVE_SKYLINES_TRACKING
    } else if (i.IsSkyLines() && net_components != nullptr &&
               net_components->tracking) {
      const auto &data = net_components->tracking->GetSkyLinesData();
      const std::lock_guard lock{data.mutex};

      auto live = data.traffic.find(i.skylines_id);
      if (live != data.traffic.end()) {
        if (live->second.location != i.location)
          modified = true;

        i.location = live->second.location;

        if (i.location.IsValid() &&
            CommonInterface::Basic().location_available)
          i.vector = GeoVector(CommonInterface::Basic().location,
                               i.location);
      } else {
        if (i.location.IsValid() || i.vector.IsValid())
          /* this item has disappeared: redraw the list */
          modified = true;

        i.location.SetInvalid();
        i.vector.SetInvalid();
      }
#endif
    }
  }

  last_update = max_time;

  if (modified)
    GetList().Invalidate();
}

void
TrafficListWidget::UpdateButtons()
{
  if (buttons == nullptr)
    return;

  unsigned cursor = GetList().GetCursorIndex();
  bool valid_cursor = cursor < items.size();
  bool flarm_cursor = valid_cursor && items[cursor].IsFlarm();
  bool valid_location = valid_cursor && items[cursor].location.IsValid();

  buttons->SetRowEnabled(DETAILS, flarm_cursor);
  buttons->SetRowEnabled(MAP, valid_location);
}

void
TrafficListWidget::Prepare(ContainerWindow &parent,
                           const PixelRect &rc) noexcept
{
  const DialogLook &look = UIGlobals::GetDialogLook();
  ListControl &list = CreateList(parent, look, rc,
                                 row_renderer.CalculateLayout(*look.list.font_bold,
                                                              look.small_font));

  if (filter_widget != nullptr)
    UpdateList();
  else
    list.SetLength(items.size());
}

#ifdef HAVE_SKYLINES_TRACKING

/**
 * Calculate how many minutes have passed since #past_ms.
 */
[[gnu::const]]
static constexpr auto
SinceInMinutes(TimeStamp now,
               duration<uint32_t, milliseconds::period> past_ms) noexcept
{
  using Minutes = duration<unsigned, minutes::period>;
  constexpr Minutes ONE_DAY = hours{24};
  auto now_minutes = now.Cast<Minutes>() % ONE_DAY;
  auto past_minutes = duration_cast<Minutes>(past_ms) % ONE_DAY;

  if (past_minutes >= hours{20} && now_minutes < hours{4})
    /* midnight rollover */
    now_minutes += ONE_DAY;

  if (past_minutes > now_minutes)
    return Minutes{};

  return now_minutes - past_minutes;
}

#endif

void
TrafficListWidget::OnPaintItem(Canvas &canvas, PixelRect rc,
                               unsigned index) noexcept
{
  assert(index < items.size());
  Item &item = items[index];

  // MSVC don't like this Macro extension used in else branch
#ifdef HAVE_SKYLINES_TRACKING
  assert(item.IsFlarm() || item.IsSkyLines());
#else
  assert(item.IsFlarm());
#endif

  item.AutoLoad();

  const FlarmNetRecord *record = item.record;
  const char *callsign = item.callsign;

  const DialogLook &look = UIGlobals::GetDialogLook();
  const Font &name_font = *look.list.font_bold;
  const Font &small_font = look.small_font;

  const unsigned text_padding = Layout::GetTextPadding();
  const unsigned frame_padding = text_padding / 2;

  char tmp_id[10];
  item.id.Format(tmp_id);

  canvas.Select(name_font);

  StaticString<256> tmp;

  if (item.IsFlarm()) {
    if (record != nullptr)
      tmp.Format("%s - %s - %s",
                 callsign, record->registration.c_str(), tmp_id);
    else if (callsign != nullptr)
      tmp.Format("%s - %s", callsign, tmp_id);
    else
      tmp.Format("%s", tmp_id);
#ifdef HAVE_SKYLINES_TRACKING
  } else if (item.IsSkyLines()) {
    if (!item.name.empty())
      tmp = item.name.c_str();
    else
      tmp.UnsafeFormat("SkyLines %u", item.skylines_id);
#endif
  } else {
    tmp = "?";
  }

  if (item.color != FlarmColor::NONE) {
    const TrafficLook &traffic_look = UIGlobals::GetLook().traffic;

    switch (item.color) {
    case FlarmColor::NONE:
    case FlarmColor::COUNT:
      gcc_unreachable();

    case FlarmColor::GREEN:
      canvas.Select(traffic_look.team_pen_green);
      break;
    case FlarmColor::BLUE:
      canvas.Select(traffic_look.team_pen_blue);
      break;
    case FlarmColor::YELLOW:
      canvas.Select(traffic_look.team_pen_yellow);
      break;
    case FlarmColor::MAGENTA:
      canvas.Select(traffic_look.team_pen_magenta);
      break;
    }

    canvas.SelectHollowBrush();

    const PixelSize size = canvas.CalcTextSize(tmp);
    canvas.DrawRectangle(PixelRect{{rc.left + row_renderer.GetX(), rc.top + row_renderer.GetFirstY()}, size}.WithMargin(frame_padding));
  }

  row_renderer.DrawFirstRow(canvas, rc, tmp);

  canvas.Select(small_font);

  /* draw bearing and distance on the right */
  if (item.vector.IsValid()) {
    row_renderer.DrawRightFirstRow(canvas, rc,
                                            FormatUserDistanceSmart(item.vector.distance).c_str());

    // Draw leg bearing
    rc.right = row_renderer.DrawRightSecondRow(canvas, rc,
                                               FormatBearing(item.vector.bearing).c_str());
  }

  if (record != nullptr) {
    tmp.clear();

    if (!record->pilot.empty())
      tmp = record->pilot.c_str();

    if (!record->plane_type.empty()) {
      if (!tmp.empty())
        tmp.append(" - ");

      tmp.append(record->plane_type);
    }

    if (!record->airfield.empty()) {
      if (!tmp.empty())
        tmp.append(" - ");

      tmp.append(record->airfield);
    }

    if (!tmp.empty())
      row_renderer.DrawSecondRow(canvas, rc, tmp);
#ifdef HAVE_SKYLINES_TRACKING
  } else if (item.IsSkyLines()) {
    if (CommonInterface::Basic().time_available) {
      tmp.UnsafeFormat(_("%u minutes ago"),
                       SinceInMinutes(CommonInterface::Basic().time,
                                      item.time_of_day));
    } else
      tmp.clear();

    if (!item.near_name.empty())
      tmp.AppendFormat(" near %s (%s)",
                       item.near_name.c_str(),
                       FormatUserDistanceSmart(item.near_distance).c_str());

    if (!tmp.empty())
      tmp.append("; ");
    tmp.append(FormatUserAltitude(item.altitude).c_str());

    if (!tmp.empty())
      row_renderer.DrawSecondRow(canvas, rc, tmp);
#endif
  }
}

void
TrafficListWidget::OpenDetails(unsigned index)
{
  if (index >= items.size())
    return;

  Item &item = items[index];

  if (item.IsFlarm()) {
    dlgFlarmTrafficDetailsShowModal(item.id);
    UpdateList();
  }
}

void
TrafficListWidget::OpenMap(unsigned index)
{
  if (index >= items.size())
    return;

  Item &item = items[index];
  if (!item.location.IsValid())
    return;

  if (PanTo(item.location))
    dialog.SetModalResult(mrCancel);
}

void
TrafficListWidget::OnActivateItem(unsigned index) noexcept
{
  if (buttons == nullptr)
    /* it's a traffic picker: finish the dialog */
    dialog.SetModalResult(mrOK);
  else
    OpenDetails(index);
}

void
TrafficListDialog()
{
  const DialogLook &look = UIGlobals::GetDialogLook();

  WidgetDialog dialog(WidgetDialog::Full{}, UIGlobals::GetMainWindow(),
                      look, _("Traffic"));

  auto filter_widget = std::make_unique<TrafficFilterWidget>(look);

  auto buttons_widget = std::make_unique<TrafficListButtons>(look, dialog);

  auto list_widget =
    std::make_unique<TrafficListWidget>(dialog, *filter_widget,
                                        *buttons_widget);

  filter_widget->SetListener(list_widget.get());
  buttons_widget->SetList(list_widget.get());

  auto left_widget =
    std::make_unique<TwoWidgets>(std::move(filter_widget),
                                 std::move(buttons_widget),
                                 true);

  auto widget = std::make_unique<TwoWidgets>(std::move(left_widget),
                                             std::move(list_widget),
                                             false);

  dialog.FinishPreliminary(widget.release());
  dialog.ShowModal();
}

FlarmId
PickFlarmTraffic(const char *title, FlarmId array[], unsigned count)
{
  assert(count > 0);

  WidgetDialog dialog(WidgetDialog::Full{}, UIGlobals::GetMainWindow(),
                      UIGlobals::GetDialogLook(), title);

  TrafficListWidget *const list_widget =
    new TrafficListWidget(dialog, array, count);

  Widget *widget = list_widget;

  dialog.AddButton(_("Select"), mrOK);
  dialog.AddButton(_("Cancel"), mrCancel);
  dialog.EnableCursorSelection();
  dialog.FinishPreliminary(widget);

  return dialog.ShowModal() == mrOK
    ? list_widget->GetCursorId()
    : FlarmId::Undefined();
}
