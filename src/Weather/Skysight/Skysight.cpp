// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "Skysight.hpp"
#include "system/Path.hpp"
#include "system/FileUtil.hpp"
#include "util/StringCompare.hxx"
#include "util/Macros.hpp"
#include "util/StaticString.hxx"
#include "Profile/Profile.hpp"
#include "ActionInterface.hpp"
#include "system/FileUtil.hpp"
#include "Interface.hpp"
#include "UIGlobals.hpp"
#include "Language/Language.hpp"
#include "LogFile.hpp"
#include "MapWindow/OverlayBitmap.hpp"
#include "MapWindow/GlueMapWindow.hpp"
#include "thread/Debug.hpp"
#include "time/DateTime.hpp"
#include "Formatter/TimeFormatter.hpp"
#include "io/ZipArchive.hpp"

#include "MainWindow.hpp"

#include <string>
#include <vector>
#include <memory>
#include <thread>

/**
 * TODO(Caz, 2020):
 * -- overlay only shows following render -- no way to trigger from child 
 *    thread
 * -- no transparent bg on overlay on android
 *
 * --- for release ----
 * - Use SkysightImageFile elsewhere instead of recalculating forecast time,
 *   move to separate imp file
 * - clean up libs
 * - rebase on latest master, clean up
 * - move cache trimming to API?
 * - clean up layers/activelayers/displayed_layer -- inheritance rather
 *   than pointer
 * - Add documentation
 * - Test cubie compile / libs

 --- style ----
 * fix variable style/case,
 * reduce use of STL strings
 * Can use AtScopeExit for object cleanup in tiff generation
 * Use consistent string conventions ( _()? )
 * replace #defines in self.hpp with better c++ idioms
* Use static_cast<> instead of c casts
 */

Skysight *Skysight::self = nullptr;

/*
 * Img File
 */
SkysightImageFile::SkysightImageFile(Path _filename) {
  filename = _filename;
  fullpath = AllocatedPath::Build(Skysight::GetLocalPath(), filename);
  SkysightImageFile(filename, fullpath);
}

SkysightImageFile::SkysightImageFile(Path _filename, Path _path) { 
  filename = _filename;
  fullpath = _path;
  region = std::string(_("INVALID"));
  layer = std::string(_("INVALID"));
  datetime = 0;
  is_valid = false;
  mtime = 0;

  // images are in format region-layer-datetime.tif
  if (!filename.EndsWithIgnoreCase(".tif"))
    return;

  std::string file_base = filename.GetBase().c_str();

  std::size_t p = file_base.find(_("-"));
  if (p == std::string::npos)
    return;

  std::string reg = file_base.substr(0, p);
  std::string rem = file_base.substr(p+1);

  p = rem.find(_("-"));
  if (p == std::string::npos)
    return;
  std::string met = rem.substr(0, p);
  datetime = std::stoi(rem.substr(p + 1));
  mtime = File::GetTime(fullpath);

  region = reg;
  layer = met;
  is_valid = true;
}

/*
 * ******   SELECTED LAYERS ************
 */
bool
Skysight::SelectedLayersFull()
{
  assert(api);
  return (api->SelectedLayersFull());
}

bool
Skysight::LayerExists(const std::string_view id)
{
  assert(api);
  return api->LayerExists(id);
}

size_t
Skysight::AddSelectedLayer(const std::string_view id)
{
  assert(api);

  if (!api->LayerExists(id))
    return -3;  // layer don't exists

  if (api->IsSelectedLayer(id))
    return -1;  // layer is already loaded

  if (api->SelectedLayersFull())
    return -2;

  SkySight::Layer *m = api->GetLayer(id);
  if (m) {
    GetSelectedLayerState(id, *m);

    api->selected_layers.push_back(*m);
    SaveSelectedLayers();
  }
  return api->selected_layers.size() - 1;
}

void
Skysight::RefreshSelectedLayer(const std::string_view id)
{
  auto layer = api->GetLayer(id);
  if (layer)
    GetSelectedLayerState(id, *layer);

}

SkySight::Layer *
Skysight::GetSelectedLayer(int index)
{
  assert(index < (int)api->selected_layers.size());
  auto &layer = api->selected_layers.at(index);

  return &layer;
}

SkySight::Layer *
Skysight::GetSelectedLayer(const std::string_view id)
{
  for (auto &layer : api->selected_layers)
    if (layer.id == id)
      return &layer;

  return nullptr;
}

#if 1  // TODO(aug): possible not needed anymore...
void
Skysight::SetSelectedLayerUpdateState(const std::string_view id, bool state)
{
  auto layer = api->GetLayer(id);
  if (layer)
    layer->updating = state;
}
#endif

void
Skysight::RemoveSelectedLayer(size_t index)
{
  assert(index < api->selected_layers.size());
  api->selected_layers.erase(api->selected_layers.begin() + index);
  SaveSelectedLayers();
}

void
Skysight::RemoveSelectedLayer(const std::string_view id)
{
  for (std::vector<SkySight::Layer>::const_iterator iter = api->selected_layers.begin();
    iter < api->selected_layers.end(); ++iter) {
    if (iter->id == id) {
      api->selected_layers.erase(iter);
      break;
    }
  }

  SaveSelectedLayers();
}

bool
Skysight::SelectedLayersUpdating()
{
  for (auto &layer : api->selected_layers)
    if (layer.updating) return true;

  return false;
}

size_t
Skysight::NumSelectedLayers()
{
  return api->selected_layers.size();
}

void
Skysight::SaveSelectedLayers()
{
  std::string am_list;

  if (NumSelectedLayers()) {
    for(auto &layer: api->selected_layers) {
      am_list += layer.id;
      am_list += ",";
    }
    am_list.pop_back();
  } else {
    am_list = "";
  }
  Profile::Set(ProfileKeys::SkysightSelectedLayers, am_list.c_str());
}

void
Skysight::LoadSelectedLayers()
{
  const char *id = Profile::Get(ProfileKeys::SkysightSelectedLayers);
  if (id == nullptr)
    return;
  std::string am_list = std::string(id);
  size_t pos;

  api->selected_layers.clear();
  while ((pos = am_list.find(",")) != std::string::npos) {
    AddSelectedLayer(am_list.substr(0, pos).c_str());
    am_list.erase(0, pos + 1);
  }
  AddSelectedLayer(am_list.c_str()); // last one

  auto d = Profile::Get(ProfileKeys::WeatherLayerDisplayed);
  if (d == nullptr)
    return;

  if (!api->IsSelectedLayer(d))
    return;

  SetActiveLayer(d);
}

bool
Skysight::IsReady([[maybe_unused]] bool force_update)
{
  if (email.empty() || password.empty() || region.empty())
    return false;

  return (NumLayers() > 0);
}

Skysight::Skysight(CurlGlobal &_curl)
{
  self = this;
  curl = &_curl;
  Init();
}

void
Skysight::Init()
{
  if (api) {
    delete api;
    api = nullptr;
#if !defined(__GNUC__) || __GNUC__ >= 13
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
#endif
  }
  active_layer = nullptr;

#if defined(SKYSIGHT_FILE_DEBUG)
  // save in debug case an additional file in folder
  auto filename = AllocatedPath::Build(GetLocalPath(), 
    (DateTime::str_now() + " ====== Start-SkySight.tmp").c_str());
  if (File::CreateExclusive(filename))
      File::WriteExisting(filename, DateTime::str_now().c_str());
#endif
 
  const auto settings =
    CommonInterface::GetComputerSettings().weather.skysight;
  region = settings.region.c_str();
  email = settings.email.c_str();
  password = settings.password.c_str();

  api = new SkysightAPI(GetLocalPath());
  api->InitAPI(email, password, region, APIInited);
  CleanupFiles();
}

void
Skysight::APIInited([[maybe_unused]] const std::string details,
  [[maybe_unused]] const bool success,
  [[maybe_unused]] const std::string layer_id,
  [[maybe_unused]] const time_t time_index)
{
  if (!self)
    return;

  if (self->api) {
    self->LoadSelectedLayers();
    self->Render(true);
  }
}

// TODO(August2111): use layer_name or layer only...
bool
Skysight::GetSelectedLayerState(const std::string_view layer_name,
  [[maybe_unused]] SkySight::Layer &layer)
{
  std::string search_pattern = region + "-" + layer_name.data() + "*";
  std::vector<SkysightImageFile> img_files = ScanFolder(search_pattern);

  if (img_files.size() > 0) {
    time_t min_date = (time_t)std::numeric_limits<uint64_t>::max;
    time_t max_date = 0;
    time_t updated = 0;

    for (auto &i : img_files) {
      min_date = std::min(min_date, i.datetime);
      max_date = std::max(max_date, i.datetime);
      updated = std::max(updated, i.mtime);
    }
    auto m = GetLayer(layer_name);
    if (m) {
      m->from = min_date;
      m->to = max_date;
      m->mtime = updated;

      return true;
    }
  }

  return false;
}

std::vector<SkysightImageFile>
Skysight::ScanFolder(std::string search_string = "*.tif")
{
  // start by checking for output files
  std::vector<SkysightImageFile> file_list;

  struct SkysightFileVisitor: public File::Visitor {
    std::vector<SkysightImageFile> &file_list;
    explicit SkysightFileVisitor(std::vector<SkysightImageFile> &_file_list):
      file_list(_file_list) {}

    void Visit(Path path, Path filename) override {
      // is this a tif filename
      if (filename.EndsWithIgnoreCase(".tif")) {
        SkysightImageFile img_file = SkysightImageFile(filename, path);
        if (img_file.is_valid)
          file_list.emplace_back(img_file);
      }
    }

  } visitor(file_list);

  Directory::VisitSpecificFiles(GetLocalPath(), search_string.c_str(),
                visitor);
  return file_list;
}

void
Skysight::CleanupFiles()
{
  struct SkysightTIFVisitor: public File::Visitor {
    explicit SkysightTIFVisitor(const time_t _to): to(_to) {}
    const time_t to;
    void Visit(Path path, Path filename) override {
      SkysightImageFile img_file = SkysightImageFile(filename, path);
      if ((img_file.mtime < to) || (img_file.datetime < to ) ) {
        File::Delete(path);
      }
    }
  };

  struct SkysightFileDeleter: public File::Visitor {
    explicit SkysightFileDeleter(const time_t _to): to(_to) {}
    const time_t to;
    void Visit(Path fullpath, [[maybe_unused]] Path filename) override {
        auto mtime = File::GetTime(fullpath);
        if (mtime < to) { 
          File::Delete(fullpath);
        }
    }
  };
  char buffer[30];
  FormatISO8601(buffer, std::time(0));
  LogFmt("Time-Compare: {}, {} ({})", buffer, DateTime::str_now(), DateTime::now());
  
  auto now = DateTime::now();
  SkysightTIFVisitor  visitor_tif(now - 24 * ONE_HOUR);  // 1 day
  SkysightFileDeleter deleter_jpg(now - 12 * ONE_HOUR);  // 1/2 day
  SkysightFileDeleter deleter_tmp(now -  6 * ONE_HOUR);  // 6 hours
  SkysightFileDeleter deleter_txt(now -  ONE_HOUR);  // 1 hour

  auto path = GetLocalPath();  // local SkySight (cache) path
  Directory::VisitSpecificFiles(path, "*.tif", visitor_tif);
  Directory::VisitSpecificFiles(path, "*.jpg", deleter_jpg);
  Directory::VisitSpecificFiles(path, "*.tmp", deleter_tmp);
  // "*.txt;*.json" is not possible...
  Directory::VisitSpecificFiles(path, "*.txt", deleter_txt);
  Directory::VisitSpecificFiles(path, "*.json", deleter_txt);
}

void
Skysight::Render([[maybe_unused]] bool force_update)
{
  // das ist nur ein Test!!!!!!!!!!!!!!!!!!!!!
  // update_flag = true;

  if (active_layer) {
    if (update_flag) {  // set by download OnComplete
      DisplayActiveLayer();
    }
  }
}

time_t
Skysight::GetForecastTime(time_t time)
{
  auto forecast_time = (time / HALF_HOUR + 1) * HALF_HOUR;
  // is forecast_time plausible?
  return forecast_time;
}

bool
Skysight::SetActiveLayer(const std::string_view id,
                         time_t forecast_time)
{
  if (api->IsSelectedLayer(id)) {
    active_layer = api->GetLayer(id);
    if (active_layer) {
      active_layer->forecast_time = forecast_time;
      if (api) {
        api->ResetLastUpdate();
      }
      return true;
    }
  }
  return false;
}

#if 1  // TODO(aug): possible not needed anymore...
void
Skysight::DownloadComplete([[maybe_unused]] const std::string details,
  const bool success,  const std::string layer_id,
  [[maybe_unused]] const time_t time_index)
{
  if (!self)
    return;

  self->SetSelectedLayerUpdateState(layer_id, false);
  self->RefreshSelectedLayer(layer_id);

  if (success && (self->GetActiveLayerId() == layer_id.c_str())) {
    if (!self->update_flag) {
      self->update_flag = true;
      GlueMapWindow *map_window = UIGlobals::GetMapIfActive();
      if (map_window)
        map_window->DeferRedraw();

      // CommonInterface::main_window->SendGPSUpdate(); // which one is better?
      CommonInterface::main_window->SendCalculatedUpdate();
    }
  }
}
#endif

#ifdef SKYSIGHT_OFFLINE_MODE
bool
Skysight::DownloadSelectedLayer(const std::string_view id = "*")
{
  if (id == "*") {
    bool bret = true;
    for (auto &layer : api->selected_layers) {
      if (!layer.tile_layer)
        bret &= DownloadSelectedLayer(layer.id);
    }
    return bret;
  }  else {
    auto layer = GetLayer(id);
    if (layer && !layer->tile_layer) {
      time_t now = DateTime::now();
      SetSelectedLayerUpdateState(id, true);
#ifdef _DEBUG
      api->GetImageAt(id.data(), now, now + ONE_HOUR,
#else
      api->GetImageAt(id.data(), now, now + ONE_DAY,
#endif
        DownloadComplete);
      return true;
    }
  }
  return false;
}
#endif

void
Skysight::MapOverlayReset()
{
  auto *map = UIGlobals::GetMap();
  if (map)
    for (uint16_t i = 0; i < skysight_overlays; i++) 
      map->SetOverlay(i, nullptr);
}

void
Skysight::DeactivateLayer()
{
  active_layer = nullptr;
  MapOverlayReset();
  skysight_overlays = 1;
  Profile::Set(ProfileKeys::WeatherLayerDisplayed, "");
}

bool
Skysight::SetLayerActive(const std::string_view id)
{
  update_flag = false;

  if (id.empty()) {
    DeactivateLayer();
    return false;
  }

  if (!api->IsSelectedLayer(id))
    return false;

  Profile::Set(ProfileKeys::WeatherLayerDisplayed, id.data());

  if (!SetActiveLayer(id))
    return false;

  return true;
}

void Skysight::KeyIsNew() {
  if (api)
    // now you can get all data from server 
    api->TimerInvoke();
}

#ifdef SKYSIGHT_FORECAST 
bool
Skysight::DisplayForecastLayer()
{
  // TODO: We're only searching w/ a max offset of 1 hr, simplify this!
  AllocatedPath filename;
  bool found = false;

  if (skysight_overlays != 1 /*max_skysight_overlays*/) {
    MapOverlayReset();
    skysight_overlays = 1 /*max_skysight_overlays*/;
  }


  time_t test_time = DateTime::TimeRaster(DateTime::now() + TEN_MINUTES,
    HALF_HOUR, 1);

  // TODO(August2111): this procedure to find the best image I have to
  // analyze exactly!
  constexpr auto max_steps = 3;
  for (int j = 0; !found && (j < max_steps); j++) {
    filename = api->GetPath(SkysightCallType::Image, active_layer->id,
        test_time);

    if (!File::Exists(filename)) {
      auto zip_file = filename.WithSuffix(".zip");
      auto nc_file = filename.WithSuffix(".nc");
      if (!File::Exists(nc_file) &&
        File::Exists(zip_file))
        ZipIO::UnzipSingleFile(zip_file, nc_file);
      if (File::Exists(nc_file)) {
        char buffer[8];
        File::ReadString(nc_file, buffer, sizeof(buffer));  // read buffer
        if (strncmp(buffer, "CDF", 3) == 0) {
          // and now it is a CDF file
          api->CallCDFDecoder(active_layer, test_time,
            nc_file.c_str(), filename.c_str(),
            DownloadComplete);
          found = true;
        }
      }
    }
    if (File::Exists(filename)) {
      // needed for (selected) object view in map
      active_layer->forecast_time = test_time;
      if (UpdateActiveLayer(0, filename, { 0, 0, 0 })) {
        update_flag = false;  // is already updated
        return true;
      } else {
        return false;
      }
    } else {
      test_time += HALF_HOUR;  // next possible forecast
    }
  }
  return false;
}
#endif

bool
Skysight::UpdateActiveLayer(const uint32_t overlay_index, const Path &filename,
  GeoBitmap::TileData tile)
{
  if (!File::Exists(filename))
    return false;
  auto *map = UIGlobals::GetMap();
  if (map == nullptr)
    return false;

  LogFmt("SkySight::UpdateActiveLayer {}", filename.c_str());
  std::unique_ptr<MapOverlayBitmap> bmp;
  try {
    bmp.reset(new MapOverlayBitmap(filename));
    // what is with the new created MapOverlayBitmap? Where is deleting this?
  }
  catch (...) {
    LogError(std::current_exception(), "MapOverlayBitmap load error");
    return false;
  }

  if (!active_layer)
    return false;

  StaticString<256> label;
  label.Format("SkySight: %s", active_layer->name.c_str());
  if (active_layer->live_layer || !active_layer->tile_layer)
    label.AppendFormat("  (%s)", DateTime::time_str(
      active_layer->forecast_time, "%Y-%m-%d %H:%M").c_str());
  if (active_layer->tile_layer)
    label.AppendFormat(" - tile: %u (%u, %u)", tile.zoom, tile.x, tile.y);
  bmp->SetLabel(label);
  bmp->SetAlpha(active_layer->alpha);
  map->SetOverlay(overlay_index, std::move(bmp));
  // update_flag = false;  // is already updated
  return true;
}

bool
Skysight::DisplayTileLayer()
{
  GlueMapWindow *map_window = UIGlobals::GetMap();
  GeoBitmap::TileData base_tile;
  if (map_window && active_layer) { // && map_window->IsPanning()) {
    base_tile = GeoBitmap::GetTile(map_window->VisibleProjection(),
	  active_layer->zoom_min, active_layer->zoom_max);
  } else {
    return false;
  }

  if (skysight_overlays != max_skysight_overlays) {
    MapOverlayReset();
    skysight_overlays = max_skysight_overlays;
  }

  bool layer_changed = display_layer != active_layer;
  if (layer_changed || map_tile_zoom != base_tile.zoom) {
    if (api)
      api->TimerInvoke();
    map_tile_zoom = base_tile.zoom;
    if (layer_changed)
      display_layer = active_layer;
  }

  time_t refresh_time = (DateTime::now() / TEN_MINUTES) * TEN_MINUTES;

  auto map_bounds = map_window->VisibleProjection().GetScreenBounds();
  if (!map_bounds.Check() || !map_bounds.IsValid())
    return false;

  auto tile = base_tile;
  size_t tile_no = 0;
  for (tile.x = base_tile.x - 1; tile.x <= base_tile.x + 1; tile.x++)
    for (tile.y = base_tile.y - 1; tile.y <= base_tile.y + 1; tile.y++,
      tile_no++) {

      if (!GeoBitmap::GetBounds(tile).Overlaps(map_bounds)) {
        map_window->SetOverlay(tile_no, nullptr);
        tile_filenames[tile_no] = "";
        continue;
      }

      AllocatedPath filename;
      bool found = false;

      if (!active_layer->live_layer) { // osm
        filename = api->GetPath(SkysightCallType::Tile,
          active_layer->id, 0, tile);
        if (tile_filenames[tile_no] != filename.c_str()) {
          active_layer->forecast_time = 0;
          if (UpdateActiveLayer(tile_no, filename, tile))
            tile_filenames[tile_no] = filename.c_str();
        }
      } else {
        time_t test_time = refresh_time;
        constexpr auto max_steps = 3;

        for (int j = 0; !found && (j < max_steps); j++) {
          filename = api->GetPath(SkysightCallType::Tile, active_layer->id, test_time, tile);

          if (File::Exists(filename)) {
            // needed for (selected) object view in map
            if (tile_filenames[tile_no] != filename.c_str()) {
              active_layer->forecast_time = test_time;
              if (UpdateActiveLayer(tile_no, filename, tile))
                tile_filenames[tile_no] = filename.c_str();
            }
            break;
          } else {
            test_time -= TEN_MINUTES;  // previous live picture
          }
        }
      }
    }
  return true;
}

bool
Skysight::DisplayActiveLayer()
{
  if (!active_layer)
    return false;

  AllocatedPath filename;
  bool found = false;
  if (active_layer->tile_layer) {
    // no time stamp...
    found = DisplayTileLayer();
  } else if (active_layer->id.starts_with("osm")) {
    // no time stamp...
    filename = api->GetPath(SkysightCallType::Image, active_layer->id, 0);
    found = File::Exists(filename);
#ifdef SKYSIGHT_FORECAST 
  } else {
    // TODO: We're only searching w a max offset of 1 hr, simplify this!
    found = DisplayForecastLayer();
#endif  // SKYSIGHT_FORECAST 
  }
  if (found) {
    if (active_layer->forecast_time > 0) {
      active_layer->time_name = active_layer->name + ", ";
      active_layer->time_name += DateTime::time_str(
        active_layer->forecast_time, "%H:%M");
    } else {
      active_layer->time_name.clear();
    }
  }
  return found;
}
