// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "OpenVario/System/System.hpp"
#ifdef DBUS_FUNCTIONS
#include "lib/dbus/Connection.hxx"
#include "lib/dbus/ScopeMatch.hxx"
#include "lib/dbus/Systemd.hxx"
#endif

#include "system/Process.hpp"
#include "system/FileUtil.hpp"
#include "io/KeyValueFileReader.hpp"
#include "io/FileOutputStream.hxx"
#include "io/BufferedOutputStream.hxx"
#include "io/FileLineReader.hpp"
#include "Dialogs/Error.hpp"
#include "Hardware/RotateDisplay.hpp"

#include "Profile/File.hpp"
#include "Profile/Map.hpp"

#include "util/StaticString.hxx"
#include "util/ConvertString.hpp"

#include "OpenVario/System/System.hpp"

#ifndef _WIN32
#include <unistd.h>
#include <sys/stat.h>
#endif
#include <fmt/format.h>

#include <stdarg.h>

#include <map>
#include <string>
#include <iostream>

#ifndef __MSVC__
#include <unistd.h>
#include <sys/stat.h>
#endif
#include <fmt/format.h>

#include <map>

OpenVario_Device ovdevice;

OpenVario_Device::OpenVario_Device() {
  StaticString<0x100> home;
  home.SetUTF8(getenv("HOME"));
  home_path = Path(home);
#ifdef _WIN32
//  DataPath = Path(_T("D:/Data/OpenSoarData"));
  data_path = Path(_T("D:/Data/XCSoarData"));
#else
  data_path = Path(_T("data"));

  system_config =
      AllocatedPath::Build(Path(_T("/boot")), Path(_T("config.uEnf")));
  is_real = File::Exists(system_config);
#endif
  if (Directory::Exists(data_path)) {
    // auto config = AllocatedPath::Build(data_path, Path(_T("openvario.cfg")));
    settings_config =
        AllocatedPath::Build(data_path, Path(_T("openvario.cfg")));
    upgrade_config = AllocatedPath::Build(data_path, Path(_T("upgrade.cfg")));
  } else {
    settings_config =
        AllocatedPath::Build(home_path, Path(_T("openvario.cfg")));
    upgrade_config = AllocatedPath::Build(home_path, Path(_T("upgrade.cfg")));
  }
  system_config = AllocatedPath::Build(home_path, Path(_T("config.uEnv")));

  if (!File::Exists(settings_config))
    File::CreateExclusive(settings_config);

  assert(File::Exists(settings_config));
}
//----------------------------------------------------------
void ReadBool(std::map<std::string, std::string, std::less<>> &map,
              std::string_view name, bool &value) noexcept {
  if (map.find(name) != map.end())
    value = map.find(name)->second != "False";
}
//----------------------------------------------------------
void ReadInteger(std::map<std::string, std::string, std::less<>> &map,
                 std::string_view name, unsigned &value) noexcept {
  if (map.find(name) != map.end())
    value = std::stoul(map.find(name)->second);
}
//----------------------------------------------------------
void ReadString(std::map<std::string, std::string, std::less<>> &map,
                std::string_view name, std::string_view &value) noexcept {
  if (map.find(name) != map.end())
    value = map.find(name)->second;
}
//----------------------------------------------------------
void 
OpenVario_Device::LoadSettings() noexcept
{
  LoadConfigFile(system_map, GetSystemConfig());
  LoadConfigFile(settings, GetSettingsConfig());
  LoadConfigFile(upgrade_map, GetUpgradeConfig());

  ReadInteger(system_map, "rotation", rotation);
#ifdef _DEBUG
  std::string_view fdtfile;
  ReadString(system_map, "fdtfile", fdtfile);
#endif

  ReadBool(settings, "Enabled", enabled);
  ReadInteger(settings, "iTest", iTest);
  ReadInteger(settings, "Timeout", timeout);
  ReadInteger(settings, "Brightness", brightness);

}
//----------------------------------------------------------
void
LoadConfigFile(std::map<std::string, std::string, std::less<>> &map, Path path)
{
  if (File::Exists(path)) {
    FileLineReaderA reader(path);
    KeyValueFileReader kvreader(reader);
    KeyValuePair pair;
    while (kvreader.Read(pair))
      map.emplace(pair.key, pair.value);
  }
}

//----------------------------------------------------------
void
WriteConfigFile(std::map<std::string, std::string, std::less<>> &map, Path path)
{
  FileOutputStream file(path);
  BufferedOutputStream buffered(file);

  for (const auto &i : map)
    buffered.Fmt("{}={}\n", i.first, i.second);

  buffered.Flush();
  file.Commit();
}

//----------------------------------------------------------
/*/
void 
GetConfigInt(const std::string &keyvalue, unsigned &value,
                            const Path &config)
{
  if (File::Exists(config)) {
    ProfileMap configuration;
    Profile::LoadFile(configuration, config);
    configuration.Get(keyvalue.c_str(), value);
  } else {
    debugln("ConfigFile '%s' does not exist!", config.c_str());
  }
}

void 
ChangeConfigInt(const std::string &keyvalue, int value,
                            const Path &config)
{
  if (File::Exists(config)) {
    ProfileMap configuration;
    try {
      Profile::LoadFile(configuration, config);
    } catch (std::exception &e) {
      Profile::SaveFile(configuration, config);
    }
    configuration.Set(keyvalue.c_str(), value);
    Profile::SaveFile(configuration, config);
  } else {
    debugln("ConfigFile '%s' does not exist!", config.c_str());
  }
}
*/

//----------------------------------------------------------
uint_least8_t
OpenvarioGetBrightness() noexcept
{
  char line[4];
  int result = 10;

  if (File::ReadString(Path(_T("/sys/class/backlight/lcd/brightness")), line, sizeof(line))) {
    result = atoi(line);
  }

  return result;
}

void
OpenvarioSetBrightness(uint_least8_t value) noexcept
{
  if (value < 1) { value = 1; }
  if (value > 10) { value = 10; }

  File::WriteExisting(Path(_T("/sys/class/backlight/lcd/brightness")), fmt::format_int{value}.c_str());
}

DisplayOrientation
OpenvarioGetRotation()
{
  std::map<std::string, std::string, std::less<>> map;
  LoadConfigFile(map, Path(_T("/boot/config.uEnv")));

  uint_least8_t result;
  result = map.contains("rotation") ? std::stoi(map.find("rotation")->second) : 0;

  switch (result) {
  case 0: return DisplayOrientation::LANDSCAPE;
  case 1: return DisplayOrientation::REVERSE_PORTRAIT;
  case 2: return DisplayOrientation::REVERSE_LANDSCAPE;
  case 3: return DisplayOrientation::PORTRAIT;
  default: return DisplayOrientation::DEFAULT;
  }
}

void
OpenvarioSetRotation(DisplayOrientation orientation)
{
  std::map<std::string, std::string, std::less<>> map;

  Display::Rotate(orientation);

  int rotation = 0; 
  switch (orientation) {
  case DisplayOrientation::DEFAULT:
  case DisplayOrientation::LANDSCAPE:
    break;
  case DisplayOrientation::REVERSE_PORTRAIT:
    rotation = 1;
    break;
  case DisplayOrientation::REVERSE_LANDSCAPE:
    rotation = 2;
    break;
  case DisplayOrientation::PORTRAIT:
    rotation = 3;
    break;
  };

  File::WriteExisting(Path(_T("/sys/class/graphics/fbcon/rotate")), fmt::format_int{rotation}.c_str());

  LoadConfigFile(map, Path(_T("/boot/config.uEnv")));
  map.insert_or_assign("rotation", fmt::format_int{rotation}.c_str());
  WriteConfigFile(map, Path(_T("/boot/config.uEnv")));
}

#ifdef DBUS_FUNCTIONS
SSHStatus
OpenvarioGetSSHStatus()
{
  auto connection = ODBus::Connection::GetSystem();

  if (Systemd::IsUnitEnabled(connection, "dropbear.socket")) {
    return SSHStatus::ENABLED;
  } else if (Systemd::IsUnitActive(connection, "dropbear.socket")) {
    return SSHStatus::TEMPORARY;
  } else {
    return SSHStatus::DISABLED;
  }
}

void
OpenvarioEnableSSH(bool temporary)
{
  auto connection = ODBus::Connection::GetSystem();
  const ODBus::ScopeMatch job_removed_match{connection, Systemd::job_removed_match};

  if (temporary)
    Systemd::DisableUnitFile(connection, "dropbear.socket");
  else
    Systemd::EnableUnitFile(connection, "dropbear.socket");

  Systemd::StartUnit(connection, "dropbear.socket");
}

void
OpenvarioDisableSSH()
{
  auto connection = ODBus::Connection::GetSystem();
  const ODBus::ScopeMatch job_removed_match{connection, Systemd::job_removed_match};

  Systemd::DisableUnitFile(connection, "dropbear.socket");
  Systemd::StopUnit(connection, "dropbear.socket");
}
#endif  // DBUS_FUNCTIONS


#ifndef MAX_PATH
#define MAX_PATH 0x100
#endif
void debugln(const char *fmt, ...) noexcept {
  char buf[MAX_PATH];
  va_list ap;

  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf) - 1, fmt, ap);
  va_end(ap);

  std::cout << buf << std::endl;
}

//----------------------------------------------------------