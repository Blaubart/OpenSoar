// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "Internal.hpp"
#include "Protocol.hpp"
#include "Device/RecordedFlight.hpp"
#include "Device/Port/Port.hpp"
#include "system/Path.hpp"
#include "Operation/Operation.hpp"
#include "vlconv.h"
#include "grecord.h"

/**
 * Sizes of VL memory regions
 */
static constexpr size_t VLAPI_LOG_MEMSIZE = 81920L;

static bool
ConvertDirectoryToRecordedFlightList(const std::vector<DIRENTRY> &dir,
                                     RecordedFlightList &flight_list)
{
  RecordedFlightInfo flight_info;
  for (unsigned i=0; (i < dir.size()) && !flight_list.full(); i++) {
    const DIRENTRY &flight = dir[i];
    /*
     * Only show logs with a takeoff detected
     */
    if  (flight.takeoff == 1) {
      if (!flight.firsttime.IsPlausible() || !flight.lasttime.IsPlausible())
        return false;
      flight_info.date = {flight.firsttime.year,
                          flight.firsttime.month,
                          flight.firsttime.day};
      flight_info.start_time = {flight.firsttime.hour,
                                flight.firsttime.minute,
                                flight.firsttime.second};
      flight_info.end_time = {flight.lasttime.hour,
                              flight.lasttime.minute,
                              flight.lasttime.second};
      flight_info.internal.volkslogger = i;
      flight_list.append(flight_info);
    }
  }
  return true;
}

static bool
ReadFlightListInner(Port &port,
                    RecordedFlightList &flight_list,
                    OperationEnvironment &env)
{
  env.SetProgressRange(10);
  Volkslogger::ConnectAndFlush(port, env, std::chrono::seconds(20));
  env.SetProgressPosition(3);

  uint8_t dirbuffer[VLAPI_LOG_MEMSIZE];
  int data_length = Volkslogger::ReadFlightList(port, env,
                                                dirbuffer, sizeof(dirbuffer));
  if (data_length <= 0)
    return data_length == 0;

  std::vector<DIRENTRY> directory;
  if (!conv_dir(directory, dirbuffer, data_length))
    return false;

  if (directory.empty())
    return true;

  env.SetProgressPosition(8);
  if (!ConvertDirectoryToRecordedFlightList(directory, flight_list))
    return false;
  env.SetProgressPosition(10);

  return true;
}

static bool
DownloadFlightInner(Port &port, unsigned bulkrate,
                    const RecordedFlightInfo &flight,
                    Path path,
                    OperationEnvironment &env)
{
  Volkslogger::ConnectAndFlush(port, env, std::chrono::seconds(20));

  uint8_t logbuffer[VLAPI_LOG_MEMSIZE];
  const size_t length = Volkslogger::ReadFlight(port, bulkrate, env,
                                                flight.internal.volkslogger,
                                                true,
                                                logbuffer, sizeof(logbuffer));
  if (length == 0)
    return false;

  FILE *outfile = fopen(path.c_str(), "wt");
  if (outfile == nullptr)
    return false;

  size_t r = convert_gcs(outfile, logbuffer, length, true);
  if (r == 0) {
    fclose(outfile);
    return false;
  }

  print_g_record(outfile,   // output to stdout
                 logbuffer, // binary file is in buffer
                 r          // length of binary file to include
                 );
  fclose(outfile);
  return true;
}

bool
VolksloggerDevice::ReadFlightList(RecordedFlightList &flight_list,
                                  OperationEnvironment &env)
{
  port.StopRxThread();

  // change to IO mode baud rate
  unsigned old_baud_rate = port.GetBaudrate();
  if (old_baud_rate == 9600)
    old_baud_rate = 0;
  else if (old_baud_rate != 0)
    port.SetBaudrate(9600);

  bool success = ReadFlightListInner(port, flight_list, env);

  // restore baudrate
  if (old_baud_rate != 0)
    port.SetBaudrate(old_baud_rate);

  return success;
}

bool
VolksloggerDevice::DownloadFlight(const RecordedFlightInfo &flight,
                                  Path path,
                                  OperationEnvironment &env)
{
  port.StopRxThread();

  // change to IO mode baud rate
  unsigned old_baud_rate = port.GetBaudrate();
  if (old_baud_rate == 9600)
    old_baud_rate = 0;
  else if (old_baud_rate != 0)
    port.SetBaudrate(9600);

  bool success = DownloadFlightInner(port, bulkrate,
                                     flight, path, env);
  // restore baudrate
  if (old_baud_rate != 0)
     port.SetBaudrate(old_baud_rate);

  return success;
}
