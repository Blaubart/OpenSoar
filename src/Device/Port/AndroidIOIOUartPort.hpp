// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "util/Compiler.h"

#include <memory>

class Port;
class PortListener;
class DataHandler;
class IOIOHelper;

namespace AndroidIOIOUartPort
{
  static inline unsigned getNumberUarts() { return 4; }

  static inline const char *getPortHelp(unsigned UartID) {
    switch (UartID) {
    case 0:
      return "IOIO external board Uart: pin3=out, pin4=in";
    case 1:
      return "IOIO external board Uart: pin5=out, pin6=in";
    case 2:
      return "IOIO external board Uart: pin10=out, pin11=in";
    case 3:
      return "IOIO external board Uart: pin12=out, pin13=in";
    default:
      return "Illegal IOIO Uart ID";
    }
  }
}

std::unique_ptr<Port>
OpenAndroidIOIOUartPort(IOIOHelper &ioio_helper,
                        unsigned uart_id, unsigned baud_rate,
                        PortListener *listener, DataHandler &handler);
