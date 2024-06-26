// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

package de.opensoar;

/**
 * A listener that gets called when data is received on a socket.
 */
interface InputListener {
  void dataReceived(byte[] data, int length);
}
