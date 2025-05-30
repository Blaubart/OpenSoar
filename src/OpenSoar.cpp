// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

/**
 * This is the main entry point for the application
 * @file OpenSoar.cpp
 */

#include "Startup.hpp"
#include "LocalPath.hpp"
#include "Version.hpp"
#include "LogFile.hpp"
#include "CommandLine.hpp"
#include "MainWindow.hpp"
#include "Interface.hpp"
#include "Look/GlobalFonts.hpp"
#include "ui/window/Init.hpp"
#include "net/http/Init.hpp"
#include "ResourceLoader.hpp"
#include "Language/Language.hpp"
#include "Language/LanguageGlue.hpp"
#include "Simulator.hpp"
#include "Audio/GlobalPCMMixer.hpp"
#include "Audio/GlobalPCMResourcePlayer.hpp"
#include "Audio/GlobalVolumeController.hpp"
#include "system/Args.hpp"
#include "io/async/GlobalAsioThread.hpp"
#include "io/async/AsioThread.hpp"
#include "util/PrintException.hxx"

#include "UIGlobals.hpp"
#include "system/Process.hpp"

// see MinGW.cmake: #define DEBUG_CONSOLE_OUTPUT
#ifdef DEBUG_CONSOLE_OUTPUT
# include <iostream>
#endif

#ifdef ENABLE_SDL
/* this is necessary on Mac OS X, to let libSDL bootstrap Quartz
   before entering our main() */
#include <SDL_main.h>
#endif

#ifdef __APPLE__
#include <TargetConditionals.h>
#if !TARGET_OS_IPHONE
#import <AppKit/AppKit.h>
#endif
#endif

#include <cassert>

static const char *const Usage = "\n"
  "  -datapath=      path to XCSoar/OpenSoar data can be defined\n"
#ifdef SIMULATOR_AVAILABLE
  "  -simulator      bypass startup-screen, use simulator mode directly\n"
  "  -fly            bypass startup-screen, use fly mode directly\n"
#endif
  "  -profile=fname  load profile from file fname\n"
  "  -WIDTHxHEIGHT   use screen resolution WIDTH x HEIGHT\n"
  "  -portrait       use a 480x640 screen resolution\n"
  "  -square         use a 480x480 screen resolution\n"
  "  -small          use a 320x240 screen resolution\n"
#if !defined(ANDROID)
  "  -dpi=DPI        force usage of DPI for pixel density\n"
  "  -dpi=XDPIxYDPI  force usage of XDPI and YDPI for pixel density\n"
#endif
#ifdef HAVE_CMDLINE_FULLSCREEN
  "  -fullscreen     full-screen mode\n"
#endif
#ifdef HAVE_CMDLINE_RESIZABLE
  "  -resizable      resizable window\n"
#endif
#ifdef _WIN32
  "  -console        open debug output console\n"
#endif
  ;

static int Main() {
  ScreenGlobalInit screen_init;

#if defined(__APPLE__) && !TARGET_OS_IPHONE
  // We do not want the ugly non-localized main menu which SDL creates
  [NSApp setMainMenu: [[NSMenu alloc] init]];
#endif

#ifdef _WIN32
  /* try to make the UI most responsive */
  SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
#endif

  AllowLanguage();
  InitLanguage();

  ScopeGlobalAsioThread global_asio_thread;
  const Net::ScopeInit net_init(asio_thread->GetEventLoop());

#ifdef EXTERNAL_AUDIO_INIT
  InitAudio(&asio_thread->GetEventLoop());
  // How I get this variables to live up to the end?
#else
  ScopeGlobalPCMMixer global_pcm_mixer(asio_thread->GetEventLoop());
  ScopeGlobalPCMResourcePlayer global_pcm_resouce_player;
  ScopeGlobalVolumeController global_volume_controller;
#endif

  // Perform application initialization and run loop
  int ret = EXIT_FAILURE;
  if (Startup(screen_init.GetDisplay()))
    ret = CommonInterface::main_window->RunEventLoop();

  Shutdown();

#ifdef EXTERNAL_AUDIO_INIT
  ShutdownAudio();
#endif

  DisallowLanguage();

  Fonts::Deinitialize();

  DeinitialiseDataPath();

  return ret;
}

static int 
Finishing(int ret) {
  LogString("Finishing");
  switch (ret) {
  case EXIT_REBOOT:
#ifdef IS_OPENVARIO
    Run("/sbin/reboot");
    break;
#endif
    return ret;
  case EXIT_SHUTDOWN:
#ifdef IS_OPENVARIO
    Run("/sbin/poweroff");
    break;
#endif
    return ret;
  case EXIT_SYSTEM:
  default:
    break;
  }
  return ret;
}

/**
 * Main entry point for the whole OpenSoar application
 */
#ifndef _WIN32
int main(int argc, char **argv)
#else
int WINAPI
WinMain(HINSTANCE hInstance, [[maybe_unused]] HINSTANCE hPrevInstance,
  [[maybe_unused]] LPSTR lpCmdLine2,
  [[maybe_unused]] int nCmdShow)
#endif

try {
#ifdef DEBUG_CONSOLE_OUTPUT
  std::cout << "Start: OpenSoar!" << std::endl;
#endif
  // Read options from the command line
  int ret = -1;
  bool rerun = false;
  do {
    {
      rerun = false;
      // UI::TopWindow::SetExitValue(0);

#ifdef _WIN32
      if (UIGlobals::CommandLine == nullptr)
        UIGlobals::CommandLine = GetCommandLine();
      Args args(UIGlobals::CommandLine, Usage);
#else
      Args args(argc, argv, Usage);
#endif
      CommandLine::Parse(args);
    }

    InitialiseDataPath();

#ifdef USE_WIN32_RESOURCES
    if (!ResourceLoader::Initialized())
       ResourceLoader::Init(hInstance);
#endif
    // Write startup note + version to logfile
    LogFormat("Starting OpenSoar %s", OpenSoar_ProductToken);

    // int
    ret = Main();

#if defined(__APPLE__) && TARGET_OS_IPHONE
    /* For some reason, the app process does not exit on iOS, but a black
     * screen remains, if the process is not explicitly terminated */
    exit(ret);
#endif

#ifdef IS_OPENVARIO
    if (ret == 0)
      ret = UI::TopWindow::GetExitValue();
#endif
    rerun = (ret == EXIT_RESTART);
    if (rerun)
      UI::TopWindow::SetExitValue(0);
  } while (rerun);
  return Finishing(ret);
} catch (...) {
  PrintException(std::current_exception());
  return EXIT_FAILURE;
}
