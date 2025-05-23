set(_SOURCES
        MapWindow/GlueMapWindow.cpp
        MapWindow/GlueMapWindowDisplayMode.cpp
        MapWindow/GlueMapWindowEvents.cpp
        MapWindow/GlueMapWindowItems.cpp
        MapWindow/GlueMapWindowOverlays.cpp
        MapWindow/Items/AirspaceBuilder.cpp
        MapWindow/Items/Builder.cpp
        MapWindow/Items/List.cpp
        MapWindow/Items/MapItem.cpp
        MapWindow/Items/OverlayMapItem.cpp
        MapWindow/Items/TrafficBuilder.cpp
        MapWindow/Items/WeatherBuilder.cpp
        MapWindow/MapCanvas.cpp
        MapWindow/MapWindow.cpp
        MapWindow/MapWindowBlackboard.cpp
        MapWindow/MapWindowContest.cpp
        MapWindow/MapWindowEvents.cpp
        MapWindow/MapWindowGlideRange.cpp
        MapWindow/MapWindowRender.cpp
        MapWindow/MapWindowSymbols.cpp
        MapWindow/MapWindowTask.cpp
        MapWindow/MapWindowThermal.cpp
        MapWindow/MapWindowTraffic.cpp
        MapWindow/MapWindowTrail.cpp
        MapWindow/MapWindowWaypoints.cpp
        MapWindow/StencilMapCanvas.cpp
        MapWindow/TargetMapWindow.cpp
        MapWindow/TargetMapWindowDrag.cpp
        MapWindow/TargetMapWindowEvents.cpp
)
## if(UNIX) + WIN32!
  list(APPEND _SOURCES
        MapWindow/OverlayBitmap.cpp
  )
## endif()

set(SCRIPT_FILES
    CMakeSource.cmake
)
