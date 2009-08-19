/*
Copyright_License {

  XCSoar Glide Computer - http://xcsoar.sourceforge.net/
  Copyright (C) 2000 - 2008

  	M Roberts (original release)
	Robin Birch <robinb@ruffnready.co.uk>
	Samuel Gisiger <samuel.gisiger@triadis.ch>
	Jeff Goodenough <jeff@enborne.f2s.com>
	Alastair Harrison <aharrison@magic.force9.co.uk>
	Scott Penrose <scottp@dd.com.au>
	John Wharington <jwharington@gmail.com>
	Lars H <lars_hn@hotmail.com>
	Rob Dunning <rob@raspberryridgesheepfarm.com>
	Russell King <rmk@arm.linux.org.uk>
	Paolo Ventafridda <coolwind@email.it>
	Tobias Lohner <tobias@lohner-net.de>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

}
*/
#if !defined(AFX_UTILS_H__695AAC30_F401_4CFF_9BD9_FE62A2A2D0D2__INCLUDED_)
#define AFX_UTILS_H__695AAC30_F401_4CFF_9BD9_FE62A2A2D0D2__INCLUDED_

#include <windows.h>
#include <shlobj.h>
#include <math.h>
#include "Task.h"
#include "Airspace.h"
#include <zzip/lib.h>


#define  POLARUSEWINPILOTFILE  6    // if this polat is selected use the winpilot file

extern const TCHAR szRegistryKey[];
extern const TCHAR szRegistrySpeedUnitsValue[];
extern const TCHAR szRegistryDistanceUnitsValue[];
extern const TCHAR szRegistryAltitudeUnitsValue[];
extern const TCHAR szRegistryLiftUnitsValue[];
extern const TCHAR szRegistryTaskSpeedUnitsValue[];
extern const TCHAR szRegistryDisplayUpValue[];
extern const TCHAR szRegistryDisplayText[];
extern const TCHAR szRegistrySafetyAltitudeArrival[];
extern const TCHAR szRegistrySafetyAltitudeBreakOff[];
extern const TCHAR szRegistrySafetyAltitudeTerrain[];
extern const TCHAR szRegistrySafteySpeed[];
extern const TCHAR szRegistryFAISector[];
extern const TCHAR szRegistrySectorRadius[];
extern const TCHAR szRegistryPolarID[];
extern const TCHAR szRegistryWayPointFile[];
extern const TCHAR szRegistryAdditionalWayPointFile[];
extern const TCHAR szRegistryAirspaceFile[];
extern const TCHAR szRegistryAdditionalAirspaceFile[];
extern const TCHAR szRegistryAirfieldFile[];
extern const TCHAR szRegistryTopologyFile[];
extern const TCHAR szRegistryPolarFile[];
extern const TCHAR szRegistryTerrainFile[];
extern const TCHAR szRegistryLanguageFile[];
extern const TCHAR szRegistryStatusFile[];
extern const TCHAR szRegistryInputFile[];
extern const TCHAR szRegistryAltMode[];
extern const TCHAR szRegistryClipAlt[];
extern const TCHAR szRegistryAltMargin[];
extern const TCHAR szRegistryRegKey[];
extern const TCHAR szRegistrySnailTrail[];
extern const TCHAR szRegistryDrawTopology[];
extern const TCHAR szRegistryDrawTerrain[];
extern const TCHAR szRegistryFinalGlideTerrain[];
extern const TCHAR szRegistryAutoWind[];
extern const TCHAR szRegistryStartLine[];
extern const TCHAR szRegistryStartRadius[];
extern const TCHAR szRegistryFinishLine[];
extern const TCHAR szRegistryFinishRadius[];
extern const TCHAR szRegistryAirspaceWarning[];
extern const TCHAR szRegistryAirspaceBlackOutline[];
extern const TCHAR szRegistryWarningTime[];
extern const TCHAR szRegistryAcknowledgementTime[];
extern const TCHAR szRegistryCircleZoom[];
extern const TCHAR szRegistryWindUpdateMode[];
extern const TCHAR szRegistryHomeWaypoint[];
extern const TCHAR szRegistryAlternate1[];         // VENTA3
extern const TCHAR szRegistryAlternate2[];
extern const TCHAR szRegistryTeamcodeRefWaypoint[];
extern const TCHAR szRegistryPilotName[];
extern const TCHAR szRegistryAircraftType[];
extern const TCHAR szRegistryAircraftRego[];
extern const TCHAR szRegistryLoggerID[];
extern const TCHAR szRegistryLoggerShort[];
extern const TCHAR szRegistryNettoSpeed[];
extern const TCHAR szRegistryCDICruise[];
extern const TCHAR szRegistryCDICircling[];
extern const TCHAR szRegistryAutoBlank[];
extern const TCHAR szRegistryAutoBacklight[]; // VENTA4
extern const TCHAR szRegistryAutoSoundVolume[]; // VENTA4
extern const TCHAR szRegistryExtendedVisualGlide[]; // VENTA4
extern const TCHAR szRegistryVirtualKeys[]; // VENTA5
extern const TCHAR szRegistryAverEffTime[]; // VENTA6
extern const TCHAR szRegistryVarioGauge[];
extern const TCHAR szRegistryDebounceTimeout[];
extern const TCHAR szRegistryAppDefaultMapWidth[];
extern const TCHAR szRegistryAppIndFinalGlide[];
extern const TCHAR szRegistryAppIndLandable[];
extern const TCHAR szRegistryAppInverseInfoBox[];
extern const TCHAR szRegistryAppInfoBoxColors[];
extern const TCHAR szRegistryAppGaugeVarioSpeedToFly[];
extern const TCHAR szRegistryAppGaugeVarioAvgText[];
extern const TCHAR szRegistryAppGaugeVarioMc[];
extern const TCHAR szRegistryAppGaugeVarioBugs[];
extern const TCHAR szRegistryAppGaugeVarioBallast[];
extern const TCHAR szRegistryAppGaugeVarioGross[];
extern const TCHAR szRegistryAppCompassAppearance[];
extern const TCHAR szRegistryAppStatusMessageAlignment[];
extern const TCHAR szRegistryAppTextInputStyle[];
extern const TCHAR szRegistryAppInfoBoxBorder[];
#if defined(PNA) || defined(FIVV)
extern const TCHAR szRegistryAppInfoBoxGeom[];   // VENTA-ADDON GEOM CHANGE
extern const TCHAR szRegistryAppInfoBoxModel[]; // VENTA-ADDON MODEL CHANGE
#endif
extern const TCHAR szRegistryAppAveNeedle[];
extern const TCHAR szRegistryAutoAdvance[];
extern const TCHAR szRegistryUTCOffset[];
extern const TCHAR szRegistryBlockSTF[];
extern const TCHAR szRegistryAutoZoom[];
extern const TCHAR szRegistryMenuTimeout[];
extern const TCHAR szRegistryLockSettingsInFlight[];
extern const TCHAR szRegistryTerrainContrast[];
extern const TCHAR szRegistryTerrainBrightness[];
extern const TCHAR szRegistryTerrainRamp[];
extern const TCHAR szRegistryEnableFLARMMap[];
extern const TCHAR szRegistryEnableFLARMGauge[];
extern const TCHAR szRegistrySnailTrail[];
extern const TCHAR szRegistryTrailDrift[];
extern const TCHAR szRegistryThermalLocator[];
extern const TCHAR szRegistryGliderScreenPosition[];
extern const TCHAR szRegistryAnimation[];
extern const TCHAR szRegistrySetSystemTimeFromGPS[];
extern const TCHAR szRegistryAutoForceFinalGlide[];

extern const TCHAR szRegistryVoiceClimbRate[];
extern const TCHAR szRegistryVoiceTerrain[];
extern const TCHAR szRegistryVoiceWaypointDistance[];
extern const TCHAR szRegistryVoiceTaskAltitudeDifference[];
extern const TCHAR szRegistryVoiceMacCready[];
extern const TCHAR szRegistryVoiceNewWaypoint[];
extern const TCHAR szRegistryVoiceInSector[];
extern const TCHAR szRegistryVoiceAirspace[];

extern const TCHAR szRegistryFinishMinHeight[];
extern const TCHAR szRegistryStartMaxHeight[];
extern const TCHAR szRegistryStartMaxHeightMargin[];
extern const TCHAR szRegistryStartMaxSpeed[];
extern const TCHAR szRegistryStartMaxSpeedMargin[];
extern const TCHAR szRegistryStartHeightRef[];

extern const TCHAR szRegistryEnableNavBaroAltitude[];

extern const TCHAR szRegistryLoggerTimeStepCruise[];
extern const TCHAR szRegistryLoggerTimeStepCircling[];

extern const TCHAR szRegistrySafetyMacCready[];
extern const TCHAR szRegistryAbortSafetyUseCurrent[];
extern const TCHAR szRegistryAutoMcMode[];
extern const TCHAR szRegistryWaypointsOutOfRange[];
extern const TCHAR szRegistryEnableExternalTriggerCruise[];
extern const TCHAR szRegistryFAIFinishHeight[];
extern const TCHAR szRegistryOLCRules[];
extern const TCHAR szRegistryHandicap[];
extern const TCHAR szRegistrySnailWidthScale[];
extern const TCHAR szRegistryLatLonUnits[];
extern const TCHAR szRegistryUserLevel[];
extern const TCHAR szRegistryRiskGamma[];
extern const TCHAR szRegistryWindArrowStyle[];
extern const TCHAR szRegistryDisableAutoLogger[];
extern const TCHAR szRegistryMapFile[];
extern const TCHAR szRegistryBallastSecsToEmpty[];
extern const TCHAR szRegistryAccelerometerZero[];
extern const TCHAR szRegistryUseCustomFonts[];
extern const TCHAR szRegistryFontInfoWindowFont[];
extern const TCHAR szRegistryFontTitleWindowFont[];
extern const TCHAR szRegistryFontMapWindowFont[];
extern const TCHAR szRegistryFontTitleSmallWindowFont[];
extern const TCHAR szRegistryFontMapWindowBoldFont[];
extern const TCHAR szRegistryFontCDIWindowFont[];
extern const TCHAR szRegistryFontMapLabelFont[];
extern const TCHAR szRegistryFontStatisticsFont[];

extern bool LockSettingsInFlight;
extern bool LoggerShortName;

BOOL GetFromRegistry(const TCHAR *szRegValue, DWORD *pPos);

#ifdef FIVV
BOOL DelRegistryKey(const TCHAR *szRegistryKey); // VENTA2-ADDON delregistrykey
#endif
#ifdef PNA
void CleanRegistry(); // VENTA2-ADDON cleanregistrykeyA
bool SetBacklight(); // VENTA4-ADDON for PNA
bool SetSoundVolume(); // VENTA4-ADDON for PNA
#endif

HRESULT SetToRegistry(const TCHAR *szRegValue, DWORD Pos);
HRESULT SetToRegistry(const TCHAR *szRegValue, bool bVal);	// JG
HRESULT SetToRegistry(const TCHAR *szRegValue, int nVal);	// JG
BOOL GetRegistryString(const TCHAR *szRegValue, TCHAR *pPos, DWORD dwSize);
HRESULT SetRegistryString(const TCHAR *szRegValue, const TCHAR *Pos);
void ReadRegistrySettings(void);
void SetRegistryColour(int i, DWORD c);
void SetRegistryBrush(int i, DWORD c);
void SetRegistryAirspacePriority(int i);
void SetRegistryAirspaceMode(int i);
int GetRegistryAirspaceMode(int i);
void StoreType(int Index,int InfoType);
void irotate(int &xin, int &yin, const double &angle);
void irotatescale(int &xin, int &yin, const double &angle, const double &scale,
                  double &x, double &y);
void protate(POINT &pin, const double &angle);
void protateshift(POINT &pin, const double &angle, const int &x, const int &y);
void rotate(double &xin, double &yin, const double &angle);
void frotate(float &xin, float &yin, const float &angle);
void rotatescale(double &xin, double &yin, const double &angle, const double &scale);
void frotatescale(float &xin, float &yin, const float &angle, const float &scale);

void DistanceBearing(double lat1, double lon1, double lat2, double lon2,
                     double *Distance, double *Bearing);
double DoubleDistance(double lat1, double lon1, double lat2, double lon2,
		      double lat3, double lon3);

double Reciprocal(double InBound);
double BiSector(double InBound, double OutBound);
double HalfAngle(double Angle0, double Angle1);
void SectorEndPoint(double StartLat, double StartLon, double  Radial, double Dist, double *EndLat, double *EndLon);
void CalculateNewPolarCoef(void);
void FindLatitudeLongitude(double Lat, double Lon,
                           double Bearing, double Distance,
                           double *lat_out, double *lon_out);
void ConvertFlightLevels(void);
BOOL PolygonVisible(const POINT *lpPoints, int nCount, RECT rc);
void ReadPort1Settings(DWORD *PortIndex, DWORD *SpeedIndex);
void ReadPort2Settings(DWORD *PortIndex, DWORD *SpeedIndex);
void ReadPort3Settings(DWORD *PortIndex, DWORD *SpeedIndex);
void WritePort1Settings(DWORD PortIndex, DWORD SpeedIndex);
void WritePort2Settings(DWORD PortIndex, DWORD SpeedIndex);
void WritePort3Settings(DWORD PortIndex, DWORD SpeedIndex);
int  Circle(HDC hdc, long x, long y, int radius, RECT rc, bool clip=false,
            bool fill=true);
int Segment(HDC hdc, long x, long y, int radius, RECT rc,
	    double start,
	    double end,
            bool horizon= false);
// VENTA3 DrawArc
int DrawArc(HDC hdc, long x, long y, int radius, RECT rc,
	    double start,
	    double end);
void ReadAssetNumber(void);
void WriteProfile(const TCHAR *szFile);
void ReadProfile(const TCHAR *szFile);
#if defined(PNA) || defined(FIVV)  // VENTA-ADDON
void SetModelType();
bool SetModelName(DWORD Temp);
#endif
double ScreenAngle(int x1, int y1, int x2, int y2);
void ReadCompaqID(void);
void ReadUUID(void);
void FormatWarningString(int Type, TCHAR *Name , AIRSPACE_ALT Base, AIRSPACE_ALT Top, TCHAR *szMessageBuffer, TCHAR *TileBuffer );
BOOL ReadString(ZZIP_FILE* zFile, int Max, TCHAR *String);
BOOL ReadString(HANDLE hFile, int Max, TCHAR *String);
BOOL ReadStringX(FILE *fp, int Max, TCHAR *String);

// Fast trig functions
void InitSineTable(void);

extern double COSTABLE[4096];
extern double SINETABLE[4096];
extern double INVCOSINETABLE[4096];
extern int ISINETABLE[4096];
extern int ICOSTABLE[4096];

bool AngleInRange(double Angle0, double Angle1, double x, bool is_signed=false);
double AngleLimit180(double theta);
double AngleLimit360(double theta);

#ifdef __MINGW32__
#define DEG_TO_INT(x) ((unsigned short)(int)((x)*(65536.0/360.0)))>>4
#else
#define DEG_TO_INT(x) ((unsigned short)((x)*(65536.0/360.0)))>>4
#endif

#define invfastcosine(x) INVCOSINETABLE[DEG_TO_INT(x)]
#define ifastcosine(x) ICOSTABLE[DEG_TO_INT(x)]
#define ifastsine(x) ISINETABLE[DEG_TO_INT(x)]
#define fastcosine(x) COSTABLE[DEG_TO_INT(x)]
#define fastsine(x) SINETABLE[DEG_TO_INT(x)]

double StrToDouble(const TCHAR *Source, TCHAR **Stop);
void PExtractParameter(TCHAR *Source, TCHAR *Destination, int DesiredFieldNumber);
void SaveWindToRegistry();
void LoadWindFromRegistry();
void SaveSoundSettings();
void ReadDeviceSettings(const int devIdx, TCHAR *Name);
void WriteDeviceSettings(const int devIdx, const TCHAR *Name);

unsigned int isqrt4(unsigned long val);


WORD crcCalc(void *Buffer, size_t size);
void ExtractDirectory(TCHAR *Dest, TCHAR *Source);
double DoSunEphemeris(double lon, double lat);

void *bsearch(void *key, void *base0, size_t nmemb, size_t size, int (*compar)(const void *elem1, const void *elem2));
TCHAR *strtok_r(TCHAR *s, TCHAR *delim, TCHAR **lasts);


void ResetInfoBoxes(void);

void SaveRegistryToFile(const TCHAR* szFile);
void LoadRegistryFromFile(const TCHAR* szFile);

/* =====================================================
   Interface Files !
   ===================================================== */

void ReadStatusFile(void);
void StatusFileInit(void);
void _init_Status(int num);

typedef struct {
	TCHAR *key;		/* English key */
	TCHAR *sound;		/* What sound entry to play */
	TCHAR *nmea_gps;		/* NMEA Sentence - to GPS serial */
	TCHAR *nmea_vario;		/* NMEA Sentence - to Vario serial */
	bool doStatus;
	bool doSound;
	int delay_ms;		/* Delay for DoStatusMessage */
	int iFontHeightRatio;	// TODO - not yet used
	bool docenter;		// TODO - not yet used
	int *TabStops;		// TODO - not yet used
	int disabled;		/* Disabled - currently during run time */
} StatusMessageSTRUCT;

typedef struct {
	TCHAR *key;
	TCHAR *text;
} GetTextSTRUCT;



// Parse string (new lines etc) and malloc the string
TCHAR* StringMallocParse(TCHAR* old_string);

void LocalPath(TCHAR* buf, const TCHAR* file = TEXT(""), int loc = CSIDL_PERSONAL);
void LocalPathS(char* buf, const TCHAR* file = TEXT(""), int loc = CSIDL_PERSONAL);

void ExpandLocalPath(TCHAR* filein);
void ContractLocalPath(TCHAR* filein);

void ConvertTToC(CHAR* pszDest, const TCHAR* pszSrc);
void ConvertCToT(TCHAR* pszDest, const CHAR* pszSrc);

void propGetFontSettings(TCHAR *Name, LOGFONT* lplf);
void propGetFontSettingsFromString(TCHAR *Buffer, LOGFONT* lplf);
int propGetScaleList(double *List, size_t Size);

long GetUTCOffset(void);
int TextToLineOffsets(TCHAR* text, int* LineOffsets, int maxLines);
void RestoreRegistry(void);
void StoreRegistry(void);
void XCSoarGetOpts(LPTSTR CommandLine);

bool CheckRectOverlap(RECT rc1, RECT rc2);
int MeasureCPULoad();

TCHAR* GetWinPilotPolarInternalName(int i);

void SetSourceRectangle(RECT fromRect);
RECT WINAPI DrawWireRects(LPRECT lprcTo, UINT nMilliSecSpeed);

void OpenFLARMDetails();
void CloseFLARMDetails();
TCHAR* LookupFLARMDetails(long id);
int LookupFLARMDetails(TCHAR *cn);
bool AddFlarmLookupItem(int id, TCHAR *name, bool saveFile);
int LookupSecondaryFLARMId(int id);

double FindQNH(double alt_raw, double alt_known);
double AltitudeToQNHAltitude(double alt);
double StaticPressureToAltitude(double ps);
double AirDensity(double altitude);
double AirDensityRatio(double altitude);

double HexStrToDouble(TCHAR *Source, TCHAR **Stop);

long CheckFreeRam(void);

void MemCheckPoint();
void MemLeakCheck();
void MyCompactHeaps();
unsigned long FindFreeSpace(const TCHAR *path);
bool MatchesExtension(const TCHAR *filename, const TCHAR* extension);
BOOL PlayResource (const TCHAR* lpName);
void CreateDirectoryIfAbsent(TCHAR *filename);

bool InterfaceTimeoutZero(void);
void InterfaceTimeoutReset(void);
bool InterfaceTimeoutCheck(void);

#ifdef __cplusplus
extern "C"{
#endif

bool FileExistsW(TCHAR *FileName);
bool FileExistsA(char *FileName);

#ifdef _UNICODE
#define FileExists FileExistsW
#else
#define FileExists FileExistsA
#endif

#ifdef __cplusplus
}
#endif


//2^36 * 1.5,  (52-_shiftamt=36) uses limited precisicion to floor
//16.16 fixed point representation,

// =================================================================================
// Real2Int
// =================================================================================
inline int Real2Int(double val)
{
#if (WINDOWS_PC>0)
  val += 68719476736.0*1.5;
  return *((long*)&val) >> 16;
#else
  return (int)val;
#endif
}


inline int iround(double i) {
    return Real2Int(floor(i+0.5));
}

inline long lround(double i) {
    return (long)(floor(i+0.5));
}

inline unsigned int CombinedDivAndMod(unsigned int &lx) {
  unsigned int ox = lx & 0xff;
  // JMW no need to check max since overflow will result in
  // beyond max dimensions
  lx = lx>>8;
  return ox;
}

bool RotateScreen(void);

int GetTextWidth(HDC hDC, TCHAR *text);
void ExtTextOutClip(HDC hDC, int x, int y, TCHAR *text, int width);

#endif
