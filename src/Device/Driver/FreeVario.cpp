#include "Device/Config.hpp"
#include "Device/Driver/FreeVario.hpp"
#include "Device/Driver.hpp"
#include "Device/Port/Port.hpp"
#include "Device/Util/NMEAWriter.hpp"
#include "Message.hpp"
#include "NMEA/Checksum.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/Derived.hpp"
#include "NMEA/MoreData.hpp"
#include "NMEA/InputLine.hpp"
#include "Units/System.hpp"
#include "Operation/Operation.hpp"
// #include "Operation/PopupOperationEnvironment.hpp"
#include "LogFile.hpp"
#include "Interface.hpp"
#include "CalculationThread.hpp"
#include "Protection.hpp"
#include "Input/InputEvents.hpp"
#include <iostream>

using std::string_view_literals::operator""sv;

/*
 * Commands via NMEA from FreeVario Device to XCSoar:
 *
 * $PFV,M,S,<double value>  - set MC External to defined value
 * $PFV,M,U                 - set MC External up by 0.1
 * $PFV,M,D                 - set MC External down by 0.1
 *
 * $PFV,F,C                 - set Vario FlightMode to Circle
 * $PFV,F,S                 - set Vario FlightMode to SpeedToFly
 *
 * $PFV,Q,S,<double value>  - set QNH to defined value
 * $PFV,B,S,<double value>  - set Bugs to percent by capacity 0 - 100
 * $PFV,S,S,<integer value> - set Mute status and gives it back to sound board
 * $PFV,A,S,<integer value> - set attenuation and gives it back to sound board
 * $PFV,D,S,<integer value> - set STF-Mode and gives it back to sound board 
 *
 * Commands for NMEA compatibility with OpenVario especially variod
 * 
 * $POV,C,STF*4B  -- if this command is received it is resend to NMEA
 *                   Device 1 and Device 2 to sent variod to speed to fly
 *                   audio mode
 * $POV,C,VAR*4F  -- if this command is received it is resend to NMEA
 *                   Device 1 and Device 2 to set variod to vario audio mode
 *  
 */

class FreeVarioDevice : public AbstractDevice {
    Port &port;

public:
  explicit FreeVarioDevice(Port &_port):port(_port){}
  bool ParseNMEA(const char *line,NMEAInfo &info) override;
  static bool PFVParser(NMEAInputLine &line, NMEAInfo &info, Port &port);
  bool POVParserAndForward(NMEAInputLine &line);
  bool SendCmd(double value, const char *cmd, OperationEnvironment &env);
  bool PutMacCready(double mc, OperationEnvironment &env) override;
  bool PutBugs(double bugs, OperationEnvironment &env) override;
  bool PutQNH(const AtmosphericPressure &pres,
              OperationEnvironment &env) override;
  void OnCalculatedUpdate(const MoreData &basic,
              const DerivedInfo &calculated) override;
  void OnSensorUpdate(const MoreData &basic) override;
};

/**
 * Parse NMEA messsage to change between STF and vario mode 
   and check if it is a valid FreeVario message
 * Is true when a valid message or false if no valid message
 */
bool
FreeVarioDevice::POVParserAndForward(NMEAInputLine &line)
{
  NullOperationEnvironment env;
  // PopupOperationEnvironment env;
  bool messageValid = false;

  while (!line.IsEmpty() ) {

     char command = line.ReadOneChar();
      if (command >= 32 && command <= 126) {
        char buff[4] ;
        line.Read(buff,4);
        StaticString<4> bufferAsString(buff);

       if ( 'C' == command && strcmp("STF",bufferAsString) == 0){
         messageValid = true;
         InputEvents::eventSendNMEAPort1("POV,C,STF*4B");
         InputEvents::eventSendNMEAPort2("POV,C,STF*4B");
       }
       if ('C' == command && strcmp("VAR",bufferAsString) == 0){
         messageValid = true;
         InputEvents::eventSendNMEAPort1("POV,C,VAR*4F");
         InputEvents::eventSendNMEAPort2("POV,C,VAR*4F");
       }
     }
  }
  return messageValid;
}

/**
 * Parse NMEA message and check if it is a valid FreeVario message
 * Is true when a valid message or false if no valid message
 */
bool
FreeVarioDevice::PFVParser(NMEAInputLine &line, NMEAInfo &info, Port &port)
{
  NullOperationEnvironment env;
  bool validMessage = false;

  while (!line.IsEmpty()) {

    char type = line.ReadOneChar();
    char subCommand = line.ReadOneChar();

    if (subCommand == '\0')
      return validMessage;  // from now the string is invalid...

    switch (type) {
    case '\0':  // from now the string is invalid or finished
      return validMessage;
    case 'M': {
      if (subCommand == 'S') {
        double mcIn;
        if (line.ReadChecked(mcIn)) {
          info.settings.ProvideMacCready(mcIn, info.clock);
          validMessage = true;
        }
      }

      else if (subCommand == 'U') {
        double new_mc = std::min(info.settings.mac_cready + 0.1, 5.0);
        info.settings.ProvideMacCready(new_mc, info.clock);
        validMessage = true;
      }

      else if (subCommand == 'D') {
        double new_mc = std::max(info.settings.mac_cready - 0.1, 0.0);
        info.settings.ProvideMacCready(new_mc, info.clock);
        validMessage = true;
      }
      break;
    }
    case 'B': {
      if (subCommand == 'S') {
        double bugsIn;
        if (line.ReadChecked(bugsIn)) {
          info.settings.ProvideBugs((100 - bugsIn) / 100., info.clock);
          validMessage = true;
        }
      }
      break;
    }

    case 'Q': {
      if (subCommand == 'S') {
        double qnhIn;
        if (line.ReadChecked(qnhIn)) {
          AtmosphericPressure pres = info.static_pressure.HectoPascal(qnhIn);
          info.settings.ProvideQNH(pres, info.clock);
          validMessage = true;
        }
      }
      break;
    }

    case 'F': {
      if (subCommand == 'S') {
        info.switch_state.flight_mode = SwitchState::FlightMode::CRUISE;
        InputEvents::eventStatusMessage("Speed to Fly Mode");
        validMessage = true;
      } else if (subCommand == 'C') {
        info.switch_state.flight_mode = SwitchState::FlightMode::CIRCLING;
        InputEvents::eventStatusMessage("Vario Mode");
        validMessage = true;
      } else if (subCommand == 'A') {
        info.switch_state.flight_mode = SwitchState::FlightMode::UNKNOWN;
        InputEvents::eventStatusMessage("Automatic Mode");
        validMessage = true;
      }
      break;
    }

    case 'S': {
      if (subCommand == 'S') {
        char nmeaOutbuffer[80];
        int soundState;
        bool stateOK = line.ReadChecked(soundState);
        if (stateOK)
          snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,MUT,%d", soundState);
        PortWriteNMEA(port, nmeaOutbuffer, env);
        validMessage = true;
      }
      break;
    }

    case 'A': {
      if (subCommand == 'S') {
        char nmeaOutbuffer[80];
        int attenState;
        bool stateOK = line.ReadChecked(attenState);
        if (stateOK)
          snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,ATT,%d", attenState);
        PortWriteNMEA(port, nmeaOutbuffer, env);
        validMessage = true;
      }
      break;
    }

    case 'D': {
      if (subCommand == 'S') {
        char nmeaOutbuffer[80];
        int STFState;
        bool stateOK = line.ReadChecked(STFState);
        if (stateOK)
          snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,SMO,%d", STFState);
        PortWriteNMEA(port, nmeaOutbuffer, env);
        validMessage = true;
      }
      break;
    }
    default:
      // Just break on default
      break;
    }
  }

  return validMessage;
 }

/**
 * Parse incoming NMEA messages to check for PFV or POV messages
 */
bool
FreeVarioDevice::ParseNMEA(const char *_line, NMEAInfo &info)
{
  NullOperationEnvironment env;

  if (VerifyNMEAChecksum(_line)) {
    NMEAInputLine line(_line);
    const auto type = line.ReadView();
    if (type == "$PFV"sv) {
        return PFVParser(line, info, port);
    } else if (type == "$POV"sv) {
        return POVParserAndForward(line);  // no info and port necessary
    }
  }
  return false;
}

/*
 * Send total_energy_vario, external wind direction and external wind speed to 
 * FreeVario device on every sensor update. Is needed to have a good refresh 
 * rate on the external device showing the vario and external wind values.
 */
void
FreeVarioDevice::OnSensorUpdate(const MoreData &basic)
{

 NullOperationEnvironment env;
 char nmeaOutbuffer[80];

 if (basic.total_energy_vario_available.IsValid()) {
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,VAR,%f", basic.total_energy_vario);
    PortWriteNMEA(port, nmeaOutbuffer, env);
 }

  if (basic.settings.mac_cready_available.IsValid()){
    double externalMC = basic.settings.mac_cready;
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,MCE,%0.2f", (double)externalMC);
    PortWriteNMEA(port, nmeaOutbuffer, env);
  }

   // TODO(August2111): basic.netto_variable has no timestamp unfortunately?
   // if (basic.netto_vario_available.IsValid()) {
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,VAN,%f", basic.netto_vario);
    PortWriteNMEA(port, nmeaOutbuffer, env);
   // }
}

/*
 * Always send the calculated updated values to the FreeVario to have a good
 * refresh rate on the external device
 */
void
FreeVarioDevice::OnCalculatedUpdate(const MoreData &basic,
  const DerivedInfo &calculated)
{

 NullOperationEnvironment env;
 char nmeaOutbuffer[80];

 if (!basic.external_instantaneous_wind_available.IsValid() && calculated.wind.IsNonZero() && basic.track_available){
    const Angle relWindDirection = (calculated.wind.bearing - Angle::HalfCircle() - basic.track).AsBearing();
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,AWD,%f", relWindDirection.Degrees());
    PortWriteNMEA(port, nmeaOutbuffer, env);
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,AWS,%f", calculated.wind.norm);
    PortWriteNMEA(port, nmeaOutbuffer, env);
 }
 
 if (basic.external_wind_available.IsValid() && basic.attitude.heading_available){
    const Angle relWindDirection = (basic.external_wind.bearing - Angle::HalfCircle() - basic.attitude.heading).AsBearing();     
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,AWD,%f", relWindDirection.Degrees());
    PortWriteNMEA(port, nmeaOutbuffer, env);
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,AWS,%f", basic.external_wind.norm);
    PortWriteNMEA(port, nmeaOutbuffer, env);
 }

 if (basic.external_instantaneous_wind_available.IsValid() && basic.attitude.heading_available){
    const Angle relWindDirection = (basic.external_instantaneous_wind.bearing - Angle::HalfCircle() - basic.attitude.heading).AsBearing();     
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,CWD,%f", relWindDirection.Degrees());
    PortWriteNMEA(port, nmeaOutbuffer, env);
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,CWS,%f", basic.external_instantaneous_wind.norm);
    PortWriteNMEA(port, nmeaOutbuffer, env);
 }
 
 // vario average last 30 secs
 snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,VAA,%f",calculated.average);
 PortWriteNMEA(port, nmeaOutbuffer, env);

 if (calculated.altitude_agl_valid){
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,HAG,%f", calculated.altitude_agl);
    PortWriteNMEA(port, nmeaOutbuffer, env);
 }

 if (calculated.circling){
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,MOD,%s", "C");
    PortWriteNMEA(port, nmeaOutbuffer, env);
 } else {
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,MOD,%s", "S");
    PortWriteNMEA(port, nmeaOutbuffer, env);
 }
 
  double stfKmh = ((calculated.V_stf * 60 * 60) / 1000);
  snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,STF,%f", stfKmh);
  PortWriteNMEA(port, nmeaOutbuffer, env);
  
 if (basic.baro_altitude_available.IsValid()){
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,HIG,%f", basic.baro_altitude);
    PortWriteNMEA(port, nmeaOutbuffer, env);
 } else if (basic.gps_altitude_available.IsValid()){
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,HIG,%f", basic.gps_altitude);
    PortWriteNMEA(port, nmeaOutbuffer, env);
 } else {
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,HIG,%f", 0.0);
    PortWriteNMEA(port, nmeaOutbuffer, env);
 }

  bool tempAvil = basic.temperature_available;
  if (tempAvil){
    double temp = basic.temperature.ToCelsius();
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,TEM,%f", temp);
    PortWriteNMEA(port, nmeaOutbuffer, env);
  }

  double trueAirspeedKmh = ((basic.true_airspeed * 60 * 60) / 1000);
  snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,TAS,%f", trueAirspeedKmh);
  PortWriteNMEA(port, nmeaOutbuffer, env);

  if (basic.ground_speed_available.IsValid()){
    double groundSpeed = ((basic.ground_speed * 60 * 60) / 1000);
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,GRS,%f", groundSpeed);
    PortWriteNMEA(port, nmeaOutbuffer, env);
  } else {
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,GRS,%d", -1);
    PortWriteNMEA(port, nmeaOutbuffer, env);
  }
}
/*
 *  Send the internal xcsoar mc value to FreeVario device to
 *  be informed about MC changes doen in XCSoar
 */
bool
FreeVarioDevice::SendCmd(double value, const char *cmd,
  OperationEnvironment &env)
{
 if (!EnableNMEA(env)) {
    return false;
 }
 char nmeaOutbuffer[80];
 snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer), cmd, value);
 PortWriteNMEA(port, nmeaOutbuffer, env);
 return true;
}

bool
FreeVarioDevice::PutMacCready(double mc, OperationEnvironment &env)
{
 return SendCmd(mc, "PFV,MCI,%0.2f", env);
}

bool
FreeVarioDevice::PutBugs(double bugs,OperationEnvironment &env){
 if (!EnableNMEA(env)){return false;}
    char nmeaOutbuffer[80];
    double bugsAsPercentage = (1 - bugs) * 100;
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,BUG,%f",bugsAsPercentage);
    PortWriteNMEA(port, nmeaOutbuffer, env);
    return true;
}

bool
FreeVarioDevice::PutQNH(const AtmosphericPressure &pres,OperationEnvironment &env) {
 if (!EnableNMEA(env)){return false;}
    char nmeaOutbuffer[80];
    snprintf(nmeaOutbuffer,sizeof(nmeaOutbuffer),"PFV,QNH,%f",pres.GetHectoPascal());
    PortWriteNMEA(port, nmeaOutbuffer, env);
    return true;
}


static Device *
FreeVarioCreateOnPort([[maybe_unused]] const DeviceConfig &config,
 Port &com_port)
{
 return new FreeVarioDevice(com_port);
}

const struct DeviceRegister free_vario_driver = {
 "FreeVario",
 "FreeVario",
 DeviceRegister::SEND_SETTINGS|
 DeviceRegister::RECEIVE_SETTINGS,
 FreeVarioCreateOnPort,
};
