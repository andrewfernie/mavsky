//#include <OctoWS2811.h>
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  A copy of the GNU General Public License is available at <http://www.gnu.org/licenses/>.
//    
//  Creator:  Scott Simpson
//
//
//  For Teensy 3.1 support
//    Connect:
//      SPort S     -> TX1
//      SPort +     -> Vin
//      SPort -     -> GND
//  
//      Mavlink TX  -> RX2
//      Mavlink GND -> GND
//
//
//  Required Connections
//  --------------------
//    pin 2:  LED Strip #1    OctoWS2811 drives 8 LED Strips.
//    pin 14: LED strip #2    All 8 are the same length.
//    pin 7:  LED strip #3
//    pin 8:  LED strip #4    A 100 ohm resistor should used
//    pin 6:  LED strip #5    between each Teensy pin and the
//    pin 20: LED strip #6    wire to the LED strip, to minimize
//    pin 21: LED strip #7    high frequency ringining & noise.
//    pin 5:  LED strip #8
//    pin 15 & 16 - Connect together, but do not use
//    pin 4 - Do not use
//    pin 3 - Do not use as PWM.  Normal use is ok.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <GCS_MAVLink.h>
#include <EEPROM.h>
#include "MavSky.h"
#include "FrSkySPort.h"
#include "MavConsole.h"
#include "Diags.h"
#include "Logger.h"
#include "DataBroker.h"

#include "Led.h"

//************************************* Please select your options here before compiling **************************
// Choose one (only) of these target boards
#define Target_Board   0      // Teensy 3.x              Un-comment this line if you are using a Teensy 3.x
//#define Target_Board   1      // Blue Pill STM32F103C    OR un-comment this line if you are using a Blue Pill STM32F103C
//#define Target_Board   2      // Maple_Mini STM32F103C   OR un-comment this line if you are using a Maple_Mini STM32F103C

// Choose one (only) of these three modes
#define Ground_Mode          // Converter between Taranis and LRS tranceiver (like Orange)
//#define Air_Mode             // Converter between FrSky receiver (like XRS) and Flight Controller (like Pixhawk)
//#define Relay_Mode           // Converter between LRS tranceiver (like Orange) and FrSky receiver (like XRS) in relay box on the ground

//#define Battery_mAh_Source  1  // Get battery mAh from the FC - note both RX and TX lines must be connected      
//#define Battery_mAh_Source  2  // Define bat1_capacity and bat2_capacity below and use those 
#define Battery_mAh_Source  3  // Define battery mAh in the LUA script on the Taranis/Horus - Recommended
#define SPort_Serial   1    // The default is Serial 1, but 3 is possible if we don't want aux port

//#define Aux_Port_Enabled            // For BlueTooth or other auxilliary serial passthrough
//#define Request_Missions_From_FC    // Un-comment if you need mission waypoint from FC - NOT NECESSARY RIGHT NOW
#define Request_Mission_Count_From_FC // Needed for yaapu's mission/waypoint script
//#define QLRS                        // Un-comment this line only if you are using the QLRS telemetry system

const uint16_t bat1_capacity = 5200;
const uint16_t bat2_capacity = 0;



//*** LEDS ********************************************************************************************************
//uint16_t MavStatusLed = 13; 
uint8_t MavLedState = LOW;
uint16_t BufStatusLed = 12;
uint8_t BufLedState = LOW;

#if (Target_Board == 0) // Teensy3x
#define MavStatusLed  13
#elif (Target_Board == 1) // Blue Pill
#define MavStatusLed  PC13
#elif (Target_Board == 2) //  Maple Mini
#define MavStatusLed  33  // PB1
#endif
//********************************************************* 
#if (Target_Board == 1) // Blue Pill
#if defined Aux_Port_Enabled       
#error Blue Pill board does not have enough UARTS for Auxilliary port. Un-comment #define Aux_Port_Enabled.
#endif
#endif

#if (Target_Board == 1) // Blue Pill
#if (SPort_Serial  == 3)    
#error Blue Pill board does not have Serial3. This configuration is not possible.
#endif
#endif

#define Debug               Serial         // USB 
#define frBaud              57600          // Use 57600
#define mavSerial           Serial2        
#define mavBaud             57600   

#if (SPort_Serial == 1) 
#define frSerial              Serial1        // S.Port 
#elif (SPort_Serial == 3)
#define frSerial            Serial3        // S.Port 
#else
#error SPort_Serial can only be 1 or 3. Please correct.
#endif  

#if defined Aux_Port_Enabled
#if (SPort_Serial == 3) 
#error Aux port and SPort both configured for Serial3. Please correct.
#else 
#define auxSerial             Serial3        // Mavlink telemetry to and from auxilliary adapter     
#define auxBaud               57600          // Use 57600
#define auxDuplex                            // Pass aux <-> FC traffic up and down, else only down from FC
#endif
#endif

//#define Frs_Dummy_rssi       // For LRS testing only - force valid rssi. NOTE: If no rssi FlightDeck or other script won't connect!
//#define Data_Streams_Enabled // Rather set SRn in Mission Planner

#define Max_Waypoints  256     // Note. This is a RAM trade-off. If exceeded then Debug message and shut down

// Debugging options below ***************************************************************************************
#define Mav_Debug_All
//#define Frs_Debug_All
//#define Frs_Debug_Payload
//#define Mav_Debug_RingBuff
//#define Debug_Air_Mode
//#define Mav_List_Params
//#define Aux_Debug_Params
//#define Aux_Port_Debug
//#define Mav_Debug_Params
//#define Frs_Debug_Params
//#define Mav_Debug_Servo
//#define Frs_Debug_Servo
//#define Mav_Debug_Rssi
//#define Mav_Debug_RC
//#define Frs_Debug_RC
//#define Mav_Debug_Heartbeat
//#define Mav_Debug_SysStatus
//#define Frs_Debug_LatLon
//#define Frs_Debug_APStatus
//#define Debug_Batteries
//#define Frs_Debug_Home
//#define Mav_Debug_GPS_Raw     // #24
//#define Mav_Debug_GPS_Int     // #33
//#define Frs_Debug_YelYaw
//#define Frs_Debug_GPS_Status
//#define Mav_Debug_Raw_IMU
//#define Mav_Debug_Hud
//#define Frs_Debug_Hud
//#define Mav_Debug_Scaled_Pressure
//#define Mav_Debug_Attitude
//#define Frs_Debug_Attitude
//#define Mav_Debug_Text
//#define Frs_Debug_Text    
//#define Mav_Debug_Mission 
//#define Frs_Debug_Mission              
//*****************************************************************************************************************

#define LEDPIN          13
#define PROBEPIN        12
                          
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Diags diags;
Logger *logger;
MavConsole *console;
MavLinkData *mav;
FrSkySPort *frsky;
DataBroker *data_broker;
  
LedController* led_strip_ptr;

void setup()  {
  console = new MavConsole(Serial);
  logger = new Logger();
  mav = new MavLinkData();
  frsky = new FrSkySPort();
  data_broker = new DataBroker();

  delay(5000);

  pinMode(LEDPIN, OUTPUT);
  pinMode(PROBEPIN, OUTPUT);
  console->console_print("%s\r\nStarting\r\n]", PRODUCT_STRING);

  led_strip_ptr = new LedController();  
}

void check_for_faults() {
  int mav_online;
  mav_online = mav->mavlink_heartbeat_data_valid();
  diags.set_fault_to(FAULT_MAV_OFFLINE, !mav_online);
  if(mav_online) {                                                                  // don't set other mav faults if mav is offline
    diags.set_fault_to(FAULT_MAV_SYS_STATUS, !mav->mavlink_sys_status_data_valid());
    diags.set_fault_to(FAULT_MAV_GPS, !mav->mavlink_gps_data_valid());
    diags.set_fault_to(FAULT_MAV_VFR_HUD, !mav->mavlink_vfr_data_valid());
    diags.set_fault_to(FAULT_MAV_RAW_IMU, !mav->mavlink_imu_data_valid());
    diags.set_fault_to(FAULT_MAV_ATTITUDE, !mav->mavlink_attitude_data_valid());
  } else {
    diags.clear_fault(FAULT_MAV_SYS_STATUS);
    diags.clear_fault(FAULT_MAV_GPS);
    diags.clear_fault(FAULT_MAV_VFR_HUD);
    diags.clear_fault(FAULT_MAV_RAW_IMU);
    diags.clear_fault(FAULT_MAV_ATTITUDE);
  }
  diags.set_fault_to(FAULT_SPORT_OFFLINE, !frsky->frsky_online());
}

uint32_t next_1000_loop = 0L;
uint32_t next_200_loop = 0L;
uint32_t next_100_loop = 0L;
uint32_t next_10_loop = 0L;

void loop()  {
  uint32_t current_milli = millis();

  mav->process_mavlink_packets();

  frsky->frsky_process();         

  console->check_for_console_command();  

  if(current_milli >= next_1000_loop) {
    next_1000_loop = current_milli + 1000;
    mav->process_1000_millisecond();
  }
  
  if(current_milli >= next_200_loop) {
    next_200_loop = current_milli + 200;
    diags.update_led();
  }
  
  if(current_milli >= next_100_loop) {
    next_100_loop = current_milli + 100;
    if(current_milli > 10000) {
      check_for_faults();
    }
    mav->process_100_millisecond();   
  }

  if(current_milli >= next_10_loop) {
    next_10_loop = current_milli + 10;
    digitalWrite(PROBEPIN, HIGH);  
    led_strip_ptr->process_10_millisecond();
    digitalWrite(PROBEPIN, LOW);  
    led_strip_ptr->update_leds();
  }
}

