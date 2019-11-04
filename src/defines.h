// 1-channel LoRa Gateway for ESP8266 / ESP32
// Copyright (c) 2016, 2017, 2018, 2019 Maarten Westenberg version for ESP8266
// Version 6.1.0 E EU868
// Date: 2019-10-20
//
// Based on work done by Thomas Telkamp for Raspberry PI 1ch gateway and many others.
// Contibutions of Dorijan Morelj and Andreas Spies for OLED support.
//
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the MIT License
// which accompanies this distribution, and is available at
// https://opensource.org/licenses/mit-license.php
//
// NO WARRANTY OF ANY KIND IS PROVIDED
//
// Author: Maarten Westenberg (mw12554@hotmail.com)
//
// ********************************************************************************
// ********************************************************************************
// Ported to use with Semtech SX1262 chips
// Author: Bernd Giesecke (https://github.com/beegee-tokyo)
//
// Port is work in progress
// CAD and HOP is not supported for SX1262 chips
// Downlink is not working
// Compiles under PlatformIO only
// ********************************************************************************
// ********************************************************************************
//
// This file contains a number of compile-time settings that can be set on (=1) or off (=0)
// The disadvantage of compile time is minor compared to the memory gain of not having
// too much code compiled and loaded on your ESP8266.
//
// ----------------------------------------------------------------------------------------

// NOTE: Compile with ESP32 setting and board "ESP32 Dev Module" or "Heltec WiFi Lora 32"
#define VERSION "V.6.1.0.E.US915; 191020a"

// This value of DEBUG determines whether some parts of code get compiled.
// Also this is the initial value of debug parameter.
// The value can be changed using the admin webserver
// For operational use, set initial DEBUG vaulue 0
#define DEBUG 2

// Debug message will be put on Serial is this one is set.
// If set to 0, not USB Serial prints are done
// Set to 1 it will prinr all user level messages (with correct debug set)
// If set to 2 it will also print interrupt messages (not recommended)
#define DUSB 1

// Define whether we should do a formatting of SPIFFS when starting the gateway
// This is usually a good idea if the webserver is interrupted halfway a writing
// operation. Also to be used when software is upgraded
// Normally, value 0 is a good default.
#define _SPIFF_FORMAT 0

// Define the frequency band the gateway will listen on. Valid options are
// EU863_870 (Europe), US902_928 (North America) & AU925_928 (Australia), CN470_510 (China).
// See https://www.thethingsnetwork.org/docs/lorawan/frequency-plans.html
// #define EU863_870 1
#define US902_928 1
// #define AU925_928 1
// #define CN470_510 1

// Define whether to use the old Semtech gateway API, which is still supported by TTN,
// but is more lightweight than the new TTN tcp based protocol.
// NOTE: Only one of the two should be defined!
//
#define _UDPROUTER 1
// #define _TTNROUTER 1

// The spreading factor is the most important parameter to set for a single channel
// gateway. It specifies the speed/datarate in which the gateway and node communicate.
// As the name says, in principle the single channel gateway listens to one channel/frequency
// and to one spreading factor only.
// This parameters contains the default value of SF, the actual version can be set with
// the webserver and it will be stored in SPIFF
// NOTE: The frequency is set in the loraModem.h file and is default 868.100000 MHz.
#define _SPREADING SF7

// Channel Activity Detection
// This function will scan for valid LoRa headers and determine the Spreading
// factor accordingly. If set to 1 we will use this function which means the
// 1-channel gateway will become even more versatile. If set to 0 we will use the
// continuous listen mode.
// Using this function means that we HAVE to use more dio pins on the RFM95/sx1276
// device and also connect enable dio1 to detect this state.
#define _CAD 0

// Definitions for the admin webserver.
// A_SERVER determines whether or not the admin webpage is included in the sketch.
// Normally, leave it in!
#define A_SERVER 1		 // Define local WebServer only if this define is set
#define A_REFRESH 1		 // Allow the webserver refresh or not?
#define A_SERVERPORT 80  // Local webserver port (normally 80)
#define A_MAXBUFSIZE 192 // Must be larger than 128, but small enough to work

// Definitions for over the air updates. At the moment we support OTA with IDE
// Make sure that tou have installed Python version 2.7 and have Bonjour in your network.
// Bonjour is included in iTunes (which is free) and OTA is recommended to install
// the firmware on your router witout having to be really close to the gateway and
// connect with USB.
#define A_OTA 1

// We support a few pin-out configurations out-of-the-box: HALLARD, COMPRESULT and TTGO ESP32.
// If you use one of these two, just set the parameter to the right value.
// If your pin definitions are different, update the loraModem.h file to reflect these settings.
//	1: HALLARD
//	2: COMRESULT pin out
//	3: ESP32 Wemos pin out
//	4: ESP32 TTGO pinning (should work for 433 and OLED too).
//	5: ESP32 TTGO EU433 MHz with OLED
//	6: Other, define your own in loraModem.h
#define _PIN_OUT 6

// Gather statistics on sensor and Wifi status
// 0= No statistics
// 1= Keep track of messages statistics, number determined by MAX_STAT
// 2= Option 1 + Keep track of messages received PER each SF (default)
// 3= See Option 2, but with extra channel info (Do not use when no Hopping is selected)
#define STATISTICS 1

// Maximum number of statistics records gathered. 20 is a good maximum (memory intensive)
// For ESP32 maybe 30 could be used as well
#define MAX_STAT 10

// Single channel gateways if they behave strict should only use one frequency
// channel and one spreading factor. However, the TTN backend replies on RX2
// timeslot for spreading factors SF9-SF12.
// Also, the server will respond with SF12 in the RX2 timeslot.
// If the 1ch gateway is working in and for nodes that ONLY transmit and receive on the set
// and agreed frequency and spreading factor. make sure to set STRICT to 1.
// In this case, the frequency and spreading factor for downlink messages is adapted by this
// gateway
// NOTE: If your node has only one frequency enabled and one SF, you must set this to 1
//		in order to receive downlink messages
// NOTE: In all other cases, value 0 works for most gateways with CAD enabled
#define _STRICT_1CH 1

// Allows configuration through WifiManager AP setup. Must be 0 or 1
#define WIFIMANAGER 0

// Define the name of the accesspoint if the gateway is in accesspoint mode (is
// getting WiFi SSID and password using WiFiManager)
#define AP_NAME "LoRaGateway"
#define AP_PASSWD "LoRaGateway"

// This section defines whether we use the gateway as a repeater
// For his, we use another output channle as the channel (default==0) we are
// receiving the messages on.
#define _REPEATER 0

// Will we use Mutex or not?
// +SPI is input for SPI, SPO is output for SPI
#define MUTEX 0

// Define if OLED Display is connected to I2C bus. Note that defining an OLED display does not
// impact performance very much, certainly if no OLED is connected. Wrong OLED will not show
// sensible results on display
// OLED==0; No OLED display connected
// OLED==1; 0.9 Oled Screen based on SSD1306
// OLED==2;	1"3 Oled screens for Wemos, 128x64 SH1106
#define OLED 2

// Define whether we want to manage the gateway over UDP (next to management
// thru webinterface).
// This will allow us to send messages over the UDP connection to manage the gateway
// and its parameters. Sometimes the gateway is not accesible from remote,
// in this case we would allow it to use the SERVER UDP connection to receive
// messages as well.
// NOTE: Be aware that these messages are NOT LoRa and NOT LoRa Gateway spec compliant.
//	However that should not interfere with regular gateway operation but instead offer
//	functions to set/reset certain parameters from remote.
#define GATEWAYMGT 0

// Do extensive loggin
// Use the ESP8266 SPIFS filesystem to do extensive logging.
// We must take care that the filesystem never(!) is full, and for that purpose we
// rather have new records/line of statistics than very old.
// Of course we must store enough records to make the filesystem work
#define STAT_LOG 1

// Name of he configfile in SPIFFs	filesystem
// In this file we store the configuration and other relevant info that should
// survive a reboot of the gateway
#define CONFIGFILE "/gwayConfig.txt"

// Set the Server Settings (IMPORTANT)
#define _LOCUDPPORT 1700 // UDP port of gateway! Often 1700 or 1701 is used for upstream comms

// Timing
#define _MSG_INTERVAL 15   // Reset timer in seconds
#define _PULL_INTERVAL 55  // PULL_DATA messages to server to get downstream in milliseconds
#define _STAT_INTERVAL 120 // Send a 'stat' message to server
#define _NTP_INTERVAL 3600 // How often do we want time NTP synchronization
#define _WWW_INTERVAL 60   // Number of seconds before we refresh the WWW page

// MQTT definitions, these settings should be standard for TTN
// and need not changing
#define _TTNPORT 1700							 // Standard port for TTN
#define _TTNSERVER "router.us.thethings.network" //"us-west.thethings.network" "router.eu.thethings.network" "router.us.thethings.network" "asia-se.thethings.network"

// If you have a second back-end server defined such as Semtech or loriot.io
// your can define _THINGPORT and _THINGSERVER with your own value.
// If not, make sure that you do not define these, which will save CPU time
// Port is UDP port in this program
//
// Default for testing: Switched off
// #define _THINGSERVER "westenberg.org"		// Server URL of the LoRa-udp.js handler
// #define _THINGPORT 1700						// Port 1700 is old compatibility

// This defines whether or not we would use the gateway as
// as sort of backend system which decodes
// 1: _LOCALSERVER is used
// 0: Do not use _LOCALSERVER
// If you want to decode messages you need to fill out the decodes structure 
// in loraModem.cpp and adapt the definition of decodes in loraModem.h
#define _LOCALSERVER 1 // See server definitions for decodes

// Gateway Ident definitions
#define _DESCRIPTION "ESP32 GW" // Name of the gateway
#define _EMAIL "owner@example.com"	 // Owner
#define _PLATFORM "ESP32"
#define _LAT 14.472595
#define _LON 121.001144
#define _ALT 25 // Altitude

// ntp
// Please add daylight saving time to NTP_TIMEZONES when desired
#define NTP_TIMESERVER "asia.pool.ntp.org" // Country and region specific
#define NTP_TIMEZONES 8					   // How far is our Timezone from UTC (excl daylight saving/summer time)
#define SECS_IN_HOUR 3600
#define NTP_INTR 0 // Do NTP processing with interrupts or in loop();

// lora sensor code definitions
// Defines whether the gateway will also report sensor/status value on MQTT
// after all, a gateway can be a node to the system as well. Some sensors like GPS can be
// sent to the backend as a parameter, some (like humidity for example) can only be sent
// as a regular sensor value.
// Set its LoRa address and key below in this file, See spec. para 4.3.2
#define GATEWAYNODE 0
#define _CHECK_MIC 0

#if GATEWAYNODE == 1
#define _DEVADDR               \
	{                          \
		0x22, 0x22, 0x22, 0x22 \
	}
#define _APPSKEY                                                                                       \
	{                                                                                                  \
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 \
	}
#define _NWKSKEY                                                                                       \
	{                                                                                                  \
		0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 \
	}
#define _SENSOR_INTERVAL 300
// For ESP32 based T_BEAM/TTGO boards these two are normally included
// If included make value 1, else if not, make them 0
#define _GPS 1
#define _BATTERY 1
#endif

// Define the correct radio type that you are using
// #define CFG_sx1276_radio
//#define CFG_sx1272_radio
#define CFG_sx1262_radio

// Serial Port speed
#define _BAUDRATE 115200 // Works for debug messages to serial momitor

// We can put the gateway in such a mode that it will (only) recognize
// nodes that are put in a list of trusted nodes
// Values:
// 0: Do not use names for trusted Nodes
// 1: Use the nodes as a translation table for hex codes to names (in TLN)
// 2: Same as 1, but is nodes NOT in the nodes list below they are NOT
//		forwarded or counted! (not yet fully implemented)
#define _TRUSTED_NODES 1
/// \todo _TRUSTED_DECODE collides with _LOCALSERVER
#define _TRUSTED_DECODE 1

// RX TX and status buffer sizes
#define TX_BUFF_SIZE 1024 // Upstream buffer to send to MQTT
#define RX_BUFF_SIZE 1024 // Downstream received from MQTT
#define STATUS_SIZE 512   // Should(!) be enough based on the static text .. was 1024

// Includes go here
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
#define ESP32_ARCH 1
#endif

#include <Esp.h> // ESP specific IDE functions
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstdlib>
#include <sys/time.h>
#include <cstring>
#include <string> // C++ specific string functions

#include <SPI.h>	   // For the RFM95 or SX1262 bus
#include <TimeLib.h>   // http://playground.arduino.cc/code/time
#include <DNSServer.h> // Local DNSserver
#include <ArduinoJson.h>
#include <FS.h> // ESP Specific
#include <WiFiUdp.h>
#include <pins_arduino.h>
#include <gBase64.h> // https://github.com/adamvr/arduino-base64 (changed the name)

// Local include files
#include "loraModem.h" // For RFM95 modules
#include "loraFiles.h"
#include "sensor.h"
#include "oLED.h"

// For SX1262
#include <SX126x-Arduino.h>

extern "C"
{
#include "lwip/err.h"
#include "lwip/dns.h"
}

#if WIFIMANAGER == 1
#include <WiFiManager.h> // Library for ESP WiFi config through an AP
#endif

#if (GATEWAYNODE == 1) || (_LOCALSERVER == 1)
#include "AES-128_V10.h"
#endif

// ----------- Specific ESP32 stuff --------------
#if ESP32_ARCH == 1 // IF ESP32

#include "WiFi.h"
#include <ESPmDNS.h>
#include <SPIFFS.h>
#if A_SERVER == 1
#include <ESP32WebServer.h> // Dedicated Webserver for ESP32
#include <Streaming.h>		// http://arduiniana.org/libraries/streaming/
#endif
#if A_OTA == 1
#include <ESP32httpUpdate.h> // Not yet available
#include <ArduinoOTA.h>
#endif //OTA

// ----------- Specific ESP8266 stuff --------------
#else

#include <ESP8266WiFi.h> // Which is specific for ESP8266
#include <ESP8266mDNS.h>
extern "C"
{
#include "user_interface.h"
#include "c_types.h"
}
#if A_SERVER == 1
#include <ESP8266WebServer.h>
#include <Streaming.h> // http://arduiniana.org/libraries/streaming/
#endif				   //A_SERVER
#if A_OTA == 1
#include <ESP8266httpUpdate.h>
#include <ArduinoOTA.h>
#endif //OTA

#endif //ESP_ARCH

// Global variables go here
#ifndef DEFINES_H
#define DEFINES_H
// Wifi definitions
// WPA is an array with SSID and password records. Set WPA size to number of entries in array
// When using the WiFiManager, we will overwrite the first entry with the
// accesspoint we last connected to with WifiManager
// NOTE: Structure needs at least one (empty) entry.
//		So WPASIZE must be >= 1
struct wpas
{
	char login[32]; // Maximum Buffer Size (and allocated memory)
	char passw[64];
};

// Please fill in at least ONE SSID and password from your own WiFI network
// in struct wpas wpa[] in the file wLan.cpp. 
// This is needed to get the gateway working
// Note: DO NOT use the first and the last line of the stucture, these should be empty strings and
//the first line in te struct is reserved for WifiManager.
extern wpas wpa[];
#endif // DEFINES_H

// For asserting and testing the following defines are used.
//
#if !defined(CFG_noassert)
#define ASSERT(cond) \
	if (!(cond))     \
	gway_failed(__FILE__, __LINE__)
#else
#define ASSERT(cond) /**/
#endif

// Function & global variable declarations
// ----------------------------------------------------------------------------
// FORWARD DECLARATIONS
// These forward declarations are done since other .cpp fils are linked by the
// compiler/linker AFTER the main.cpp file.
// And espcecially when calling functions with ICACHE_RAM_ATTR the complier
// does not want this.
// Solution can also be to specify less STRICT compile options in Makefile
// ----------------------------------------------------------------------------

void ICACHE_RAM_ATTR Interrupt_0();
void ICACHE_RAM_ATTR Interrupt_1();

void ftoa(float f, char *val, int p); // main.cpp

int sendPacket(uint8_t *buf, uint8_t length); // txRx.cpp
int receivePacket();						  // txRx.cpp

void setupWWW();											   // wwwServer.cpp
void printIP(IPAddress ipa, const char sep, String &response); // wwwServer.cpp

void SerialTime();											 // utils.cpp
void SerialStat(uint8_t intr);								 // utils.cpp
void printHexDigit(uint8_t digit);							 // utils.cpp
int inDecodes(char *id);									 // utils.cpp
void stringTime(time_t t, String &response);				 // utils.cpp
int SerialName(char *a, String &response);					 // utils.cpp
void printHEX(char *hexa, const char sep, String &response); // utils.cpp
void printTime();											 // utils.cpp

void init_oLED(void);		// oLED.cpp
void acti_oLED();			// oLED.cpp
void addr_oLED();			// oLED.cpp
void dispWriteHeader(void); // oLED.cpp

void setupOta(char *hostname); // otaServer.cpp

void addLog(const unsigned char *line, int cnt);		  // loraFiles.cpp
int writeGwayCfg(const char *fn);						  // loraFiles.cpp
int initConfig(struct espGwayConfig *c);				  // loraFiles.cpp
int readConfig(const char *fn, struct espGwayConfig *c);  // loraFiles.cpp
int writeConfig(const char *fn, struct espGwayConfig *c); // loraFiles.cpp

void initLoraModem();							 // loraModem.cpp
void rxLoraModem();								 // loraModem.cpp
void writeRegister(uint8_t addr, uint8_t value); // loraModem.cpp
uint8_t readRegister(uint8_t addr);				 // loraModem.cpp
void cadScanner();								 // loraModem.cpp
void hop();										 // loraModem.cpp
void opmode(uint8_t mode);						 // loraModem.cpp
void setRate(uint8_t sf, uint8_t crc);			 // loraModem.cpp
uint8_t receivePkt(uint8_t *payload);			 // loraModem.cpp
void setFreq(uint32_t freq);					 // loraModem.cpp
void startReceiver();							 // loraModem.cpp
void txLoraModem(uint8_t *payLoad, uint8_t payLength, uint32_t tmst, uint8_t sfTx,
				 uint8_t powe, uint32_t freq, uint8_t crc, uint8_t iiq); // loraModem.cpp

void stateMachine(); // stateMachine.cpp

bool connectUdp();												   // udpSemtech.cpp
int readUdp(int packetSize);									   // udpSemtech.cpp
int sendUdp(IPAddress server, int port, uint8_t *msg, int length); // udpSemtech.cpp
void sendstat();												   // udpSemtech.cpp
void pullData();												   // udpSemtech.cpp

void updateOtaa(); // otaServer.cpp

int WlanReadWpa();			 // wLan.cpp
int WlanConnect(int maxTry); // wLan.cpp

uint8_t encodePacket(uint8_t *Data, uint8_t DataLength,
					 uint16_t FrameCount, uint8_t *DevAddr,
					 uint8_t *AppSKey, uint8_t Direction); // sensor.cpp

#if MUTEX == 1
// Forward declarations
void ICACHE_FLASH_ATTR CreateMutux(int *mutex);
bool ICACHE_FLASH_ATTR GetMutex(int *mutex);
void ICACHE_FLASH_ATTR ReleaseMutex(int *mutex);
#endif

extern bool otaActive;
extern sf_t sf;
extern sf_t sfi;
extern uint8_t debug;
extern uint8_t pdebug;
extern uint8_t ifreq;
extern uint8_t MAC_array[6];
extern uint32_t eventTime;
extern uint32_t doneTime;
extern uint32_t sendTime;
extern time_t startTime;
extern IPAddress ttnServer;
extern bool sx1272;
extern WiFiUDP Udp;
extern float lat;
extern float lon;
extern int alt;
extern char platform[24];
extern char email[40];
extern char description[64];
extern espGwayConfig gwayConfig;

#if A_SERVER == 1
#if ESP32_ARCH == 1
extern ESP32WebServer server;
#else
extern ESP8266WebServer server;
#endif
#endif

#if _GPS == 1
#include <TinyGPS++.h>
extern TinyGPSPlus gps;
#endif