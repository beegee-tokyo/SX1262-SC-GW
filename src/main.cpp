// 1-channel LoRa Gateway for ESP8266
// Copyright (c) 2016, 2017, 2018, 2019 Maarten Westenberg version for ESP8266
// Version 6.1.0
// Date: 2019-10-20
// Author: Maarten Westenberg (mw12554@hotmail.com)
//
// Based on work done by Thomas Telkamp for Raspberry PI 1-ch gateway and many others.
//
// All rights reserved. This program and the accompanying materials
// are made available under the terms of the MIT License
// which accompanies this distribution, and is available at
// https://opensource.org/licenses/mit-license.php
//
// NO WARRANTY OF ANY KIND IS PROVIDED
//
// The protocols and specifications used for this 1ch gateway:
// 1. LoRA Specification version V1.0 and V1.1 for Gateway-Node communication
//
// 2. Semtech Basic communication protocol between Lora gateway and server version 3.0.0
//	https://github.com/Lora-net/packet_forwarder/blob/master/PROTOCOL.TXT
//
// Notes:
// - Once call gethostbyname() to get IP for services, after that only use IP
//	 addresses (too many gethost name makes the ESP unstable)
// - Only call yield() in main stream (not for background NTP sync).
//
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
// ----------------------------------------------------------------------------------------
#include <Arduino.h>
#include "defines.h"

// ----------- Declaration of vars --------------
uint8_t debug = 1;	 // Debug level! 0 is no msgs, 1 normal, 2 extensive
uint8_t pdebug = 0xFF; // Allow all atterns (departments)

#if GATEWAYNODE == 1
#if _GPS == 1
#include <TinyGPS++.h>
TinyGPSPlus gps;
HardwareSerial Serial1(1);
#endif
#endif

// You can switch webserver off if not necessary but probably better to leave it in.
#if A_SERVER == 1
#if ESP32_ARCH == 1
ESP32WebServer server(A_SERVERPORT);
#else
ESP8266WebServer server(A_SERVERPORT);
#endif
#endif

using namespace std;

byte currentMode = 0x81;

bool sx1272 = true; // Actually we use sx1276/RFM95

uint8_t ifreq = 0; // Channel Index

uint8_t MAC_array[6];

// ----------------------------------------------------------------------------
//
// Configure these values only if necessary!
//
// ----------------------------------------------------------------------------

// Set spreading factor (SF7 - SF12)
sf_t sf = _SPREADING;
sf_t sfi = _SPREADING; // Initial value of SF

// Set location, description and other configuration parameters
// Defined in ESP-sc_gway.h
//
float lat = _LAT; // Configuration specific info...
float lon = _LON;
int alt = _ALT;
char platform[24] = _PLATFORM;		 // platform definition
char email[40] = _EMAIL;			 // used for contact email
char description[64] = _DESCRIPTION; // used for free form description

// define servers

IPAddress ntpServer; // IP address of NTP_TIMESERVER
IPAddress ttnServer; // IP Address of thethingsnetwork server
IPAddress thingServer;

WiFiUDP Udp;

time_t startTime = 0;   // The time in seconds since 1970 that the server started
						// be aware that UTP time has to succeed for meaningful values.
						// We use this variable since millis() is reset every 50 days...
uint32_t eventTime = 0; // Timing of _event to change value (or not).
uint32_t sendTime = 0;  // Time that the last message transmitted
uint32_t doneTime = 0;  // Time to expire when CDDONE takes too long
uint32_t statTime = 0;  // last time we sent a stat message to server
uint32_t pulltime = 0;  // last time we sent a pull_data request to server

#if A_SERVER == 1
uint32_t wwwtime = 0;
#endif
#if NTP_INTR == 0
uint32_t ntptimer = 0;
#endif

#if GATEWAYNODE == 1
uint16_t frameCount = 0; // We write this to SPIFF file
#endif

// volatile bool inSPI This initial value of mutex is to be free,
// which means that its value is 1 (!)
//
int mutexSPI = 1;

// ----------------------------------------------------------------------------
// DIE is not used actively in the source code anymore.
// It is replaced by a Serial.print command so we know that we have a problem
// somewhere.
// There are at least 3 other ways to restart the ESP. Pick one if you want.
// ----------------------------------------------------------------------------
void die(const char *s)
{
	Serial.println(s);
	if (debug >= 2)
		Serial.flush();

	delay(50);
	// system_restart();									// SDK function
	// ESP.reset();
	abort(); // Within a second
}

// ----------------------------------------------------------------------------
// gway_failed is a function called by ASSERT in ESP-sc-gway.h
//
// ----------------------------------------------------------------------------
void gway_failed(const char *file, uint16_t line)
{
	Serial.print(F("Program failed in file: "));
	Serial.print(file);
	Serial.print(F(", line: "));
	Serial.println(line);
	if (debug >= 2)
		Serial.flush();
}

// ----------------------------------------------------------------------------
// Convert a float to string for printing
// Parameters:
//	f is float value to convert
//	p is precision in decimal digits
//	val is character array for results
// ----------------------------------------------------------------------------
void ftoa(float f, char *val, int p)
{
	int j = 1;
	int ival, fval;
	char b[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	for (int i = 0; i < p; i++)
	{
		j = j * 10;
	}

	ival = (int)f;				  // Make integer part
	fval = (int)((f - ival) * j); // Make fraction. Has same sign as integer part
	if (fval < 0)
		fval = -fval; // So if it is negative make fraction positive again.
					  // sprintf does NOT fit in memory
	if ((f < 0) && (ival == 0))
		strcat(val, "-");
	strcat(val, itoa(ival, b, 10)); // Copy integer part first, base 10, null terminated
	strcat(val, ".");				// Copy decimal point

	itoa(fval, b, 10); // Copy fraction part base 10
	for (int i = 0; i < (p - strlen(b)); i++)
	{
		strcat(val, "0"); // first number of 0 of faction?
	}

	// Fraction can be anything from 0 to 10^p , so can have less digits
	strcat(val, b);
}

// ============================================================================
// NTP TIME functions

// ----------------------------------------------------------------------------
// Send the request packet to the NTP server.
//
// ----------------------------------------------------------------------------
int sendNtpRequest(IPAddress timeServerIP)
{
	const int NTP_PACKET_SIZE = 48; // Fixed size of NTP record
	byte packetBuffer[NTP_PACKET_SIZE];

	memset(packetBuffer, 0, NTP_PACKET_SIZE); // Zeroise the buffer.

	packetBuffer[0] = 0b11100011; // LI, Version, Mode
	packetBuffer[1] = 0;		  // Stratum, or type of clock
	packetBuffer[2] = 6;		  // Polling Interval
	packetBuffer[3] = 0xEC;		  // Peer Clock Precision
	// 8 bytes of zero for Root Delay & Root Dispersion
	packetBuffer[12] = 49;
	packetBuffer[13] = 0x4E;
	packetBuffer[14] = 49;
	packetBuffer[15] = 52;

	if (!sendUdp((IPAddress)timeServerIP, (int)123, packetBuffer, NTP_PACKET_SIZE))
	{
		gwayConfig.ntpErr++;
		gwayConfig.ntpErrTime = now();
		return (0);
	}
	return (1);
}

// ----------------------------------------------------------------------------
// Get the NTP time from one of the time servers
// Note: As this function is called from SyncINterval in the background
//	make sure we have no blocking calls in this function
// ----------------------------------------------------------------------------
time_t getNtpTime()
{
	gwayConfig.ntps++;

	if (!sendNtpRequest(ntpServer)) // Send the request for new time
	{
		if ((debug >= 0) && (pdebug & P_MAIN))
			Serial.println(F("M sendNtpRequest failed"));
		return (0);
	}

	const int NTP_PACKET_SIZE = 48; // Fixed size of NTP record
	byte packetBuffer[NTP_PACKET_SIZE];
	memset(packetBuffer, 0, NTP_PACKET_SIZE); // Set buffer cntents to zero

	uint32_t beginWait = millis();
	delay(10);
	while (millis() - beginWait < 1500)
	{
		int size = Udp.parsePacket();
		if (size >= NTP_PACKET_SIZE)
		{

			if (Udp.read(packetBuffer, NTP_PACKET_SIZE) < NTP_PACKET_SIZE)
			{
				break;
			}
			else
			{
				// Extract seconds portion.
				unsigned long secs;
				secs = packetBuffer[40] << 24;
				secs |= packetBuffer[41] << 16;
				secs |= packetBuffer[42] << 8;
				secs |= packetBuffer[43];
				// UTC is 1 TimeZone correction when no daylight saving time
				return (secs - 2208988800UL + NTP_TIMEZONES * SECS_IN_HOUR);
			}
			Udp.flush();
		}
		delay(100); // Wait 100 millisecs, allow kernel to act when necessary
	}

	Udp.flush();

	// If we are here, we could not read the time from internet
	// So increase the counter
	gwayConfig.ntpErr++;
	gwayConfig.ntpErrTime = now();
#if DUSB >= 1
	if ((debug >= 0) && (pdebug & P_MAIN))
	{
		Serial.println(F("M getNtpTime:: read failed"));
	}
#endif
	return (0); // return 0 if unable to get the time
}

// ----------------------------------------------------------------------------
// Set up regular synchronization of NTP server and the local time.
// ----------------------------------------------------------------------------
#if NTP_INTR == 1
void setupTime()
{
	setSyncProvider(getNtpTime);
	setSyncInterval(_NTP_INTERVAL);
}
#endif

// ============================================================================
// MAIN PROGRAM CODE (SETUP AND LOOP)

// ----------------------------------------------------------------------------
// Setup code (one time)
// _state is S_INIT
// ----------------------------------------------------------------------------
void setup()
{

	char MAC_char[19]; // XXX Unbelievable
	MAC_char[18] = 0;

	Serial.begin(_BAUDRATE); // As fast as possible for bus
	delay(100);

#if _GPS == 1
	// Pins are define in LoRaModem.h together with other pins
	Serial1.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX); // PIN 12-TX 15-RX
#endif

#ifdef ESP32
#if DUSB >= 1
	Serial.print(F("ESP32 defined, freq="));
	Serial.print(freqs[0].upFreq);
	Serial.println();
#endif
#endif
#ifdef ARDUINO_ARCH_ESP32
#if DUSB >= 1
	Serial.println(F("ARDUINO_ARCH_ESP32 defined"));
#endif
#endif

#if DUSB >= 1
	Serial.flush();

	delay(500);

	if (SPIFFS.begin())
	{
		Serial.println(F("SPIFFS init success"));
	}
	else
	{
	}
#endif
#if _SPIFF_FORMAT >= 1
#if DUSB >= 1
	if ((debug >= 0) && (pdebug & P_MAIN))
	{
		Serial.println(F("M Format Filesystem ... "));
	}
#endif
	SPIFFS.format(); // Normally disabled. Enable only when SPIFFS corrupt
#if DUSB >= 1
	if ((debug >= 0) && (pdebug & P_MAIN))
	{
		Serial.println(F("Done"));
	}
#endif
#endif

	Serial.print(F("Assert="));
#if defined CFG_noassert
	Serial.println(F("No Asserts"));
#else
	Serial.println(F("Do Asserts"));
#endif

#if OLED >= 1
	init_oLED(); // When done display "STARTING" on OLED
#endif

	delay(500);
	yield();
#if DUSB >= 1
	if (debug >= 1)
	{
		Serial.print(F("debug="));
		Serial.println(debug);
		yield();
	}
#endif

	WiFi.mode(WIFI_STA);
	WiFi.setAutoConnect(true);
	//WiFi.begin();

	WlanReadWpa(); // Read the last Wifi settings from SPIFFS into memory

	WiFi.macAddress(MAC_array);

	sprintf(MAC_char, "%02x:%02x:%02x:%02x:%02x:%02x",
			MAC_array[0], MAC_array[1], MAC_array[2], MAC_array[3], MAC_array[4], MAC_array[5]);
	Serial.print("MAC: ");
	Serial.print(MAC_char);
	Serial.print(F(", len="));
	Serial.println(strlen(MAC_char));

	// We start by connecting to a WiFi network, set hostname
	char hostname[12];

	// Setup WiFi UDP connection. Give it some time and retry x times..
	while (WlanConnect(0) <= 0)
	{
		Serial.print(F("Error Wifi network connect "));
		Serial.println();
		yield();
	}

	// After there is a WiFi router connection, we can also set the hostname.
#if ESP32_ARCH == 1
	sprintf(hostname, "%s%02x%02x%02x", "esp32-", MAC_array[3], MAC_array[4], MAC_array[5]);
	WiFi.setHostname(hostname);
#else
	sprintf(hostname, "%s%02x%02x%02x", "esp8266-", MAC_array[3], MAC_array[4], MAC_array[5]);
	wifi_station_set_hostname(hostname);
#endif

	Serial.print(F("Host "));
#if ESP32_ARCH == 1
	Serial.print(WiFi.getHostname());
#else
	Serial.print(wifi_station_get_hostname());
#endif
	Serial.print(F(" WiFi Connected to "));
	Serial.print(WiFi.SSID());
	Serial.print(F(" on IP="));
	Serial.print(WiFi.localIP());
	Serial.println();

	delay(500);
	// If we are here we are connected to WLAN

#if defined(_UDPROUTER)
	// So now test the UDP function
	if (!connectUdp())
	{
		Serial.println(F("Error connectUdp"));
	}
#elif defined(_TTNROUTER)
	if (!connectTtn())
	{
		Serial.println(F("Error connectTtn"));
	}
#else
	Serial.print(F("Setup:: No UDP or TCP Connection defined"));

#endif
	delay(200);

#ifndef CFG_sx1262_radio
	// Pins are defined and set in loraModem.h
	pinMode(pins.ss, OUTPUT);
	pinMode(pins.rst, OUTPUT);
	pinMode(pins.dio0, INPUT); // This pin is interrupt
	pinMode(pins.dio1, INPUT); // This pin is interrupt
							   //pinMode(pins.dio2, INPUT);

	// Init the SPI pins
#if ESP32_ARCH == 1
	SPI.begin(SCK, MISO, MOSI, SS);
#else
	SPI.begin();
#endif
#endif
	delay(500);

	// We choose the Gateway ID to be the Ethernet Address of our Gateway card
	// display results of getting hardware address
	//

	Serial.print("Gateway ID: ");
	printHexDigit(MAC_array[0]);
	printHexDigit(MAC_array[1]);
	printHexDigit(MAC_array[2]);
	printHexDigit(0xFF);
	printHexDigit(0xFF);
	printHexDigit(MAC_array[3]);
	printHexDigit(MAC_array[4]);
	printHexDigit(MAC_array[5]);

	Serial.print(", Listening at SF");
	Serial.print(sf);
	Serial.print(" on ");
	Serial.print((double)freqs[ifreq].upFreq / 1000000);
	Serial.println(" MHz.");

	if (!WiFi.hostByName(NTP_TIMESERVER, ntpServer)) // Get IP address of Timeserver
	{
		die("Setup:: ERROR hostByName NTP");
	};
	delay(100);
#ifdef _TTNSERVER
	if (!WiFi.hostByName(_TTNSERVER, ttnServer)) // Use DNS to get server IP once
	{
		die("Setup:: ERROR hostByName TTN");
	};
	delay(100);
#endif
#ifdef _THINGSERVER
	if (!WiFi.hostByName(_THINGSERVER, thingServer))
	{
		die("Setup:: ERROR hostByName THING");
	}
	delay(100);
#endif

	// The Over the Air updates are supported when we have a WiFi connection.
	// The NTP time setting does not have to be precise for this function to work.
#if A_OTA == 1
	setupOta(hostname); // Uses wwwServer
#endif

	// Set the NTP Time
	// As long as the time has not been set we try to set the time.
#if NTP_INTR == 1
	setupTime(); // Set NTP time host and interval
#else
	// If not using the standard libraries, do a manual setting
	// of the time. This method works more reliable than the
	// interrupt driven method.

	//setTime((time_t)getNtpTime());
	while (timeStatus() == timeNotSet)
	{
#if DUSB >= 1
		if ((debug >= 0) && (pdebug & P_MAIN))
			Serial.println(F("M setupTime:: Time not set (yet)"));
#endif
		delay(500);
		time_t newTime;
		newTime = (time_t)getNtpTime();
		if (newTime != 0)
			setTime(newTime);
	}
	// When we are here we succeeded in getting the time
	startTime = now(); // Time in seconds
#if DUSB >= 1
	Serial.print("Time: ");
	printTime();
	Serial.println();
#endif
	writeGwayCfg(CONFIGFILE);
#if DUSB >= 1
	Serial.println(F("Gateway configuration saved"));
#endif
#endif //NTP_INTR

#if A_SERVER == 1
	// Setup the webserver
	setupWWW();
#endif

	delay(100); // Wait after setup

	// Setup and initialise LoRa state machine of _loramModem.ino
	_state = S_INIT;
	initLoraModem();

	if (_cad)
	{
		_state = S_SCAN;
		sf = SF7;
		cadScanner(); // Always start at SF7
	}
	else
	{
		_state = S_RX;
		rxLoraModem();
	}
	LoraUp.payLoad[0] = 0;
	LoraUp.payLength = 0; // Init the length to 0

#ifndef CFG_sx1262_radio
	// init interrupt handlers, which are shared for GPIO15 / D8,
	// we switch on HIGH interrupts
	if (pins.dio0 == pins.dio1)
	{
		//SPI.usingInterrupt(digitalPinToInterrupt(pins.dio0));
		attachInterrupt(pins.dio0, Interrupt_0, RISING); // Share interrupts
	}
	// Or in the traditional Comresult case
	else
	{
		//SPI.usingInterrupt(digitalPinToInterrupt(pins.dio0));
		//SPI.usingInterrupt(digitalPinToInterrupt(pins.dio1));
		attachInterrupt(pins.dio0, Interrupt_0, RISING); // Separate interrupts
		attachInterrupt(pins.dio1, Interrupt_1, RISING); // Separate interrupts
	}
#endif
	writeConfig(CONFIGFILE, &gwayConfig); // Write config

	// activate OLED display
#if OLED >= 1
	acti_oLED();
	addr_oLED();
#endif

	Serial.println(F("--------------------------------------"));
} //setup

// ----------------------------------------------------------------------------
// LOOP
// This is the main program that is executed time and time again.
// We need to give way to the backend WiFi processing that
// takes place somewhere in the ESP8266 firmware and therefore
// we include yield() statements at important points.
//
// Note: If we spend too much time in user processing functions
// and the backend system cannot do its housekeeping, the watchdog
// function will be executed which means effectively that the
// program crashes.
// We use yield() a lot to avoid ANY watch dog activity of the program.
//
// NOTE2: For ESP make sure not to do large array declarations in loop();
// ----------------------------------------------------------------------------
void loop()
{
	// uint32_t uSeconds; // micro seconds
	int packetSize;
	uint32_t nowSeconds = now();

	if (!otaActive) // Do not handle anything while OTA is updating
	{
#ifndef CFG_sx1262_radio
		// check for event value, which means that an interrupt has arrived.
		// In this case we handle the interrupt ( e.g. message received)
		// in userspace in loop().
		//
		stateMachine(); // do the state machine
#else
		// Handle Radio events
		Radio.IrqProcess();
#endif
		// After a quiet period, make sure we reinit the modem and state machine.
		// The interval is in seconds (about 15 seconds) as this re-init
		// is a heavy operation.
		// SO it will kick in if there are not many messages for the gateway.
		// Note: Be careful that it does not happen too often in normal operation.
		//
		if (((nowSeconds - statr[0].tmst) > _MSG_INTERVAL) &&
			(msgTime <= statr[0].tmst))
		{
#if DUSB >= 1
			if ((debug >= 1) && (pdebug & P_MAIN))
			{
				Serial.print("M REINIT:: ");
				Serial.print(_MSG_INTERVAL);
				Serial.print(F(" "));
				SerialStat(0);
			}
#endif

#ifndef CFG_sx1262_radio
			// startReceiver() ??
			if ((_cad) || (_hop))
			{
				_state = S_SCAN;
				sf = SF7;
				cadScanner();
			}
			else
			{
				_state = S_RX;
				rxLoraModem();
			}
			writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t)0x00);
			writeRegister(REG_IRQ_FLAGS, (uint8_t)0xFF); // Reset all interrupt flags
#else
			Radio.Sleep();
			delay(100);
			Radio.Rx(0);
#endif
			msgTime = nowSeconds;
		}
#if A_SERVER == 1
		// Handle the Web server part of this sketch. Mainly used for administration
		// and monitoring of the node. This function is important so it is called at the
		// start of the loop() function.
		yield();
		server.handleClient();
#endif

		// If event is set, we know that we have a (soft) interrupt.
		// After all necessary web/OTA services are scanned, we will
		// reloop here for timing purposes.
		// Do as less yield() as possible.
		// XXX 180326
		if (_event == 1)
		{
			return;
		}
		else
			yield();

		// If we are not connected, try to connect.
		// We will not read Udp in this loop cycle then
		if (WlanConnect(1) < 0)
		{
#if DUSB >= 1
			if ((debug >= 0) && (pdebug & P_MAIN))
				Serial.println(F("M ERROR reconnect WLAN"));
#endif
			yield();
			return; // Exit loop if no WLAN connected
		}

		// So if we are connected
		// Receive UDP PUSH_ACK messages from server. (*2, par. 3.3)
		// This is important since the TTN broker will return confirmation
		// messages on UDP for every message sent by the gateway. So we have to consume them.
		// As we do not know when the server will respond, we test in every loop.
		//
		else
		{
			while ((packetSize = Udp.parsePacket()) > 0)
			{
#if DUSB >= 2
				Serial.println(F("loop:: readUdp calling"));
#endif
				// DOWNSTREAM
				// Packet may be PKT_PUSH_ACK (0x01), PKT_PULL_ACK (0x03) or PKT_PULL_RESP (0x04)
				// This command is found in byte 4 (buffer[3])
				if (readUdp(packetSize) <= 0)
				{
#if DUSB >= 1
					if ((debug > 0) && (pdebug & P_MAIN))
						Serial.println(F("M readUdp error"));
#endif
					break;
				}
				// Now we know we succesfully received message from host
				else
				{
					//_event=1;									// Could be done double if more messages received
				}
			}
		}

		yield(); // XXX 26/12/2017

		// stat PUSH_DATA message (*2, par. 4)
		//

		if ((nowSeconds - statTime) >= _STAT_INTERVAL)
		{ // Wake up every xx seconds
#if DUSB >= 1
			if ((debug >= 1) && (pdebug & P_MAIN))
			{
				Serial.print(F("M STAT:: ..."));
				Serial.flush();
			}
#endif
			sendstat(); // Show the status message and send to server
#if DUSB >= 1
			if ((debug >= 1) && (pdebug & P_MAIN))
			{
				Serial.println(F(" done"));
				if (debug >= 2)
					Serial.flush();
			}
#endif

			// If the gateway behaves like a node, we do from time to time
			// send a node message to the backend server.
			// The Gateway node emessage has nothing to do with the STAT_INTERVAL
			// message but we schedule it in the same frequency.
			//
#if GATEWAYNODE == 1
			if (gwayConfig.isNode)
			{
				// Give way to internal some Admin if necessary
				yield();

				// If the 1ch gateway is a sensor itself, send the sensor values
				// could be battery but also other status info or sensor info

				if (sensorPacket() < 0)
				{
#if DUSB >= 1
					Serial.println(F("sensorPacket: Error"));
#endif
				}
			}
#endif
			statTime = nowSeconds;
		}

		yield();

		// send PULL_DATA message (*2, par. 4)
		//
		nowSeconds = now();
		if ((nowSeconds - pulltime) >= _PULL_INTERVAL)
		{ // Wake up every xx seconds
#if DUSB >= 1
			if ((debug >= 2) && (pdebug & P_MAIN))
			{
				Serial.println(F("M PULL"));
				if (debug >= 1)
					Serial.flush();
			}
#endif
			pullData(); // Send PULL_DATA message to server
			startReceiver();

			pulltime = nowSeconds;
		}

		// If we do our own NTP handling (advisable)
		// We do not use the timer interrupt but use the timing
		// of the loop() itself which is better for SPI
#if NTP_INTR == 0
		// Set the time in a manual way. Do not use setSyncProvider
		// as this function may collide with SPI and other interrupts
		yield(); // 26/12/2017
		nowSeconds = now();
		if (nowSeconds - ntptimer >= _NTP_INTERVAL)
		{
			yield();
			time_t newTime;
			newTime = (time_t)getNtpTime();
			if (newTime != 0)
				setTime(newTime);
			ntptimer = nowSeconds;
		}
#endif
	}

#if A_OTA == 1
	// Perform Over the Air (OTA) update if enabled and requested by user.
	// It is important to put this function early in loop() as it is
	// not called frequently but it should always run when called.
	//
	yield();
	ArduinoOTA.handle();
#endif

} //loop
