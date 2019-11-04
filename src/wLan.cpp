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
// This file contains the LoRa filesystem specific code

#include "defines.h"

// Please fill in at least ONE SSID and password from your own WiFI network
// in struct wpas wpa[].
// This is needed to get the gateway working
// Note: DO NOT use the first and the last line of the stucture, these should be empty strings and
//the first line in te struct is reserved for WifiManager.
wpas wpa[] = {
	{"", ""}, // Reserved for WiFi Manager
	{"BEER", "LIKE_IT"},
	{"PH", "LIVE_THERE"},
	{"", ""}};

// ----------------------------------------------------------------------------
// WLANSTATUS prints the status of the Wlan.
// The status of the Wlan "connection" can change if we have no relation
// with the well known router anymore. Normally this relation is preserved
// but sometimes we have to reconfirm to the router again and we get the same
// address too.
// So, if the router is still in range we can "survive" with the same address
// and we may have to renew the "connection" from time to time.
// But when we loose the SSID connection, we may have to look for another router.
//
// Parameters: <none>
// Return value: Returns 1 when still WL_CONNETED, otherwise returns 0
// ----------------------------------------------------------------------------
int WlanStatus()
{

	switch (WiFi.status())
	{
	case WL_CONNECTED:
#if DUSB >= 1
		if (debug >= 0)
		{
			Serial.print(F("A WlanStatus:: CONNECTED to ")); // 3
			Serial.println(WiFi.SSID());
		}
#endif
		WiFi.setAutoReconnect(true); // Reconenct to this AP if DISCONNECTED
		return (1);
		break;

	// In case we get disconnected from the AP we loose the IP address.
	// The ESP is configured to reconnect to the last router in memory.
	case WL_DISCONNECTED:
#if DUSB >= 1
		if (debug >= 0)
		{
			Serial.print(F("A WlanStatus:: DISCONNECTED, IP=")); // 6
			Serial.println(WiFi.localIP());
		}
#endif
		//while (! WiFi.isConnected() ) {
		// Wait
		delay(1);
		//}
		return (0);
		break;

	// When still pocessing
	case WL_IDLE_STATUS:
#if DUSB >= 1
		if (debug >= 0)
		{
			Serial.println(F("A WlanStatus:: IDLE")); // 0
		}
#endif
		break;

	// This code is generated as soonas the AP is out of range
	// Whene detected, the program will search for a better AP in range
	case WL_NO_SSID_AVAIL:
#if DUSB >= 1
		if (debug >= 0)
			Serial.println(F("WlanStatus:: NO SSID")); // 1
#endif
		break;

	case WL_CONNECT_FAILED:
#if DUSB >= 1
		if (debug >= 0)
			Serial.println(F("A WlanStatus:: FAILED")); // 4
#endif
		break;

	// Never seen this code
	case WL_SCAN_COMPLETED:
#if DUSB >= 1
		if (debug >= 0)
			Serial.println(F("A WlanStatus:: SCAN COMPLETE")); // 2
#endif
		break;

	// Never seen this code
	case WL_CONNECTION_LOST:
#if DUSB >= 1
		if (debug >= 0)
			Serial.println(F("A WlanStatus:: LOST")); // 5
#endif
		break;

	// This code is generated for example when WiFi.begin() has not been called
	// before accessing WiFi functions
	case WL_NO_SHIELD:
#if DUSB >= 1
		if (debug >= 0)
			Serial.println(F("A WlanStatus:: WL_NO_SHIELD")); //
#endif
		break;

	default:
#if DUSB >= 1
		if (debug >= 0)
		{
			Serial.print(F("A WlanStatus Error:: code="));
			Serial.println(WiFi.status()); // 255 means ERROR
		}
#endif
		break;
	}
	return (-1);

} // WlanStatus

// ----------------------------------------------------------------------------
// config.txt is a text file that contains lines(!) with WPA configuration items
// Each line contains an KEY vaue pair describing the gateway configuration
//
// ----------------------------------------------------------------------------
int WlanReadWpa()
{

	readConfig(CONFIGFILE, &gwayConfig);

	if (gwayConfig.sf != (uint8_t)0)
		sf = (sf_t)gwayConfig.sf;
	ifreq = gwayConfig.ch;
	debug = gwayConfig.debug;
	pdebug = gwayConfig.pdebug;
	_cad = gwayConfig.cad;
	_hop = gwayConfig.hop;
	gwayConfig.boots++; // Every boot of the system we increase the reset

#if GATEWAYNODE == 1
	if (gwayConfig.fcnt != (uint8_t)0)
		frameCount = gwayConfig.fcnt + 10;
#endif

#if WIFIMANAGER == 1
	String ssid = gwayConfig.ssid;
	String pass = gwayConfig.pass;

	char ssidBuf[ssid.length() + 1];
	ssid.toCharArray(ssidBuf, ssid.length() + 1);
	char passBuf[pass.length() + 1];
	pass.toCharArray(passBuf, pass.length() + 1);
	Serial.print(F("WlanReadWpa: "));
	Serial.print(ssidBuf);
	Serial.print(F(", "));
	Serial.println(passBuf);

	strcpy(wpa[0].login, ssidBuf); // XXX changed from wpa[0][0] = ssidBuf
	strcpy(wpa[0].passw, passBuf);

	Serial.print(F("WlanReadWpa: <"));
	Serial.print(wpa[0].login); // XXX
	Serial.print(F(">, <"));
	Serial.print(wpa[0].passw);
	Serial.println(F(">"));
#endif
}

// ----------------------------------------------------------------------------
// Print the WPA data of last WiFiManager to the config file
// ----------------------------------------------------------------------------
#if WIFIMANAGER == 1
int WlanWriteWpa(char *ssid, char *pass)
{

#if DUSB >= 1
	if ((debug >= 0) && (pdebug & P_MAIN))
	{
		Serial.print(F("M WlanWriteWpa:: ssid="));
		Serial.print(ssid);
		Serial.print(F(", pass="));
		Serial.print(pass);
		Serial.println();
	}
#endif
	// Version 3.3 use of config file
	String s((char *)ssid);
	gwayConfig.ssid = s;

	String p((char *)pass);
	gwayConfig.pass = p;

#if GATEWAYNODE == 1
	gwayConfig.fcnt = frameCount;
#endif
	gwayConfig.ch = ifreq;
	gwayConfig.sf = sf;
	gwayConfig.cad = _cad;
	gwayConfig.hop = _hop;

	writeConfig(CONFIGFILE, &gwayConfig);
	return 1;
}
#endif

// ----------------------------------------------------------------------------
// Function to join the Wifi Network
//	It is a matter of returning to the main loop() asap and make sure in next loop
//	the reconnect is done first thing. By default the system will reconnect to the
// samen SSID as it was connected to before.
// Parameters:
//		int maxTry: Number of retries we do:
//		0: Used during Setup first CONNECT
//		1: Try once and if unsuccessful return(1);
//		x: Try x times
//
//  Returns:
//		On failure: Return -1
//		On connect: return 1
//		On Disconnect state: return 0
//
//  XXX After a few retries, the ESP8266 should be reset. Note: Switching between
//	two SSID's does the trick. Rettrying the same SSID does not.
//	Workaround is found below: Let the ESP8266 forget the SSID
//
//  NOTE: The Serial works only on debug setting and not on pdebug. This is
//	because WiFi problems would make webserver (which works on WiFi) useless.
// ----------------------------------------------------------------------------
int WlanConnect(int maxTry)
{

#if WIFIMANAGER == 1
	WiFiManager wifiManager;
#endif

	unsigned char agains = 0;
	unsigned char wpa_index = (WIFIMANAGER > 0 ? 0 : 1); // Skip over first record for WiFiManager

	// The initial setup() call is done with parameter 0
	// We clear the WiFi memory and start with previous AP.
	//
	if (maxTry == 0)
	{
		Serial.println(F("WlanConnect:: Init para 0"));
		WiFi.persistent(false);
		WiFi.mode(WIFI_OFF); // this is a temporary line, to be removed after SDK update to 1.5.4
		if (gwayConfig.ssid.length() > 0)
		{
			WiFi.begin(gwayConfig.ssid.c_str(), gwayConfig.pass.c_str());
			delay(100);
		}
	}

	// So try to connect to WLAN as long as we are not connected.
	// The try parameters tells us how many times we try before giving up
	// Value 0 is reserved for setup() first time connect
	int i = 0;

	while ((WiFi.status() != WL_CONNECTED) && (i <= maxTry))
	{

		// We try every SSID in wpa array until success
		for (int j = wpa_index; (j < (sizeof(wpa) / sizeof(wpa[0]))) && (WiFi.status() != WL_CONNECTED); j++)
		{
			// Start with well-known access points in the list
			char *ssid = wpa[j].login;
			char *password = wpa[j].passw;
#if DUSB >= 1
			if (debug >= 0)
			{
				Serial.print(i);
				Serial.print(':');
				Serial.print(j);
				Serial.print(':');
				Serial.print((long)(sizeof(wpa) / sizeof(wpa[0])));
				Serial.print(F(". WiFi connect SSID="));
				Serial.print(ssid);
				if (debug >= 1)
				{
					Serial.print(F(", pass="));
					Serial.print(password);
				}
				Serial.println();
			}
#endif
			// Count the number of times we call WiFi.begin
			gwayConfig.wifis++;

			WiFi.mode(WIFI_STA);
			delay(1000);
			WiFi.begin(ssid, password);
			delay(8000);

			// Check the connection status again, return values
			// 1 = CONNECTED
			// 0 = DISCONNECTED (will reconnect)
			// -1 = No SSID or other cause
			int stat = WlanStatus();
			if (stat == 1)
			{
				writeGwayCfg(CONFIGFILE); // XXX Write configuration to SPIFFS
				return (1);
			}

			// We increase the time for connect but try the same SSID
			// We try for 10 times
			agains = 1;
			while (((WiFi.status()) != WL_CONNECTED) && (agains < 10))
			{
				agains++;
				delay(agains * 500);
#if DUSB >= 1
				if (debug >= 0)
				{
					Serial.print(".");
				}
#endif
			}
#if DUSB >= 1
			Serial.println();
#endif
			//if ( WiFi.status() == WL_DISCONNECTED) return(0);				// XXX 180811 removed

			// Make sure that we can connect to different AP's than 1
			// this is a patch. Normally we connect to previous one.
			WiFi.persistent(false);
			WiFi.mode(WIFI_OFF); // this is a temporary line, to be removed after SDK update to 1.5.4

		} //for next WPA defined AP

		i++; // Number of times we try to connect
	}		 //while

	// If we are not connected to a well known AP
	// we can invoike WIFIMANAGER or else return unsuccessful.
	if (WiFi.status() != WL_CONNECTED)
	{
#if WIFIMANAGER == 1
#if DUSB >= 1
		Serial.println(F("Starting Access Point Mode"));
		Serial.print(F("Connect Wifi to accesspoint: "));
		Serial.print(AP_NAME);
		Serial.print(F(" and connect to IP: 192.168.4.1"));
		Serial.println();
#endif
		wifiManager.autoConnect(AP_NAME, AP_PASSWD);
		//wifiManager.startConfigPortal(AP_NAME, AP_PASSWD );
		// At this point, there IS a Wifi Access Point found and connected
		// We must connect to the local SPIFFS storage to store the access point
		//String s = WiFi.SSID();
		//char ssidBuf[s.length()+1];
		//s.toCharArray(ssidBuf,s.length()+1);
		// Now look for the password
		struct station_config sta_conf;
		wifi_station_get_config(&sta_conf);

		//WlanWriteWpa(ssidBuf, (char *)sta_conf.password);
		WlanWriteWpa((char *)sta_conf.ssid, (char *)sta_conf.password);
#else
#if DUSB >= 1
		if (debug >= 0)
		{
			Serial.println(F("WlanConnect:: Not connected after all"));
			Serial.print(F("WLAN retry="));
			Serial.print(i);
			Serial.print(F(" , stat="));
			Serial.print(WiFi.status()); // Status. 3 is WL_CONNECTED
			Serial.println();
		}
#endif // DUSB
		return (-1);
#endif
	}

	yield();
	return (1);
}
