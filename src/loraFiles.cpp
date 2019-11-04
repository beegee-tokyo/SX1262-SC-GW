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

espGwayConfig gwayConfig;

// ============================================================================
// LORA SPIFFS FILESYSTEM FUNCTIONS
//
// The LoRa supporting functions are in the section below

// ----------------------------------------------------------------------------
// Supporting function to readConfig
// ----------------------------------------------------------------------------
void id_print(String id, String val)
{
#if DUSB >= 1
	if ((debug >= 0) && (pdebug & P_MAIN))
	{
		Serial.print(id);
		Serial.print(F("=\t"));
		Serial.println(val);
	}
#endif
}

// ----------------------------------------------------------------------------
// INITCONFIG; Init the gateway configuration file
// Espcecially when calling SPIFFS.format() the gateway is left in an init
// which is not very well defined. This function will init some of the settings
// to well known settings.
// ----------------------------------------------------------------------------
int initConfig(struct espGwayConfig *c)
{
	(*c).ch = 0;
	(*c).sf = _SPREADING;
	(*c).debug = 1;
	(*c).pdebug = P_GUI;
	(*c).cad = _CAD;
	(*c).hop = false;
	(*c).expert = false;
	return 0; //Ldo: should return something
}

// ----------------------------------------------------------------------------
// Read the gateway configuration file
// ----------------------------------------------------------------------------
int readConfig(const char *fn, struct espGwayConfig *c)
{

	int tries = 0;
#if DUSB >= 1
	Serial.println(F("readConfig:: Starting "));
#endif
	if (!SPIFFS.exists(fn))
	{
#if DUSB >= 1
		if ((debug >= 0) && (pdebug & P_MAIN))
			Serial.print(F("M ERR:: readConfig, file="));
		Serial.print(fn);
		Serial.println(F(" does not exist .. Formatting"));
#endif
		SPIFFS.format();
		initConfig(c);
		return (-1);
	}

	File f = SPIFFS.open(fn, "r");
	if (!f)
	{
		Serial.println(F("ERROR:: SPIFFS open failed"));
		return (-1);
	}

	while (f.available())
	{

#if DUSB >= 1
		if ((debug >= 0) && (pdebug & P_MAIN))
		{
			Serial.print('.');
		}
#endif
		// If we wait for more than 10 times, reformat the filesystem
		// We do this so that the system will be responsive (over OTA for example).
		//
		if (tries >= 10)
		{
			f.close();
#if DUSB >= 1
			if ((debug >= 0) && (pdebug & P_MAIN))
				Serial.println(F("Formatting"));
#endif
			SPIFFS.format();
			initConfig(c);
			f = SPIFFS.open(fn, "r");
			tries = 0;
		}

		String id = f.readStringUntil('='); // C++ thing
		String val = f.readStringUntil('\n');

		if (id == "SSID")
		{ // WiFi SSID
			id_print(id, val);
			(*c).ssid = val; // val contains ssid, we do NO check
		}
		else if (id == "PASS")
		{ // WiFi Password
			id_print(id, val);
			(*c).pass = val;
		}
		else if (id == "CH")
		{ // Frequency Channel
			id_print(id, val);
			(*c).ch = (uint32_t)val.toInt();
		}
		else if (id == "SF")
		{ // Spreading Factor
			id_print(id, val);
			(*c).sf = (uint32_t)val.toInt();
		}
		else if (id == "FCNT")
		{ // Frame Counter
			id_print(id, val);
			(*c).fcnt = (uint32_t)val.toInt();
		}
		else if (id == "DEBUG")
		{ // Debug Level
			id_print(id, val);
			(*c).debug = (uint8_t)val.toInt();
		}
		else if (id == "PDEBUG")
		{ // pDebug Pattern
			Serial.print(F("PDEBUG="));
			Serial.println(val);
			(*c).pdebug = (uint8_t)val.toInt();
		}
		else if (id == "CAD")
		{ // CAD setting
			Serial.print(F("CAD="));
			Serial.println(val);
			(*c).cad = (uint8_t)val.toInt();
		}
		else if (id == "HOP")
		{ // HOP setting
			Serial.print(F("HOP="));
			Serial.println(val);
			(*c).hop = (uint8_t)val.toInt();
		}
		else if (id == "BOOTS")
		{ // BOOTS setting
			id_print(id, val);
			(*c).boots = (uint8_t)val.toInt();
		}
		else if (id == "RESETS")
		{ // RESET setting
			id_print(id, val);
			(*c).resets = (uint8_t)val.toInt();
		}
		else if (id == "WIFIS")
		{ // WIFIS setting
			id_print(id, val);
			(*c).wifis = (uint8_t)val.toInt();
		}
		else if (id == "VIEWS")
		{ // VIEWS setting
			id_print(id, val);
			(*c).views = (uint8_t)val.toInt();
		}
		else if (id == "NODE")
		{ // NODE setting
			id_print(id, val);
			(*c).isNode = (uint8_t)val.toInt();
		}
		else if (id == "REFR")
		{ // REFR setting
			id_print(id, val);
			(*c).refresh = (uint8_t)val.toInt();
		}
		else if (id == "REENTS")
		{ // REENTS setting
			id_print(id, val);
			(*c).reents = (uint8_t)val.toInt();
		}
		else if (id == "NTPERR")
		{ // NTPERR setting
			id_print(id, val);
			(*c).ntpErr = (uint8_t)val.toInt();
		}
		else if (id == "NTPETIM")
		{ // NTPERR setting
			id_print(id, val);
			(*c).ntpErrTime = (uint32_t)val.toInt();
		}
		else if (id == "NTPS")
		{ // NTPS setting
			id_print(id, val);
			(*c).ntps = (uint8_t)val.toInt();
		}
		else if (id == "FILENO")
		{ // log FILENO setting
			id_print(id, val);
			(*c).logFileNo = (uint8_t)val.toInt();
		}
		else if (id == "FILEREC")
		{ // FILEREC setting
			id_print(id, val);
			(*c).logFileRec = (uint16_t)val.toInt();
		}
		else if (id == "FILENUM")
		{ // FILEREC setting
			id_print(id, val);
			(*c).logFileNum = (uint16_t)val.toInt();
		}
		else if (id == "EXPERT")
		{ // FILEREC setting
			id_print(id, val);
			(*c).expert = (uint8_t)val.toInt();
			(*c).expert = true;
		}
		else
		{
			tries++;
		}
	}
	f.close();
#if DUSB >= 1
	if (debug >= 0)
	{
		Serial.println('#');
	}
#endif
	Serial.println();
	return (1);
}

// ----------------------------------------------------------------------------
// Write the current gateway configuration to SPIFFS. First copy all the
// separate data items to the gwayConfig structure
//
// ----------------------------------------------------------------------------
int writeGwayCfg(const char *fn)
{

	gwayConfig.ssid = WiFi.SSID();
	gwayConfig.pass = WiFi.psk(); // XXX We should find a way to store the password too
	gwayConfig.ch = ifreq;		  // Frequency Index
	gwayConfig.sf = (uint8_t)sf;  // Spreading Factor
	gwayConfig.debug = debug;
	gwayConfig.pdebug = pdebug;
	gwayConfig.cad = _cad;
	gwayConfig.hop = _hop;
#if GATEWAYNODE == 1
	gwayConfig.fcnt = frameCount;
#endif
	return (writeConfig(fn, &gwayConfig));
}

// ----------------------------------------------------------------------------
// Write the configuration as found in the espGwayConfig structure
// to SPIFFS
// Parameters:
//		fn; Filename
//		c; struct config
// Returns:
//		1 when successful, -1 on error
// ----------------------------------------------------------------------------
int writeConfig(const char *fn, struct espGwayConfig *c)
{

	if (!SPIFFS.exists(fn))
	{
		Serial.print("WARNING:: writeConfig, file not exists, formatting ");
		SPIFFS.format();
		initConfig(c); // XXX make all initial declarations here if config vars need to have a value
		Serial.println(fn);
	}
	File f = SPIFFS.open(fn, "w");
	if (!f)
	{
		Serial.print("ERROR:: writeConfig, open file=");
		Serial.print(fn);
		Serial.println();
		return (-1);
	}

	f.print("SSID");
	f.print('=');
	f.print((*c).ssid);
	f.print('\n');
	f.print("PASS");
	f.print('=');
	f.print((*c).pass);
	f.print('\n');
	f.print("CH");
	f.print('=');
	f.print((*c).ch);
	f.print('\n');
	f.print("SF");
	f.print('=');
	f.print((*c).sf);
	f.print('\n');
	f.print("FCNT");
	f.print('=');
	f.print((*c).fcnt);
	f.print('\n');
	f.print("DEBUG");
	f.print('=');
	f.print((*c).debug);
	f.print('\n');
	f.print("PDEBUG");
	f.print('=');
	f.print((*c).pdebug);
	f.print('\n');
	f.print("CAD");
	f.print('=');
	f.print((*c).cad);
	f.print('\n');
	f.print("HOP");
	f.print('=');
	f.print((*c).hop);
	f.print('\n');
	f.print("NODE");
	f.print('=');
	f.print((*c).isNode);
	f.print('\n');
	f.print("BOOTS");
	f.print('=');
	f.print((*c).boots);
	f.print('\n');
	f.print("RESETS");
	f.print('=');
	f.print((*c).resets);
	f.print('\n');
	f.print("WIFIS");
	f.print('=');
	f.print((*c).wifis);
	f.print('\n');
	f.print("VIEWS");
	f.print('=');
	f.print((*c).views);
	f.print('\n');
	f.print("REFR");
	f.print('=');
	f.print((*c).refresh);
	f.print('\n');
	f.print("REENTS");
	f.print('=');
	f.print((*c).reents);
	f.print('\n');
	f.print("NTPETIM");
	f.print('=');
	f.print((*c).ntpErrTime);
	f.print('\n');
	f.print("NTPERR");
	f.print('=');
	f.print((*c).ntpErr);
	f.print('\n');
	f.print("NTPS");
	f.print('=');
	f.print((*c).ntps);
	f.print('\n');
	f.print("FILEREC");
	f.print('=');
	f.print((*c).logFileRec);
	f.print('\n');
	f.print("FILENO");
	f.print('=');
	f.print((*c).logFileNo);
	f.print('\n');
	f.print("FILENUM");
	f.print('=');
	f.print((*c).logFileNum);
	f.print('\n');
	f.print("EXPERT");
	f.print('=');
	f.print((*c).expert);
	f.print('\n');

	f.close();
	return (1);
}

// ----------------------------------------------------------------------------
// Add a line with statistics to the log.
//
// We put the check in the function to protect against calling
// the function without STAT_LOG being proper defined
// ToDo: Store the fileNo and the fileRec in the status file to save for
// restarts
// Parameters:
//		line; char array with characters to write to log
//		cnt;
// Returns:
//		<none>
// ----------------------------------------------------------------------------
void addLog(const unsigned char *line, int cnt)
{
#if STAT_LOG == 1
	char fn[16];

	if (gwayConfig.logFileRec > LOGFILEREC)
	{							   // Have to make define for this
		gwayConfig.logFileRec = 0; // In new logFile start with record 0
		gwayConfig.logFileNo++;	// Increase file ID
		gwayConfig.logFileNum++;   // Increase number of log files
	}
	gwayConfig.logFileRec++;

	// If we have too many logfies, delete the oldest
	//
	if (gwayConfig.logFileNum > LOGFILEMAX)
	{
		sprintf(fn, "/log-%d", gwayConfig.logFileNo - LOGFILEMAX);
#if DUSB >= 1
		if ((debug >= 0) && (pdebug & P_GUI))
		{
			Serial.print(F("G addLog:: Too many logfile, deleting="));
			Serial.println(fn);
		}
#endif
		SPIFFS.remove(fn);
		gwayConfig.logFileNum--;
	}

	// Make sure we have the right fileno
	sprintf(fn, "/log-%d", gwayConfig.logFileNo);

	// If there is no SPIFFS, Error
	// Make sure to write the config record/line also
	if (!SPIFFS.exists(fn))
	{
#if DUSB >= 1
		if ((debug >= 1) && (pdebug & P_GUI))
		{
			Serial.print(F("G ERROR:: addLog:: file="));
			Serial.print(fn);
			Serial.print(F(" does not exist .. rec="));
			Serial.print(gwayConfig.logFileRec);
			Serial.println();
		}
#endif
	}

	File f = SPIFFS.open(fn, "a");
	if (!f)
	{
#if DUSB >= 1
		if ((debug >= 1) && (pdebug & P_GUI))
		{
			Serial.println("G file open failed=");
			Serial.println(fn);
		}
#endif
		return; // If file open failed, return
	}

	int i;
#if DUSB >= 1
	if ((debug >= 1) && (pdebug & P_GUI))
	{
		Serial.print(F("G addLog:: fileno="));
		Serial.print(gwayConfig.logFileNo);
		Serial.print(F(", rec="));
		Serial.print(gwayConfig.logFileRec);

		Serial.print(F(": "));

		for (i = 0; i < 12; i++)
		{ // The first 12 bytes contain non printable characters
			Serial.print(line[i], HEX);
			Serial.print(' ');
		}
		Serial.print((char *)&line[i]); // The rest if the buffer contains ascii

		Serial.println();
	}
#endif //DUSB

	for (i = 0; i < 12; i++)
	{ // The first 12 bytes contain non printable characters
		//	f.print(line[i],HEX);
		f.print('*');
	}
	f.write(&(line[i]), cnt - 12); // write/append the line to the file
	f.print('\n');
	f.close(); // Close the file after appending to it

#endif //STAT_LOG
}

// ----------------------------------------------------------------------------
// Print (all) logfiles
//
// ----------------------------------------------------------------------------
void printLog()
{
	char fn[16];
	int i = 0;
#if DUSB >= 1
	while (i < LOGFILEMAX)
	{
		sprintf(fn, "/log-%d", gwayConfig.logFileNo - i);
		if (!SPIFFS.exists(fn))
			break; // break the loop

		// Open the file for reading
		File f = SPIFFS.open(fn, "r");

		int j;
		for (j = 0; j < LOGFILEREC; j++)
		{

			String s = f.readStringUntil('\n');
			if (s.length() == 0)
				break;

			Serial.println(s.substring(12)); // Skip the first 12 Gateway specific binary characters
			yield();
		}
		i++;
	}
#endif
} //printLog

// ----------------------------------------------------------------------------
// listDir
//	List the directory and put it in
// ----------------------------------------------------------------------------
void listDir(char *dir)
{
#if DUSB >= 1

#endif
}
