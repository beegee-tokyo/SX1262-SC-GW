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
// This file describes the includes necessary for ESP Filesystem.
// At this moment there is only one record written to the ESP8266
// filesystem. We can add more info, which makes the gateway even more usable,
// however for large data we should only append to the existing file used.
// This also means we'll have to check for available space so we won't run out of
// storage space to quickly.
// One way would be to use let's say 10 files of each 10000 lines and when full
// delete the first file and start writing on a new one (for example)

//
// Define Pattern debug settings, this allows debugging per
// module rather than per level. See also pdebug.
//
#define P_SCAN 0x01
#define P_CAD 0x02
#define P_RX 0x04
#define P_TX 0x08
#define P_PRE 0x10
#define P_MAIN 0x20
#define P_GUI 0x40
#define P_RADIO 0x80

// Define a log record to be written to the log file
// Keep logfiles SHORT in name! to save memory
#if STAT_LOG == 1

// We do keep admin of logfiles by number
#define LOGFILEMAX 10
#define LOGFILEREC 100

#ifndef LORAFILES_H
#define LORAFILES_H
// Definition of the configuration record that is read at startup and written
// when settings are changed.

struct espGwayConfig
{
	uint16_t fcnt;   // =0 as init value	XXX Could be 32 bit in size
	uint16_t boots;  // Number of restarts made by the gateway after reset
	uint16_t resets; // Number of statistics resets
	uint16_t views;  // Number of sendWebPage() calls
	uint16_t wifis;  // Number of WiFi Setups
	uint16_t reents; // Number of re-entrant interrupt handler calls
	uint16_t ntpErr; // Number of UTP requests that failed
	uint16_t ntps;

	uint32_t ntpErrTime; // Record the time of the last NTP error
	uint8_t ch;			 // index to freqs array, freqs[ifreq]=868100000 default
	uint8_t sf;			 // range from SF7 to SF12
	uint8_t debug;		 // range 0 to 4
	uint8_t pdebug;		 // pattern debug,

	uint16_t logFileRec; // Logging File Record number
	uint16_t logFileNo;  // Logging File Number
	uint16_t logFileNum; // Number of log files

	bool cad;	 // is CAD enabled?
	bool hop;	 // Is HOP enabled (Note: default be disabled)
	bool isNode;  // Is gateway node enabled
	bool refresh; // Is WWW browser refresh enabled
	bool expert;

	String ssid; // SSID of the last connected WiFi Network
	String pass; // Password of WiFi network
};
#endif // LORAFILES_H
#endif
