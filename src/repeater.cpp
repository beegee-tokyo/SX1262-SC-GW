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
// This file contains code for using the single channel gateway also as a repeater node.
// Please note that for node to node communication your should change the polarity
// of messages.
//
// ============================================================================

#include "defines.h"

#if _REPEATER == 1

// Define input channel and output channel
#define _ICHAN 0
#define _OCHAN 1

#ifdef _TTNSERVER
#error "Please undefined _THINGSERVER, for REPEATER shutdown WiFi"
#endif

// Send a LoRa message out from the gateway transmitter
// XXX Maybe we should block the received ontul the message is transmitter

int sendLora(char *msg, int len)
{
	// Check when len is not exceeding maximum length
	Serial.print("sendLora:: ");

	for (int i = 0; i < len; i++)
	{
		Serial.print(msg[1], HEX);
		Serial.print('.');
	}

	if (debug >= 2)
		Serial.flush();
	return (1);
}

#endif //_REPEATER==1