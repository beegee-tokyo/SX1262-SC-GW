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
// This file contains the tcp specific code enabling to receive
// and transmit packages/messages to the TTN server using their new protocol.
//
// The TTN code has been developed as an alternative to the Semtech UDP code.
// According to the TTN website the Semtech gateway interface code is less secure and
// works on UDP which is less reliable. The new protocol of TTN should solve these issues.
//
// Initial look at the code of TTN shows that it is overly complex and not written for C++
// or other languages (except for Go). The old Semtech protocol may be too simple but
// the new code is a brainiac.
// As of half 2018 the code is on hold. Some does work, but acording to the documentation
// it should be simpler to use.
//
// ========================================================================================

#include "defines.h"

#if defined(_TTNROUTER)
#if defined(_UDPROUTER)
#error "Error: Please undefine _UDPROUTER if you like to use _TTNROUTER"
#endif

// The following functions ae defined in this modue:
//
// int readTtn(int Packetsize)
// int sendTtn(IPAddress server, int port, uint8_t *msg, int length)
// bool connectTtn()
// void pullData()
// void sendstat();

// Add gateway code of functions here

void connectTtn()
{
#error "Error: Please define and use _UDPROUTER instead of _TTNROUTER"
}

int readTtn(int Packetsize)
{
#error "Error: Please define and use _UDPROUTER instead of _TTNROUTER"
}

int sendTtn(IPAddress server, int port, uint8_t *msg, int length)
{
#error "Error: Please define and use _UDPROUTER instead of _TTNROUTER"
}

#endif // _TTNROUTER