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
// This file contains a number of compile-time settings and declarations that are
// specific to the LoRa rfm95, sx1276, sx1272 radio of the gateway.
//
//
// ------------------------------------------------------------------------------------

#ifndef SENSOR_H
#define SENSOR_H

#if _TRUSTED_NODES >= 1
struct nodex
{
	uint32_t id; // This is the LoRa ID (coded in 4 bytes uint32_t
	char nm[32]; // Name of the node
};

// Add all your named and trusted nodes to this list
extern struct nodex nodes[2];
#endif //_TRUSTED_NODES

#endif // SENSOR_H