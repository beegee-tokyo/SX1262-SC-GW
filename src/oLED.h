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
// This file contains a number of compile-time settings and definitions for OLED support.
//
// ----------------------------------------------------------------------------------------

// OLEDs supported by this program must be I2C.
// This is because we do not want any disturbance in the SPI area
// which is also interfacing the LORA tranceiver.
//
// The following OLEDs are supported:
// 0. No OLED connected
// 1. 0.9" OLED (cheap)
// 2. 1.3" OLED with much better and larger display
// 4. TTGO board

#if OLED >= 1 // If OLED is used

// --------------------------------------------------------
// Define the different PIN's used for SCL/SDA for each arch.
//
#if _PIN_OUT == 1  // HALLARD
#define OLED_SCL 5 // GPIO5 / D1
#define OLED_SDA 4 // GPIO4 / D2

#elif _PIN_OUT == 2 // COMRESULT
#define OLED_SCL 0  // GPIO0 / D3
#define OLED_SDA 2  // GPIO2 / D4

#elif _PIN_OUT == 4 // TTGO (onboard version used, also for DIY)
#define OLED_SCL 15 // GPIO15 /
#define OLED_SDA 4  // GPIO4 /
#define OLED_RST 16 // Reset pin (Some OLED displays do not have it)

#elif _PIN_OUT == 6 // COMRESULT
#define OLED_SCL 32 //22                // GPIO0 / D3
#define OLED_SDA 13 // 21                // GPIO2 / D4

#endif

#ifndef OLED_H
#define OLED_H
// --------------------------------------------------------
// Define the different OLED versions
//
#if OLED == 1
#include "SSD1306.h"
#define OLED_ADDR 0x3C							// Default 0x3C for 0.9", for 1.3" it is 0x78
extern SSD1306 display;
#endif

// This is an 1.3" OLED display which is running on I2C
#if OLED == 2
#include "SH1106.h"
#define OLED_ADDR 0x3C						   // Default 0x3C for 1.3" SH1106
extern SH1106 display;
#endif
#endif // OLED_H
#endif //OLED>=1