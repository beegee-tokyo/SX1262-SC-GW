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
// This file contains the state machine code enabling to receive
// and transmit packages/messages.
// ========================================================================================
//

#include "defines.h"

#if OLED >= 1

void dispWriteHeader(void);
extern String gwFreq;

// --------------------------------------------------------
// Define the different OLED versions
//
#if OLED == 1
#include "SSD1306.h"
#define OLED_ADDR 0x3C // Default 0x3C for 0.9", for 1.3" it is 0x78
SSD1306 display(OLED_ADDR, OLED_SDA, OLED_SCL); // i2c ADDR & SDA, SCL on wemos
#endif

// This is an 1.3" OLED display which is running on I2C
#if OLED == 2
#include "SH1106.h"
#define OLED_ADDR 0x3C // Default 0x3C for 1.3" SH1106
SH1106 display(OLED_ADDR, OLED_SDA, OLED_SCL); // i2c ADDR & SDA, SCL on wemos
#endif

// --------------------------------------------------------------------
// Initilize the OLED functions.
// This function will init the OLED screenb. Depending on the
// availability of the reset button it will reset the display first.
// --------------------------------------------------------------------
void init_oLED(void)
{
#if defined OLED_RST
	pinMode(OLED_RST, OUTPUT);
	digitalWrite(OLED_RST, LOW); // low to reset OLED
	delay(100);
	digitalWrite(OLED_RST, HIGH); // must be high to turn on OLED
	delay(50);
#else
#endif
	// Initialising the UI will init the display too.
	display.init();
	delay(100);
	display.flipScreenVertically();
	display.setFont(ArialMT_Plain_24);
	display.setTextAlignment(TEXT_ALIGN_LEFT);
	display.drawString(0, 24, "STARTING");
	display.display();
}

// --------------------------------------------------------------------
// Activate the OLED
//
// --------------------------------------------------------------------
void acti_oLED()
{
	// Initialising the UI will init the display too.
	// display.clear();
	dispWriteHeader();

#if OLED == 1
	display.drawString(0, 21, "READY");
	display.drawString(0, 31, "SSID= " + WiFi.SSID());
	display.drawString(0, 41, "WiFi=" + WiFi.localIP().toString());
#elif OLED == 2
	display.drawString(0, 21, "READY");
	display.drawString(0, 31, "SSID= " + WiFi.SSID());
	display.drawString(0, 41, "WiFi=" + WiFi.localIP().toString());
#endif

	display.display();

	delay(4000);
}

// --------------------------------------------------------------------
// Print the OLED address in use
//
// --------------------------------------------------------------------
void addr_oLED()
{
	Serial.print(F("OLED_ADDR=0x"));
	Serial.println(OLED_ADDR, HEX);
}

// --------------------------------------------------------------------
// Write a header
//
// --------------------------------------------------------------------
void dispWriteHeader(void)
{
	display.setFont(ArialMT_Plain_10);

	display.clear();
	display.setTextAlignment(TEXT_ALIGN_LEFT);
	if (!otaActive)
	{
#ifdef ESP32_ARCH
		int strWidth = display.getStringWidth("ESP32 GW " + gwFreq);
		display.drawString(64 - (strWidth / 2), 0, "ESP32 GW " + gwFreq);
#else
		int strWidth = display.getStringWidth("ESP8266 GW " + gwFreq);
		display.drawString(64 - (strWidth / 2), 0, "ESP8266 GW " + gwFreq);
#endif
	}
	else
	{
		int strWidth = display.getStringWidth("OTA update");
		display.drawString(64 - (strWidth / 2), 0, "OTA update");
	}
}

// --------------------------------------------------------------------
// Show OTA status
//
// --------------------------------------------------------------------
void dispOtaStatus(uint8_t percentage)
{
	dispWriteHeader();
	char line[128];
	sprintf(line, "Progress: %d%%", percentage);
	display.drawString(20, 24, line);
	display.display();
}

#endif