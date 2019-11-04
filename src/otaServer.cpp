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
// This file contains the ota code for the ESP Single Channel Gateway.

// Provide OTA server funcionality so the 1ch gateway can be updated
// over the air.
// This code uses the ESPhttpServer functions to update the gateway.

#include "defines.h"

#if A_OTA == 1

bool otaActive = false;

void dispOtaStatus(uint8_t percentage);

// Make sure that webserver is running before continuing

// ----------------------------------------------------------------------------
// setupOta
// Function to run in the setup() function to initialise the update function
// ----------------------------------------------------------------------------
void setupOta(char *hostname)
{

	ArduinoOTA.begin();
#if DUSB >= 1
	Serial.println(F("setupOta:: Started"));
#endif
	// Hostname defaults to esp8266-[ChipID]
	ArduinoOTA.setHostname(hostname);

	ArduinoOTA.onStart([]() {
		otaActive = true;
		Radio.Sleep();

		String type;
		// XXX version mismatch of platform.io and ArduinoOtaa
		// see https://github.com/esp8266/Arduino/issues/3020
		//if (ArduinoOTA.getCommand() == U_FLASH)
		type = "sketch";
		//else // U_SPIFFS
		//	type = "filesystem";

		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		Serial.println("Start updating " + type);
	});

	ArduinoOTA.onEnd([]() {
		Serial.println("\nEnd");
	});

	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
#if OLED >= 1
		dispOtaStatus((progress / (total / 100)));
#endif
	});

	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR)
			Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR)
			Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR)
			Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR)
			Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR)
			Serial.println("End Failed");
	});

#if DUSB >= 1
	Serial.println("Ready");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
#endif

	// Only if the Webserver is active also
#if A_SERVER == 2 // Displayed for the moment
	ESPhttpUpdate.rebootOnUpdate(false);

	server.on("/esp", HTTP_POST, [&]() {
		HTTPUpdateResult ret = ESPhttpUpdate.update(server.arg("firmware"), "1.0.0");

		switch (ret)
		{
		case HTTP_UPDATE_FAILED:
			//PREi::sendJSON(500, "Update failed.");
			Serial.println(F("Update failed"));
			break;
		case HTTP_UPDATE_NO_UPDATES:
			//PREi::sendJSON(304, "Update not necessary.");
			Serial.println(F("Update not necessary"));
			break;
		case HTTP_UPDATE_OK:
			//PREi::sendJSON(200, "Update started.");
			Serial.println(F("Update started"));
			ESP.restart();
			break;
		default:
			Serial.println(F("setupOta:: Unknown ret="));
		}
	});
#endif
}

// ----------------------------------------------------------------------------
// updateOtaa()
//
// ----------------------------------------------------------------------------
void updateOtaa()
{

	String response = "";
	printIP((IPAddress)WiFi.localIP(), '.', response);

	ESPhttpUpdate.update(response, 80, "/arduino.bin");
}

#endif