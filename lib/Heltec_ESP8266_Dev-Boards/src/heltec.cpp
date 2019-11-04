// Copyright (c) Heltec Automation. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "heltec.h"


Heltec_ESP8266::Heltec_ESP8266(){

#if defined( WIFI_Kit_8 )
     display = new SSD1306Wire(0x3c, SDA, SCL, OLED_RST, GEOMETRY_128_32);
//#elif defined( Wireless_Stick )
//	  display = new SSD1306Wire(0x3c, SDA_OLED, SCL_OLED, RST_OLED, GEOMETRY_64_32);
#endif
}

Heltec_ESP8266::~Heltec_ESP8266(){
	delete display;
}

void Heltec_ESP8266::begin(bool DisplayEnable, bool SerialEnable) {

//#if defined( WIFI_LoRa_32_V2 ) || defined( Wireless_Stick )

//	VextON();
//#endif

	// UART
	if (SerialEnable) {
		Serial.begin(115200);
		Serial.flush();
		delay(50);
		Serial.print("Serial initial done\r\n");
	}

	// OLED
	if (DisplayEnable){
		display->init();
		display->flipScreenVertically();
		display->setFont(ArialMT_Plain_10);
		display->drawString(0, 0, "OLED initial done!");
		display->display();

		if (SerialEnable){
			Serial.print("you can see OLED printed OLED initial done!\r\n");
		}
	}

	// LoRa INIT
//	if (LoRaEnable)
//	{
//#if defined(WIFI_Kit_32)
//		if(SerialEnable && WIFI_Kit_32){
//			Serial.print("The WiFi Kit 32 not have LoRa function, LoRa option must be FALSE!!!\r\n");
//		}
//#endif
//
//
//#if defined( WIFI_LoRa_32 ) || defined( WIFI_LoRa_32_V2 ) || defined( Wireless_Stick )
//		//LoRaClass LoRa;
//
//		SPI.begin(SCK,MISO,MOSI,SS);
//		LoRa.setPins(SS,RST_LoRa,DIO0);
//		if (!LoRa.begin(BAND,PABOOST))
//		{
//			if (SerialEnable){
//				Serial.print("Starting LoRa failed!\r\n");
//			}
//			if(DisplayEnable){
//				display->clear();
//				display->drawString(0, 0, "Starting LoRa failed!");
//				display->display();
//				delay(300);
//			}
//			while (1);
//		}
//		if (SerialEnable){
//			Serial.print("LoRa Initial success!\r\n");
//		}
//		if(DisplayEnable){
//			display->clear();
//			display->drawString(0, 0, "LoRa Initial success!");
//			display->display();
//			delay(300);
//		}
//#endif
//	}
}

//void Heltec_ESP32::VextON(void)
//{
//	pinMode(Vext,OUTPUT);
//	digitalWrite(Vext, LOW);
//}
//
//void Heltec_ESP32::VextOFF(void) //Vext default OFF
//{
//	pinMode(Vext,OUTPUT);
//	digitalWrite(Vext, HIGH);
//}

Heltec_ESP8266 Heltec;

