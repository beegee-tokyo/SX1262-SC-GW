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
// This file contains the LoRa modem specific code enabling to receive
// and transmit packages/messages.
// ========================================================================================
//

#include "defines.h"
#include "loraModem.h"

//
// ----------------------------------------------------------------------------------------
// Variable definitions
//
//
// ----------------------------------------------------------------------------------------

#ifdef _LOCALSERVER
// Sometimes we want to decode the sensor completely as we do in the TTN server
// This means that for all nodes we want to view the data of, we need to provide
// the AppsSKey and the NwkSKey
// Dev ID
// Dev name clear text
// NwkSKey
// AppsSKey
// Definition of all nodes that we want to decode locally on the gateway.
//
struct codex decodes[KNOWN_NODES] = {
	{0x260211CE,
	 "esp32r", // F=0
	 {0x71, 0x6E, 0xB7, 0x65, 0x90, 0x60, 0x8F, 0xEB, 0x50, 0x24, 0xF9, 0x98, 0x31, 0x29, 0x82, 0xC8},
	 {0x0B, 0x94, 0x60, 0x10, 0x8C, 0x41, 0x3C, 0x7C, 0xF8, 0xC7, 0x99, 0x14, 0x27, 0x90, 0x96, 0x92}},
	{0x260214FE,
	 "esp8266", // F=0
	 {0x66, 0xCE, 0xD5, 0xCD, 0x2A, 0x30, 0xC7, 0x1F, 0x34, 0x07, 0xB2, 0x58, 0x7D, 0x09, 0x64, 0x66},
	 {0xAF, 0x6C, 0x9A, 0x5B, 0x08, 0x64, 0x5B, 0x1E, 0x9B, 0xAB, 0x92, 0x66, 0x3E, 0xA5, 0x43, 0x42}}};
#endif

// Our code should correct the server Tramission delay settings
long txDelay = 0x00; // tx delay time on top of server TMST

#ifdef EU863_870
// This the the EU863_870 format as used in most of Europe
// It is also the default for most of the single channel gateway work.
// For each frequency SF7-SF12 are used.
vector freqs[] =
	{
		{868100000, 125, 7, 12, 868100000, 125, 7, 12}, // Channel 0, 868.1 MHz/125 primary
		{868300000, 125, 7, 12, 868300000, 125, 7, 12}, // Channel 1, 868.3 MHz/125 mandatory and (SF7BW250)
		{868500000, 125, 7, 12, 868500000, 125, 7, 12}, // Channel 2, 868.5 MHz/125 mandatory
		{867100000, 125, 7, 12, 867100000, 125, 7, 12}, // Channel 3, 867.1 MHz/125 Optional
		{867300000, 125, 7, 12, 867300000, 125, 7, 12}, // Channel 4, 867.3 MHz/125 Optional
		{867500000, 125, 7, 12, 867500000, 125, 7, 12}, // Channel 5, 867.5 MHz/125 Optional
		{867700000, 125, 7, 12, 867700000, 125, 7, 12}, // Channel 6, 867.7 MHz/125 Optional
		{867900000, 125, 7, 12, 867900000, 125, 7, 12}, // Channel 7, 867.9 MHz/125 Optional
		{868800000, 125, 7, 12, 868800000, 125, 7, 12}, // Channel 8, 868.9 MHz/125 FSK Only
		{0, 0, 0, 0, 869525000, 125, 9, 9}				// Channel 9, 869.5 MHz/125 for RX2 responses SF9(10%)
														// TTN defines an additional channel at 869.525 MHz using SF9 for class B. Not used
};

#elif defined(EU433)
// The following 3 frequencies should be defined/used in an EU433
// environment. The plan is not defined for TTN yet so we use this one.
vector freqs[] = {
	{433175000, 125, 7, 12, 433175000, 125, 7, 12}, // Channel 0, 433.175 MHz/125 primary
	{433375000, 125, 7, 12, 433375000, 125, 7, 12}, // Channel 1, 433.375 MHz primary
	{433575000, 125, 7, 12, 433575000, 125, 7, 12}, // Channel 2, 433.575 MHz primary
	{433775000, 125, 7, 12, 433775000, 125, 7, 12}, // Channel 3, 433.775 MHz primary
	{433975000, 125, 7, 12, 433975000, 125, 7, 12}, // Channel 4, 433.975 MHz primary
	{434175000, 125, 7, 12, 434175000, 125, 7, 12}, // Channel 5, 434.175 MHz primary
	{434375000, 125, 7, 12, 434375000, 125, 7, 12}, // Channel 6, 434.375 MHz primary
	{434575000, 125, 7, 12, 434575000, 125, 7, 12}, // Channel 7, 434.575 MHz primary
	{434775000, 125, 7, 12, 434775000, 125, 7, 12}  // Channel 8, 434.775 MHz primary
};

#elif defined(US902_928)
// The frequency plan for USA is a difficult one. As yout can see, the uplink protocol uses
// SF7-SF10 and BW125 whereas the downlink protocol uses SF7-SF12 and BW500.
// Also the number of chanels is not equal.
vector freqs[] = {
	// Uplink
	{902300000, 125, 7, 10, 902300000, 125, 7, 10}, // Up Ch 0, SF7BW125 to SF10BW125 primary
	{902500000, 125, 7, 10, 923900000, 500, 7, 12}, // Up Ch 1, SF7BW125 to SF10BW125
	{902700000, 125, 7, 10, 924500000, 500, 7, 12}, // Up Ch 2, SF7BW125 to SF10BW125, Dwn SF7-SF12 924,5 BW500
	{902900000, 125, 7, 10, 925100000, 500, 7, 12}, // Up Ch 3, SF7BW125 to SF10BW125, Dwn SF7-SF12 925,1 BW500
	{903100000, 125, 7, 10, 925700000, 500, 7, 12}, // Up Ch 4, SF7BW125 to SF10BW125, Dwn SF7-SF12 925,1
	{903300000, 125, 7, 10, 926300000, 500, 7, 12}, // Up Ch 5, SF7BW125 to SF10BW125, Dwn SF7-SF12
	{903500000, 125, 7, 10, 926900000, 500, 7, 12}, // Up Ch 6, SF7BW125 to SF10BW125, Dwn SF7-SF12
	{903700000, 125, 7, 10, 927500000, 500, 7, 12}, // Up Ch 7, SF7BW125 to SF10BW125, Dwn SF7-SF12
	{903900000, 500, 8, 8, 0, 0, 0, 00},			// Up Ch 8, SF8BW5000, no Dwn 0 																						// SFxxxBW500
													//	{ 903900000, 125, 7, 10, 923300000, 500, 7, 12},			// Up Ch 0, SF7BW125 to SF10BW125 primary
													//	{ 904100000, 125, 7, 10, 923900000, 500, 7, 12},			// Up Ch 1, SF7BW125 to SF10BW125
													//	{ 904300000, 125, 7, 10, 924500000, 500, 7, 12},			// Up Ch 2, SF7BW125 to SF10BW125, Dwn SF7-SF12 924,5 BW500
													//	{ 904500000, 125, 7, 10, 925100000, 500, 7, 12},			// Up Ch 3, SF7BW125 to SF10BW125, Dwn SF7-SF12 925,1 BW500
													//	{ 904700000, 125, 7, 10, 925700000, 500, 7, 12},			// Up Ch 3, SF7BW125 to SF10BW125, Dwn SF7-SF12 925,1
													//	{ 904900000, 125, 7, 10, 926300000, 500, 7, 12},			// Up Ch 4, SF7BW125 to SF10BW125, Dwn SF7-SF12
													//	{ 905100000, 125, 7, 10, 926900000, 500, 7, 12},			// Up Ch 5, SF7BW125 to SF10BW125, Dwn SF7-SF12
													//	{ 905300000, 125, 7, 10, 927500000, 500, 7, 12},			// Up Ch 6, SF7BW125 to SF10BW125, Dwn SF7-SF12
													//	{ 904600000, 500, 8,  8, 0        , 0,   0, 00},			// Up Ch 7, SF8BW5000, no Dwn 0 																						// SFxxxBW500
};

#elif defined(AU925_928)
// Australian plan or TTN/Lora frequencies
vector freqs[] = {
	{916800000, 125, 7, 10, 916800000, 125, 7, 12}, // Channel 0, 916.8 MHz primary
	{917000000, 125, 7, 10, 917000000, 125, 7, 12}, // Channel 1, 917.0 MHz mandatory
	{917200000, 125, 7, 10, 917200000, 125, 7, 12}, // Channel 2, 917.2 MHz mandatory
	{917400000, 125, 7, 10, 917400000, 125, 7, 12}, // Channel 3, 917.4 MHz Optional
	{917600000, 125, 7, 10, 917600000, 125, 7, 12}, // Channel 4, 917.6 MHz Optional
	{917800000, 125, 7, 10, 917800000, 125, 7, 12}, // Channel 5, 917.8 MHz Optional
	{918000000, 125, 7, 10, 918000000, 125, 7, 12}, // Channel 6, 918.0 MHz Optional
	{918200000, 125, 7, 10, 918200000, 125, 7, 12}, // Channel 7, 918.2 MHz Optional
	{917500000, 500, 8, 8, 0, 0, 0, 0}				// Channel 8, 917.5 SF8BW500 MHz Optional Uplink
};

#elif defined(CN470_510)
// China plan for TTN frequencies
vector freqs[] = {
	{486300000, 125, 7, 12, 486300000, 125, 7, 12}, // 486.3 - SF7BW125 to SF12BW125
	{486500000, 125, 7, 12, 486500000, 125, 7, 12}, // 486.5 - SF7BW125 to SF12BW125
	{486700000, 125, 7, 12, 486700000, 125, 7, 12}, // 486.7 - SF7BW125 to SF12BW125
	{486900000, 125, 7, 12, 486900000, 125, 7, 12}, // 486.9 - SF7BW125 to SF12BW125
	{487100000, 125, 7, 12, 487100000, 125, 7, 12}, // 487.1 - SF7BW125 to SF12BW125
	{487300000, 125, 7, 12, 487300000, 125, 7, 12}, // 487.3 - SF7BW125 to SF12BW125
	{487500000, 125, 7, 12, 487500000, 125, 7, 12}, // 487.5 - SF7BW125 to SF12BW125
	{487700000, 125, 7, 12, 487700000, 125, 7, 12}  // 487.7 - SF7BW125 to SF12BW125
};

#else
int freqs[] = {
// Print an Error, Not supported
#error "Sorry, but your frequency plan is not supported"
};
#endif

volatile state_t _state = S_INIT;
volatile uint8_t _event = 0;

// rssi is measured at specific moments and reported on others
// so we need to store the current value we like to work with
uint8_t _rssi;

/// \todo CAD and HOP are not supported by SX1262 (yet)
bool _cad = (bool)_CAD;  // Set to true for Channel Activity Detection, only when dio 1 connected
bool _hop = (bool)false; // experimental; frequency hopping. Only use when dio2 connected

unsigned long nowTime = 0;
unsigned long msgTime = 0;
unsigned long hopTime = 0;
unsigned long detTime = 0;

#if _PIN_OUT == 1
// ----------------------------------------------------------------------------
// Definition of the GPIO pins used by the Gateway for Hallard type boards
//
struct pins
{
	uint8_t dio0 = 15; // GPIO15 / D8. For the Hallard board shared between DIO0/DIO1/DIO2
	uint8_t dio1 = 15; // GPIO15 / D8. Used for CAD, may or not be shared with DIO0
	uint8_t dio2 = 15; // GPIO15 / D8. Used for frequency hopping, don't care
	uint8_t ss = 16;   // GPIO16 / D0. Select pin connected to GPIO16 / D0
	uint8_t rst = 0;   // GPIO 0 / D3. Reset pin not used
					   // MISO 12 / D6
					   // MOSI 13 / D7
					   // CLK  14 / D5
} pins;

#elif _PIN_OUT == 2
// ----------------------------------------------------------------------------
// For ComResult gateway PCB use the following settings
struct pins
{
	uint8_t dio0 = 5; // GPIO5 / D1. Dio0 used for one frequency and one SF
	uint8_t dio1 = 4; // GPIO4 / D2. Used for CAD, may or not be shared with DIO0
	uint8_t dio2 = 0; // GPIO0 / D3. Used for frequency hopping, don't care
	uint8_t ss = 15;  // GPIO15 / D8. Select pin connected to GPIO15
	uint8_t rst = 0;  // GPIO0  / D3. Reset pin not used
} pins;

#elif _PIN_OUT == 3
// ----------------------------------------------------------------------------
// For ESP32/Wemos based board
// SCK  == GPIO5/ PIN5
// SS   == GPIO18/PIN18
// MISO == GPIO19/ PIN19
// MOSI == GPIO27/ PIN27
// RST  == GPIO14/ PIN14
struct pins
{
	uint8_t dio0 = 26; // GPIO26 / Dio0 used for one frequency and one SF
	uint8_t dio1 = 26; // GPIO26 / Used for CAD, may or not be shared with DIO0
	uint8_t dio2 = 26; // GPI2O6 / Used for frequency hopping, don't care
	uint8_t ss = 18;   // GPIO18 / Dx. Select pin connected to GPIO18
	uint8_t rst = 14;  // GPIO0  / D3. Reset pin not used
} pins;

#elif _PIN_OUT == 4
// ----------------------------------------------------------------------------
// For ESP32/TTGO based board.
// SCK  == GPIO5/ PIN5
// SS   == GPIO18/PIN18 CS
// MISO == GPIO19/ PIN19
// MOSI == GPIO27/ PIN27
// RST  == GPIO14/ PIN14
struct pins
{
	uint8_t dio0 = 26; // GPIO26 / Dio0 used for one frequency and one SF
	uint8_t dio1 = 33; // GPIO26 / Used for CAD, may or not be shared with DIO0
	uint8_t dio2 = 32; // GPIO26 / Used for frequency hopping, don't care
	uint8_t ss = 18;   // GPIO18 / Dx. Select pin connected to GPIO18
	uint8_t rst = 14;  // GPIO0  / D3. Reset pin not used
} pins;
#define SCK 5
#define MISO 19
#define MOSI 27
#define RST 14
#define SS 18
#define GPS_RX 15
#define GPS_TX 12

#elif _PIN_OUT == 5
// ----------------------------------------------------------------------------
// For ESP32/TTGO based board for EU32 with 0.9" OLED
// NOTE: This board shoudl be same as general type TTGO (nr 4)
// but for the moment we include this as a separate item
//
// SCK  == GPIO5/ PIN5
// SS   == GPIO18/PIN18 CS
// MISO == GPIO19/ PIN19
// MOSI == GPIO27/ PIN27
// RST  == GPIO14/ PIN14
struct pins
{
	uint8_t dio0 = 26; // GPIO26 / Dio0 used for one frequency and one SF
	uint8_t dio1 = 33; // GPIO26 / Used for CAD, may or not be shared with DIO0
	uint8_t dio2 = 32; // GPIO26 / Used for frequency hopping, don't care
	uint8_t ss = 18;   // GPIO18 / Dx. Select pin connected to GPIO18
	uint8_t rst = 14;  // GPIO0 / D3. Reset pin not used
} pins;
#define SCK 5   // Check
#define MISO 19 // Check
#define MOSI 27 // Check
#define RST 14  // Check
#define SS 18

#else
// ----------------------------------------------------------------------------
// Use your own pin definitions, and comment #error line below
// MISO 12 / D6
// MOSI 13 / D7
// CLK  14 / D5
// SS   16 / D0
//#error "Pin Definitions _PIN_OUT must be 1(HALLARD) or 2 (COMRESULT)"
struct pins
{
	uint8_t dio0 = 26;
	uint8_t dio1 = 33;
	uint8_t dio2 = 32;
	uint8_t ss = 16;
	uint8_t rst = 27; // Reset not used
} pins;
#define SCK 14
#define MISO 12
#define MOSI 13
#define SS 16
#define DIO0 26
#endif

struct stat_t stat_t;

#if STATISTICS >= 1
// statc_c contains the statistic that are gateway related and not per
// message. Example: Number of messages received on SF7 or number of (re) boots
// So where statr contains the statistics gathered per packet the statc_c
// contains general statistics of the node
struct stat_c statc;

// History of received uplink messages from nodes
struct stat_t statr[MAX_STAT];

#else // STATISTICS==0
struct stat_t statr[1]; // Always have at least one element to store in
#endif

// Define the payload structure used to separate interrupt ans SPI
// processing from the loop() part
uint8_t payLoad[128]; // Payload i
struct LoraBuffer LoraDown;
// Up buffer (from Lora sensor to UDP)
struct LoraUp LoraUp;

//
// ========================================================================================
// SPI AND INTERRUPTS
// The RFM96/SX1276 communicates with the ESP8266 by means of interrupts
// and SPI interface. The SPI interface is bidirectional and allows both
// parties to simultaneous write and read to registers.
// Major drawback is that access is not protected for interrupt and non-
// interrupt access. This means that when a program in loop() and a program
// in interrupt do access the readRegister and writeRegister() function
// at the same time that probably an error will occur.
// Therefore it is best to either not use interrupts AT all (like LMIC)
// or only use these functions in interrupts and to further processing
// in the main loop() program.
//
// ========================================================================================

// ----------------------------------------------------------------------------------------
// Mutex definitions
//
// ----------------------------------------------------------------------------------------
#if MUTEX == 1
void CreateMutux(int *mutex)
{
	*mutex = 1;
}

#define LIB_MUTEX 1
#if LIB_MUTEX == 1
bool GetMutex(int *mutex)
{
	//noInterrupts();
	if (*mutex == 1)
	{
		*mutex = 0;
		//interrupts();
		return (true);
	}
	//interrupts();
	return (false);
}
#else
bool GetMutex(int *mutex)
{

	int iOld = 1, iNew = 0;

	asm volatile(
		"rsil a15, 1\n"	// read and set interrupt level to 1
		"l32i %0, %1, 0\n" // load value of mutex
		"bne %0, %2, 1f\n" // compare with iOld, branch if not equal
		"s32i %3, %1, 0\n" // store iNew in mutex
		"1:\n"			   // branch target
		"wsr.ps a15\n"	 // restore program state
		"rsync\n"
		: "=&r"(iOld)
		: "r"(mutex), "r"(iOld), "r"(iNew)
		: "a15", "memory");
	return (bool)iOld;
}
#endif

void ReleaseMutex(int *mutex)
{
	*mutex = 1;
}

#endif //MUTEX==1

#ifdef CFG_sx1262_radio
//********************************************************************
// This stuff is for SX1262 only.
// Goal is to move this into a separate file and use #defines to
// compile either the SX127x codes or the SX1262 codes
//********************************************************************

// ESP32 - SX126x pin configuration
int PIN_LORA_RESET = 4;  // LORA RESET
int PIN_LORA_NSS = 5;	// LORA SPI CS
int PIN_LORA_SCLK = 18;  // LORA SPI CLK
int PIN_LORA_MISO = 19;  // LORA SPI MISO
int PIN_LORA_DIO_1 = 21; // LORA DIO_1
int PIN_LORA_BUSY = 22;  // LORA SPI BUSY
int PIN_LORA_MOSI = 23;  // LORA SPI MOSI
int RADIO_TXEN = 26;	 // LORA ANTENNA TX ENABLE
int RADIO_RXEN = 27;	 // LORA ANTENNA RX ENABLE

// Function declarations
void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);
void OnCadDone(bool cadResult);

// Define LoRa parameters
#if defined(EU863_870)
#define RF_FREQUENCY 868100000 // Hz
String gwFreq = "868.1";
#elif defined(US902_928)
#define RF_FREQUENCY 902300000 // Hz
String gwFreq = "902.3";
#elif defined(U925_928)
#define RF_FREQUENCY 915200000 // Hz
String gwFreq = "915.2";
#elif defined(CN470_510)
#define RF_FREQUENCY 470300000 // Hz
String gwFreq = "470.3";
#else
#error "Please define a frequency region in ESP-sc-gway.ino."
String gwFreq = "NONE";
#endif
#define TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH 0		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000

static RadioEvents_t RadioEvents;

hw_config hwConfig;

void initLoraModem()
{
	// Define the HW configuration between MCU and SX126x
	hwConfig.CHIP_TYPE = SX1262_CHIP;		  // eByte E22 module with an SX1262
	hwConfig.PIN_LORA_RESET = PIN_LORA_RESET; // LORA RESET
	hwConfig.PIN_LORA_NSS = PIN_LORA_NSS;	 // LORA SPI CS
	hwConfig.PIN_LORA_SCLK = PIN_LORA_SCLK;   // LORA SPI CLK
	hwConfig.PIN_LORA_MISO = PIN_LORA_MISO;   // LORA SPI MISO
	hwConfig.PIN_LORA_DIO_1 = PIN_LORA_DIO_1; // LORA DIO_1
	hwConfig.PIN_LORA_BUSY = PIN_LORA_BUSY;   // LORA SPI BUSY
	hwConfig.PIN_LORA_MOSI = PIN_LORA_MOSI;   // LORA SPI MOSI
	hwConfig.RADIO_TXEN = RADIO_TXEN;		  // LORA ANTENNA TX ENABLE
	hwConfig.RADIO_RXEN = RADIO_RXEN;		  // LORA ANTENNA RX ENABLE
	hwConfig.USE_DIO2_ANT_SWITCH = false;	 // Example uses an eByte E22 module which uses RXEN and TXEN pins as antenna control
	hwConfig.USE_DIO3_TCXO = true;			  // Example uses an eByte E22 module which uses DIO3 to control oscillator voltage
	hwConfig.USE_DIO3_ANT_SWITCH = false;	 // Only Insight ISP4520 module uses DIO3 as antenna control

	// Initialize the LoRa chip
	lora_hardware_init(hwConfig);

	// Initialize the Radio callbacks
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;
	RadioEvents.CadDone = OnCadDone;

	// Initialize the Radio
	Radio.Init(&RadioEvents);

	// Set Radio channel
	Radio.SetChannel(RF_FREQUENCY);

	// Set Radio TX configuration
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
					  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
					  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
					  true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

	// Set Radio RX configuration
	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
					  LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
					  LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
					  0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

	Radio.SetPublicNetwork(true);

	// Start LoRa
	Serial.println("Starting Radio.Rx");
	Radio.Rx(0);
}

/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void)
{
#if DUSB >= 1
	if ((debug >= 0) && (pdebug & P_TX))
	{
		Serial.println(F("DOWN: Sending finished"));
	}
#endif
	statc.msg_down++;
#if STATISTICS >= 2
	switch (statr[0].ch)
	{
	case 0:
		statc.msg_down_0++;
		break;
	case 1:
		statc.msg_down_1++;
		break;
	case 2:
		statc.msg_down_2++;
		break;
	}
#endif
	// Back to listening
	Radio.Rx(0);
}

/**@brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	// There should not be an error in the message
	LoraUp.payLoad[0] = 0x00; // Empty the message
	LoraUp.payLength = size;
	memcpy(LoraUp.payLoad, payload, size);
	LoraUp.snr = snr;
	LoraUp.prssi = rssi;
	LoraUp.rssicorr = 0;
	LoraUp.sf = 7;

	// If read was successful, read the package from the LoRa bus
	//
	if (receivePacket() <= 0)
	{ // read is not successful
#if DUSB >= 1
		if ((debug >= 0) && (pdebug & P_RX))
		{
			Serial.println(F("UP: Error receivePacket"));
		}
#endif
	}

	statc.msg_ttl++; // Receive statistics counter

	statc.msg_ok++; // Receive OK statistics counter
#if STATISTICS >= 2
	switch (statr[0].ch)
	{
	case 0:
		statc.msg_ok_0++;
		break;
	case 1:
		statc.msg_ok_1++;
		break;
	case 2:
		statc.msg_ok_2++;
		break;
	}
#endif
	/// \todo Add Downlink here

	if (_state == S_TX)
	{
#if DUSB >= 1
		if ((debug >= 0) && (pdebug & P_TX))
		{
			Serial.println("Package for a device found");

			for (int idx = 0; idx < LoraDown.payLength; idx++)
			{
				Serial.printf("%0X ", LoraDown.payLoad[idx]);
			}
			Serial.printf("\nSF: %d Pwr: %d Freq: %d\n", LoraDown.sfTx, LoraDown.powe, LoraDown.fff);
		}
#endif

		// Set state to transmit
		_state = S_TXDONE;
		/// \todo find a better way than a delay
		delay(1000);
		Radio.Send(LoraDown.payLoad, LoraDown.payLength);
	}
	// Back to listening
	Radio.Rx(0);
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
#if DUSB >= 1
	if ((debug >= 0) && (pdebug & P_TX))
	{
		Serial.println(F("DOWN: Error sending package"));
	}
#endif
	// Back to listening
	Radio.Rx(0);
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout(void)
{
#if DUSB >= 1
	if ((debug >= 0) && (pdebug & P_RX))
	{
		Serial.println(F("UP: Timeout receiving LoRa"));
	}
#endif
	// Back to listening
	Radio.Rx(0);
}

/**@brief Function to be executed on Radio Rx Error event
 */
void OnRxError(void)
{
#if DUSB >= 1
	if ((debug >= 0) && (pdebug & P_RX))
	{
		Serial.println(F("UP: Error receiving LoRa"));
	}
#endif
	// Back to listening
	Radio.Rx(0);
}

/**@brief Function to be executed on Radio Rx Error event
 */
void OnCadDone(bool cadResult)
{
	// Back to listening
	Radio.Rx(0);
}

//********************************************************************
// Unused functions when LoRa module is SX1262
//********************************************************************
uint8_t readRegister(uint8_t addr)
{
	return 0;
}

void writeRegister(uint8_t addr, uint8_t value)
{
}

void writeBuffer(uint8_t addr, uint8_t *buf, uint8_t len)
{
}

void setRate(uint8_t sf, uint8_t crc)
{
}

void setFreq(uint32_t freq)
{
}

void setPow(uint8_t powe)
{
}

void opmode(uint8_t mode)
{
}

void hop()
{
}

uint8_t receivePkt(uint8_t *payload)
{
	return 0;
}

bool sendPkt(uint8_t *payLoad, uint8_t payLength)
{
	return false;
}

void txLoraModem(uint8_t *payLoad, uint8_t payLength, uint32_t tmst, uint8_t sfTx,
				 uint8_t powe, uint32_t freq, uint8_t crc, uint8_t iiq)
{
}

void rxLoraModem()
{
}

void cadScanner()
{
}

void startReceiver()
{
}

#else
//********************************************************************
// This stuff is for SX127X only.
// Goal is to move this into a separate file and use #defines to
// compile either the SX127x codes or the SX1262 codes
//********************************************************************

// ----------------------------------------------------------------------------------------
// Read one byte value, par addr is address
// Returns the value of register(addr)
//
// The SS (Chip select) pin is used to make sure the RFM95 is selected
// The variable is for obvious reasons valid for read and write traffic at the
// same time. Since both read and write mean that we write to the SPI interface.
// Parameters:
//	Address: SPI address to read from. Type uint8_t
// Return:
//	Value read from address
// ----------------------------------------------------------------------------------------

// define the SPI settings for reading messages
SPISettings readSettings(SPISPEED, MSBFIRST, SPI_MODE0);

uint8_t readRegister(uint8_t addr)
{

	SPI.beginTransaction(readSettings);
	digitalWrite(pins.ss, LOW); // Select Receiver
	SPI.transfer(addr & 0x7F);
	uint8_t res = (uint8_t)SPI.transfer(0x00);
	digitalWrite(pins.ss, HIGH); // Unselect Receiver
	SPI.endTransaction();
	return ((uint8_t)res);
}

// ----------------------------------------------------------------------------
// Write value to a register with address addr.
// Function writes one byte at a time.
// Parameters:
//	addr: SPI address to write to
//	value: The value to write to address
// Returns:
//	<void>
// ----------------------------------------------------------------------------

// define the settings for SPI writing
SPISettings writeSettings(SPISPEED, MSBFIRST, SPI_MODE0);

void writeRegister(uint8_t addr, uint8_t value)
{
	SPI.beginTransaction(writeSettings);
	digitalWrite(pins.ss, LOW); // Select Receiver

	SPI.transfer((addr | 0x80) & 0xFF);
	SPI.transfer(value & 0xFF);

	digitalWrite(pins.ss, HIGH); // Unselect Receiver

	SPI.endTransaction();
}

// ----------------------------------------------------------------------------
// Write a buffer to a register with address addr.
// Function writes one byte at a time.
// Parameters:
//	addr: SPI address to write to
//	value: The value to write to address
// Returns:
//	<void>
// ----------------------------------------------------------------------------

void writeBuffer(uint8_t addr, uint8_t *buf, uint8_t len)
{
	//noInterrupts();							// XXX

	SPI.beginTransaction(writeSettings);
	digitalWrite(pins.ss, LOW); // Select Receiver

	SPI.transfer((addr | 0x80) & 0xFF); // write buffer address
	for (uint8_t i = 0; i < len; i++)
	{
		SPI.transfer(buf[i] & 0xFF);
	}
	digitalWrite(pins.ss, HIGH); // Unselect Receiver

	SPI.endTransaction();
}

// ----------------------------------------------------------------------------
//  setRate is setting rate and spreading factor and CRC etc. for transmission
//  for example
//		Modem Config 1 (MC1) == 0x72 for sx1276
//		Modem Config 2 (MC2) == (CRC_ON) | (sf<<4)
//		Modem Config 3 (MC3) == 0x04 | (optional SF11/12 LOW DATA OPTIMIZE 0x08)
//		sf == SF7 default 0x07, (SF7<<4) == SX72_MC2_SF7
//		bw == 125 == 0x70
//		cr == CR4/5 == 0x02
//		CRC_ON == 0x04
//
//	sf is SF7 to SF12
//	crc is 0x00 (off) or
// ----------------------------------------------------------------------------

void setRate(uint8_t sf, uint8_t crc)
{
	uint8_t mc1 = 0, mc2 = 0, mc3 = 0;
#if DUSB >= 2
	if ((sf < SF7) || (sf > SF12))
	{
		if ((debug >= 1) && (pdebug & P_RADIO))
		{
			Serial.print(F("setRate:: SF="));
			Serial.println(sf);
		}
		return;
	}
#endif
	// Set rate based on Spreading Factor etc
	if (sx1272)
	{
		mc1 = 0x0A; // SX1276_MC1_BW_250 0x80 | SX1276_MC1_CR_4_5 0x02
		mc2 = ((sf << 4) | crc) % 0xFF;
		// SX1276_MC1_BW_250 0x80 | SX1276_MC1_CR_4_5 0x02 | SX1276_MC1_IMPLICIT_HEADER_MODE_ON 0x01
		if (sf == SF11 || sf == SF12)
		{
			mc1 = 0x0B;
		}
	}

	// For sx1276 chips is the CRC ON is
	else
	{
		uint8_t bw = 0; // bw setting is in freqs[ifreq].dwnBw
		uint8_t cr = 0; // cr settings dependent on SF setting
		//switch (

		if (sf == SF8)
		{
			mc1 = 0x78; // SX1276_MC1_BW_125==0x70 | SX1276_MC1_CR_4_8==0x08
		}
		else
		{
			mc1 = 0x72; // SX1276_MC1_BW_125==0x70 | SX1276_MC1_CR_4_5==0x02
		}
		mc2 = ((sf << 4) | crc) & 0xFF; // crc is 0x00 or 0x04==SX1276_MC2_RX_PAYLOAD_CRCON
		mc3 = 0x04;						// 0x04; SX1276_MC3_AGCAUTO
		if (sf == SF11 || sf == SF12)
		{
			mc3 |= 0x08;
		} // 0x08 | 0x04
	}

	// Implicit Header (IH), for class b beacons (&& SF6)
	//if (getIh(LMIC.rps)) {
	//   mc1 |= SX1276_MC1_IMPLICIT_HEADER_MODE_ON;
	//    writeRegister(REG_PAYLOAD_LENGTH, getIh(LMIC.rps)); // required length
	//}

	writeRegister(REG_MODEM_CONFIG1, (uint8_t)mc1);
	writeRegister(REG_MODEM_CONFIG2, (uint8_t)mc2);
	writeRegister(REG_MODEM_CONFIG3, (uint8_t)mc3);

	// Symbol timeout settings
	if (sf == SF10 || sf == SF11 || sf == SF12)
	{
		writeRegister(REG_SYMB_TIMEOUT_LSB, (uint8_t)0x05);
	}
	else
	{
		writeRegister(REG_SYMB_TIMEOUT_LSB, (uint8_t)0x08);
	}
	return;
}

// ----------------------------------------------------------------------------
// Set the frequency for our gateway
// The function has no parameter other than the freq setting used in init.
// Since we are using a 1ch gateway this value is set fixed.
// ----------------------------------------------------------------------------

void setFreq(uint32_t freq)
{
	// set frequency
	uint64_t frf = ((uint64_t)freq << 19) / 32000000;
	writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
	writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
	writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));

	return;
}

// ----------------------------------------------------------------------------
//	Set Power for our gateway
// ----------------------------------------------------------------------------
void setPow(uint8_t powe)
{
	if (powe >= 16)
		powe = 15;
	//if (powe >= 15) powe = 14;
	else if (powe < 2)
		powe = 2;

	ASSERT((powe >= 2) && (powe <= 15));

	uint8_t pac = (0x80 | (powe & 0xF)) & 0xFF;
	writeRegister(REG_PAC, (uint8_t)pac); // set 0x09 to pac

	// XXX Power settings for CFG_sx1272 are different

	return;
}

// ----------------------------------------------------------------------------
// Used to set the radio to LoRa mode (transmitter)
// Please note that this mode can only be set in SLEEP mode and not in Standby.
// Also there should be not need to re-init this mode is set in the setup()
// function.
// For high freqs (>860 MHz) we need to & with 0x08 otherwise with 0x00
// ----------------------------------------------------------------------------

//void ICACHE_RAM_ATTR opmodeLora()
//{
//#ifdef CFG_sx1276_radio
//       uint8_t u = OPMODE_LORA | 0x80;   					// TBD: sx1276 high freq
//#else // SX-1272
//	    uint8_t u = OPMODE_LORA | 0x08;
//#endif
//    writeRegister(REG_OPMODE, (uint8_t) u);
//}

// ----------------------------------------------------------------------------
// Set the opmode to a value as defined on top
// Values are 0x00 to 0x07
// The value is set for the lowest 3 bits, the other bits are as before.
// ----------------------------------------------------------------------------
void opmode(uint8_t mode)
{
	if (mode == OPMODE_LORA)
		writeRegister(REG_OPMODE, (uint8_t)mode);
	else
		writeRegister(REG_OPMODE, (uint8_t)((readRegister(REG_OPMODE) & ~OPMODE_MASK) | mode));
}

// ----------------------------------------------------------------------------
// Hop to next frequency as defined by NUM_HOPS
// This function should only be used for receiver operation. The current
// receiver frequency is determined by ifreq index like so: freqs[ifreq]
// ----------------------------------------------------------------------------
void hop()
{

	// 1. Set radio to standby
	opmode(OPMODE_STANDBY);

	// 3. Set frequency based on value in freq
	ifreq = (ifreq + 1) % NUM_HOPS; // Increment the freq round robin
	setFreq(freqs[ifreq].upFreq);

	// 4. Set spreading Factor
	sf = SF7;		   // Starting the new frequency
	setRate(sf, 0x40); // set the sf to SF7

	// Low Noise Amplifier used in receiver
	writeRegister(REG_LNA, (uint8_t)LNA_MAX_GAIN); // 0x0C, 0x23

	// 7. set sync word
	writeRegister(REG_SYNC_WORD, (uint8_t)0x34); // set 0x39 to 0x34 LORA_MAC_PREAMBLE

	// prevent node to node communication
	writeRegister(REG_INVERTIQ, 0x27); // 0x33, 0x27; to reset from TX

	// Max Payload length is dependent on 256 byte buffer. At startup TX starts at
	// 0x80 and RX at 0x00. RX therefore maximized at 128 Bytes
	writeRegister(REG_MAX_PAYLOAD_LENGTH, MAX_PAYLOAD_LENGTH); // set 0x23 to 0x80==128 bytes
	writeRegister(REG_PAYLOAD_LENGTH, PAYLOAD_LENGTH);		   // 0x22, 0x40==64Byte long

	writeRegister(REG_FIFO_ADDR_PTR, (uint8_t)readRegister(REG_FIFO_RX_BASE_AD)); // set reg 0x0D to 0x0F
	writeRegister(REG_HOP_PERIOD, 0x00);										  // reg 0x24, set to 0x00

	// 5. Config PA Ramp up time								// set reg 0x0A
	writeRegister(REG_PARAMP, (readRegister(REG_PARAMP) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec

	// Set 0x4D PADAC for SX1276 ; XXX register is 0x5a for sx1272
	writeRegister(REG_PADAC_SX1276, 0x84); // set 0x4D (PADAC) to 0x84
	//writeRegister(REG_PADAC, readRegister(REG_PADAC) | 0x4);

	// 8. Reset interrupt Mask, enable all interrupts
	writeRegister(REG_IRQ_FLAGS_MASK, 0x00);

	// 9. clear all radio IRQ flags
	writeRegister(REG_IRQ_FLAGS, 0xFF);

	// Be aware that micros() has increased significantly from calling
	// the hop function until printed below
	//
#if DUSB >= 1
	if ((debug >= 2) && (pdebug & P_RADIO))
	{
		Serial.print(F("hop:: hopTime:: "));
		Serial.print(micros() - hopTime);
		Serial.print(F(", "));
		SerialStat(0);
	}
#endif
	// Remember the last time we hop
	hopTime = micros(); // At what time did we hop
}

// ----------------------------------------------------------------------------
// This LoRa function reads a message from the LoRa transceiver
// on Success: returns message length read when message correctly received
// on Failure: it returns a negative value on error (CRC error for example).
// UP function
// This is the "lowlevel" receive function called by stateMachine()
// dealing with the radio specific LoRa functions
//
// Parameters:
//		Payload: uint8_t[] message. when message is read it is returned in payload.
// Returns:
//		Length of payload received
//
// 9 bytes header
// followed by data N bytes
// 4 bytes MIC end
// ----------------------------------------------------------------------------
uint8_t receivePkt(uint8_t *payload)
{
	uint8_t irqflags = readRegister(REG_IRQ_FLAGS); // 0x12; read back flags

	statc.msg_ttl++; // Receive statistics counter

	uint8_t crcUsed = readRegister(REG_HOP_CHANNEL);
	if (crcUsed & 0x40)
	{
#if DUSB >= 1
		if ((debug >= 2) && (pdebug & P_RX))
		{
			Serial.println(F("R rxPkt:: CRC used"));
		}
#endif
	}

	//  Check for payload IRQ_LORA_CRCERR_MASK=0x20 set
	if (irqflags & IRQ_LORA_CRCERR_MASK)
	{
#if DUSB >= 1
		if ((debug >= 0) && (pdebug & P_RADIO))
		{
			Serial.print(F("rxPkt:: Err CRC, ="));
			SerialTime();
			Serial.println();
		}
#endif
		return 0;
	}

	// Is header OK?
	// Please note that if we reset the HEADER interrupt in RX,
	// that we would here conclude that there is no HEADER
	else if ((irqflags & IRQ_LORA_HEADER_MASK) == false)
	{
#if DUSB >= 1
		if ((debug >= 0) && (pdebug & P_RADIO))
		{
			Serial.println(F("rxPkt:: Err HEADER"));
		}
#endif
		// Reset VALID-HEADER flag 0x10
		writeRegister(REG_IRQ_FLAGS, (uint8_t)(IRQ_LORA_HEADER_MASK | IRQ_LORA_RXDONE_MASK)); // 0x12; clear HEADER (== 0x10) flag
		return 0;
	}

	// If there are no error messages, read the buffer from the FIFO
	// This means "Set FifoAddrPtr to FifoRxBaseAddr"
	else
	{
		statc.msg_ok++; // Receive OK statistics counter
		switch (statr[0].ch)
		{
		case 0:
			statc.msg_ok_0++;
			break;
		case 1:
			statc.msg_ok_1++;
			break;
		case 2:
			statc.msg_ok_2++;
			break;
		}

		if (readRegister(REG_FIFO_RX_CURRENT_ADDR) != readRegister(REG_FIFO_RX_BASE_AD))
		{
			if ((debug >= 0) && (pdebug & P_RADIO))
			{
				Serial.print(F("RX BASE <"));
				Serial.print(readRegister(REG_FIFO_RX_BASE_AD));
				Serial.print(F("> != RX CURRENT <"));
				Serial.print(readRegister(REG_FIFO_RX_CURRENT_ADDR));
				Serial.print(F(">"));
				Serial.println();
			}
		}

		//uint8_t currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);	// 0x10
		uint8_t currentAddr = readRegister(REG_FIFO_RX_BASE_AD); // 0x0F
		uint8_t receivedCount = readRegister(REG_RX_NB_BYTES);   // 0x13; How many bytes were read
#if DUSB >= 1
		if ((debug >= 0) && (currentAddr > 64))
		{
			Serial.print(F("rxPkt:: Rx addr>64"));
			Serial.println(currentAddr);
		}
#endif
		writeRegister(REG_FIFO_ADDR_PTR, (uint8_t)currentAddr); // 0x0D

		if (receivedCount > PAYLOAD_LENGTH)
		{
#if DUSB >= 1
			if ((debug >= 0) & (pdebug & P_RADIO))
			{
				Serial.print(F("rxPkt:: receivedCount="));
				Serial.println(receivedCount);
			}
#endif
			receivedCount = PAYLOAD_LENGTH;
		}

		for (int i = 0; i < receivedCount; i++)
		{
			payload[i] = readRegister(REG_FIFO); // 0x00, FIFO will auto shift register
		}

		writeRegister(REG_IRQ_FLAGS, (uint8_t)0xFF); // Reset ALL interrupts

		// A long as DUSB is enabled, and RX debug messages are selected,
		//the received packet is displayed on the output.
#if DUSB >= 1
		if ((debug >= 0) && (pdebug & P_RX))
		{

			Serial.print(F("rxPkt:: t="));
			SerialTime();

			Serial.print(F(", f="));
			Serial.print(ifreq);
			Serial.print(F(", sf="));
			Serial.print(sf);
			Serial.print(F(", a="));
			if (payload[4] < 0x10)
				Serial.print('0');
			Serial.print(payload[4], HEX);
			if (payload[3] < 0x10)
				Serial.print('0');
			Serial.print(payload[3], HEX);
			if (payload[2] < 0x10)
				Serial.print('0');
			Serial.print(payload[2], HEX);
			if (payload[1] < 0x10)
				Serial.print('0');
			Serial.print(payload[1], HEX);
			Serial.print(F(", flags="));
			Serial.print(irqflags, HEX);
			Serial.print(F(", addr="));
			Serial.print(currentAddr);
			Serial.print(F(", len="));
			Serial.print(receivedCount);

			// If debug level 1 is specified, we display the content of the message as well
			// We need to decode the message as well will it make any sense

			if (debug >= 1)
			{			   // Must be 1 for operational use
#if _TRUSTED_DECODE == 2
				int index; // The index of the codex struct to decode
				String response = "";

				uint8_t data[receivedCount];

				uint8_t DevAddr[4];
				DevAddr[0] = payload[4];
				DevAddr[1] = payload[3];
				DevAddr[2] = payload[2];
				DevAddr[3] = payload[1];

				if ((index = inDecodes((char *)(payload + 1))) >= 0)
				{
					Serial.print(F(", Ind="));
					Serial.print(index);
					//Serial.println();
				}
				else if (debug >= 1)
				{
					Serial.print(F(", No Index"));
					Serial.println();
					return (receivedCount);
				}

				// ------------------------------

				Serial.print(F(", data="));
				for (int i = 0; i < receivedCount; i++)
				{
					data[i] = payload[i];
				} // Copy array

				//for (int i=0; i<receivedCount; i++) {
				//	if (payload[i]<=0xF) Serial.print('0');
				//	Serial.print(payload[i], HEX);
				//	Serial.print(' ');
				//}

				uint16_t frameCount = payload[7] * 256 + payload[6];

				// The message received has a length, but data starts at byte 9, and stops 4 bytes
				// before the end since those are MIC bytes
				uint8_t CodeLength = encodePacket((uint8_t *)(data + 9), receivedCount - 9 - 4, (uint16_t)frameCount, DevAddr, decodes[index].appKey, 0);

				Serial.print(F("- NEW fc="));
				Serial.print(frameCount);
				Serial.print(F(", addr="));

				for (int i = 0; i < 4; i++)
				{
					if (DevAddr[i] <= 0xF)
						Serial.print('0');
					Serial.print(DevAddr[i], HEX);
					Serial.print(' ');
				}

				Serial.print(F(", len="));
				Serial.print(CodeLength);
				Serial.print(F(", data="));

				for (int i = 0; i < receivedCount; i++)
				{
					if (data[i] <= 0xF)
						Serial.print('0');
					Serial.print(data[i], HEX);
					Serial.print(' ');
				}
#endif // _TRUSTED_DECODE
			}

			Serial.println();

			if (debug >= 2)
				Serial.flush();
		}
#endif //DUSB
		return (receivedCount);
	}

	writeRegister(REG_IRQ_FLAGS, (uint8_t)(
									 IRQ_LORA_RXDONE_MASK |
									 IRQ_LORA_RXTOUT_MASK |
									 IRQ_LORA_HEADER_MASK |
									 IRQ_LORA_CRCERR_MASK)); // 0x12; Clear RxDone IRQ_LORA_RXDONE_MASK
	return 0;
} //receivePkt

// ----------------------------------------------------------------------------
// This DOWN function sends a payload to the LoRa node over the air
// Radio must go back in standby mode as soon as the transmission is finished
//
// NOTE:: writeRegister functions should not be used outside interrupts
// ----------------------------------------------------------------------------
bool sendPkt(uint8_t *payLoad, uint8_t payLength)
{
#if DUSB >= 2
	if (payLength >= 128)
	{
		if (debug >= 1)
		{
			Serial.print("sendPkt:: len=");
			Serial.println(payLength);
		}
		return false;
	}
#endif
	writeRegister(REG_FIFO_ADDR_PTR, (uint8_t)readRegister(REG_FIFO_TX_BASE_AD)); // 0x0D, 0x0E

	writeRegister(REG_PAYLOAD_LENGTH, (uint8_t)payLength); // 0x22
	payLoad[payLength] = 0x00;
	writeBuffer(REG_FIFO, (uint8_t *)payLoad, payLength);
	return true;
}
#endif // CFG_sx126_radio

// ----------------------------------------------------------------------------
// loraWait()
// This function implements the wait protocol needed for downstream transmissions.
// Note: Timing of downstream and JoinAccept messages is VERY critical.
//
// As the ESP8266 watchdog will not like us to wait more than a few hundred
// milliseconds (or it will kick in) we have to implement a simple way to wait
// time in case we have to wait seconds before sending messages (e.g. for OTAA 5 or 6 seconds)
// Without it, the system is known to crash in half of the cases it has to wait for
// JOIN-ACCEPT messages to send.
//
// This function uses a combination of delay() statements and delayMicroseconds().
// As we use delay() only when there is still enough time to wait and we use micros()
// to make sure that delay() did not take too much time this works.
//
// Parameter: uint32-t tmst gives the micros() value when transmission should start. (!!!)
// Note: We assume LoraDown.sfTx contains the SF we will use for downstream message.
// ----------------------------------------------------------------------------

void loraWait(const uint32_t timestamp)
{
	uint32_t startMics = micros(); // Start of the loraWait function
	uint32_t tmst = timestamp;
	// XXX
	int32_t adjust = 0;
	switch (LoraDown.sfTx)
	{
	case 7:
		adjust = 60000;
		break; // Make time for SF7 longer
	case 8:
		break; // Around 60ms
	case 9:
		break;
	case 10:
		break;
	case 11:
		break;
	case 12:
		break;
	default:
#if DUSB >= 1
		if ((debug >= 1) && (pdebug & P_TX))
		{
			Serial.print(F("T loraWait:: unknown SF="));
			Serial.print(LoraDown.sfTx);
		}
#endif
		break;
	}
	tmst = tmst + txDelay + adjust;		 // tmst based on txDelay and spreading factor
	uint32_t waitTime = tmst - micros(); // Or make waitTime an unsigned and change next statement
	if (micros() > tmst)
	{ // to (waitTime<0)
		Serial.println(F("loraWait:: Error wait time < 0"));
		return;
	}

	// For larger delay times we use delay() since that is for > 15ms
	// This is the most efficient way
	while (waitTime > 16000)
	{
		delay(15); // ms delay including yield, slightly shorter
		waitTime = tmst - micros();
	}
	// The remaining wait time is less tan 15000 uSecs
	// And we use delayMicroseconds() to wait
	if (waitTime > 0)
		delayMicroseconds(waitTime);

#if DUSB >= 1
	else if ((waitTime + 20) < 0)
	{
		Serial.println(F("loraWait:: TOO LATE")); // Never happens
	}

	if ((debug >= 2) && (pdebug & P_TX))
	{
		Serial.print(F("T start: "));
		Serial.print(startMics);
		Serial.print(F(", tmst: ")); // tmst
		Serial.print(tmst);
		Serial.print(F(", end: ")); // This must be micros(), and equal to tmst
		Serial.print(micros());
		Serial.print(F(", waited: "));
		Serial.print(tmst - startMics);
		Serial.print(F(", delay="));
		Serial.print(txDelay);
		Serial.println();
		if (debug >= 2)
			Serial.flush();
	}
#endif
}

#ifndef CFG_sx1262_radio
// ----------------------------------------------------------------------------
// txLoraModem
// Init the transmitter and transmit the buffer
// After successful transmission (dio0==1) TxDone re-init the receiver
//
//	crc is set to 0x00 for TX
//	iiq is set to 0x27 (or 0x40 based on ipol value in txpkt)
//
//	1. opmode Lora (only in Sleep mode)
//	2. opmode StandBY
//	3. Configure Modem
//	4. Configure Channel
//	5. write PA Ramp
//	6. config Power
//	7. RegLoRaSyncWord LORA_MAC_PREAMBLE
//	8. write REG dio mapping (dio0)
//	9. write REG IRQ flags
// 10. write REG IRQ mask
// 11. write REG LoRa Fifo Base Address
// 12. write REG LoRa Fifo Addr Ptr
// 13. write REG LoRa Payload Length
// 14. Write buffer (byte by byte)
// 15. Wait until the right time to transmit has arrived
// 16. opmode TX
// ----------------------------------------------------------------------------

void txLoraModem(
	uint8_t *payLoad,  // Payload contents
	uint8_t payLength, // Length of payload
	uint32_t tmst,	 // Timestamp
	uint8_t sfTx,	  // Sending spreading factor
	uint8_t powe,	  // power
	uint32_t freq,	 // frequency
	uint8_t crc,	   // Do we use CRC error checking
	uint8_t iiq		   // Interrupt
)
{
#if DUSB >= 2
	if (debug >= 1)
	{
		// Make sure that all serial stuff is done before continuing
		Serial.print(F("txLoraModem::"));
		Serial.print(F("  powe: "));
		Serial.print(powe);
		Serial.print(F(", freq: "));
		Serial.print(freq);
		Serial.print(F(", crc: "));
		Serial.print(crc);
		Serial.print(F(", iiq: 0X"));
		Serial.print(iiq, HEX);
		Serial.println();
		if (debug >= 2)
			Serial.flush();
	}
#endif
	_state = S_TX;

	// 1. Select LoRa modem from sleep mode
	//opmode(OPMODE_LORA);									// set register 0x01 to 0x80

	// Assert the value of the current mode
	ASSERT((readRegister(REG_OPMODE) & OPMODE_LORA) != 0);

	// 2. enter standby mode (required for FIFO loading))
	opmode(OPMODE_STANDBY); // set 0x01 to 0x01

	// 3. Init spreading factor and other Modem setting
	setRate(sfTx, crc);

	// Frequency hopping
	//writeRegister(REG_HOP_PERIOD, (uint8_t) 0x00);		// set 0x24 to 0x00 only for receivers

	// 4. Init Frequency, config channel
	setFreq(freq);

	// 6. Set power level, REG_PAC
	setPow(powe);

	// 7. prevent node to node communication
	writeRegister(REG_INVERTIQ, (uint8_t)iiq); // 0x33, (0x27 or 0x40)

	// 8. set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP (or less for 1ch gateway)
	writeRegister(REG_DIO_MAPPING_1, (uint8_t)(
										 MAP_DIO0_LORA_TXDONE |
										 MAP_DIO1_LORA_NOP |
										 MAP_DIO2_LORA_NOP |
										 MAP_DIO3_LORA_CRC));

	// 9. clear all radio IRQ flags
	writeRegister(REG_IRQ_FLAGS, (uint8_t)0xFF);

	// 10. mask all IRQs but TxDone
	writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t)~IRQ_LORA_TXDONE_MASK);

	// txLora
	opmode(OPMODE_FSTX); // set 0x01 to 0x02 (actual value becomes 0x82)

	// 11, 12, 13, 14. write the buffer to the FiFo
	sendPkt(payLoad, payLength);

	// 15. wait extra delay out. The delayMicroseconds timer is accurate until 16383 uSec.
	loraWait(tmst);

	//Set the base addres of the transmit buffer in FIFO
	writeRegister(REG_FIFO_ADDR_PTR, (uint8_t)readRegister(REG_FIFO_TX_BASE_AD)); // set 0x0D to 0x0F (contains 0x80);

	//For TX we have to set the PAYLOAD_LENGTH
	writeRegister(REG_PAYLOAD_LENGTH, (uint8_t)payLength); // set 0x22, max 0x40==64Byte long

	//For TX we have to set the MAX_PAYLOAD_LENGTH
	writeRegister(REG_MAX_PAYLOAD_LENGTH, (uint8_t)MAX_PAYLOAD_LENGTH); // set 0x22, max 0x40==64Byte long

	// Reset the IRQ register
	writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t)0x00);			 // Clear the mask
	writeRegister(REG_IRQ_FLAGS, (uint8_t)IRQ_LORA_TXDONE_MASK); // set 0x12 to 0x08, clear TXDONE

	// 16. Initiate actual transmission of FiFo
	opmode(OPMODE_TX); // set 0x01 to 0x03 (actual value becomes 0x83)

} // txLoraModem

// ----------------------------------------------------------------------------
// Setup the LoRa receiver on the connected transceiver.
// - Determine the correct transceiver type (sx1272/RFM92 or sx1276/RFM95)
// - Set the frequency to listen to (1-channel remember)
// - Set Spreading Factor (standard SF7)
// The reset RST pin might not be necessary for at least the RFM95 transceiver
//
// 1. Put the radio in LoRa mode
// 2. Put modem in sleep or in standby
// 3. Set Frequency
// 4. Spreading Factor
// 5. Set interrupt mask
// 6. Clear all interrupt flags
// 7. Set opmode to OPMODE_RX
// 8. Set _state to S_RX
// 9. Reset all interrupts
// ----------------------------------------------------------------------------

void rxLoraModem()
{
	// 1. Put system in LoRa mode
	//opmode(OPMODE_LORA);										// Is already so

	// 2. Put the radio in sleep mode
	opmode(OPMODE_STANDBY); // CAD set 0x01 to 0x00

	// 3. Set frequency based on value in freq
	setFreq(freqs[ifreq].upFreq); // set to the right frequency

	// 4. Set spreading Factor and CRC
	setRate(sf, 0x04);

	// prevent node to node communication
	writeRegister(REG_INVERTIQ, (uint8_t)0x27); // 0x33, 0x27; to reset from TX

	// Max Payload length is dependent on 256 byte buffer.
	// At startup TX starts at 0x80 and RX at 0x00. RX therefore maximized at 128 Bytes
	//For TX we have to set the PAYLOAD_LENGTH
	//writeRegister(REG_PAYLOAD_LENGTH, (uint8_t) PAYLOAD_LENGTH);	// set 0x22, 0x40==64Byte long

	// Set CRC Protection or MAX payload protection
	//writeRegister(REG_MAX_PAYLOAD_LENGTH, (uint8_t) MAX_PAYLOAD_LENGTH);	// set 0x23 to 0x80==128

	//Set the start address for the FiFO (Which should be 0)
	writeRegister(REG_FIFO_ADDR_PTR, (uint8_t)readRegister(REG_FIFO_RX_BASE_AD)); // set 0x0D to 0x0F (contains 0x00);

	// Low Noise Amplifier used in receiver
	writeRegister(REG_LNA, (uint8_t)LNA_MAX_GAIN); // 0x0C, 0x23

	// Accept no interrupts except RXDONE, RXTOUT en RXCRC
	writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) ~(
										  IRQ_LORA_RXDONE_MASK |
										  IRQ_LORA_RXTOUT_MASK |
										  IRQ_LORA_HEADER_MASK |
										  IRQ_LORA_CRCERR_MASK));

	// set frequency hopping
	if (_hop)
	{
		//writeRegister(REG_HOP_PERIOD, 0x01);					// 0x24, 0x01 was 0xFF
		writeRegister(REG_HOP_PERIOD, 0x00); // 0x24, 0x00 was 0xFF
	}
	else
	{
		writeRegister(REG_HOP_PERIOD, 0x00); // 0x24, 0x00 was 0xFF
	}
	// Set RXDONE interrupt to dio0
	writeRegister(REG_DIO_MAPPING_1, (uint8_t)(
										 MAP_DIO0_LORA_RXDONE |
										 MAP_DIO1_LORA_RXTOUT |
										 MAP_DIO2_LORA_NOP |
										 MAP_DIO3_LORA_CRC));

	// Set the opmode to either single or continuous receive. The first is used when
	// every message can come on a different SF, the second when we have fixed SF
	if (_cad)
	{
		// cad Scanner setup, set _state to S_RX
		// Set Single Receive Mode, goes in STANDBY mode after receipt
		_state = S_RX;
		opmode(OPMODE_RX_SINGLE); // 0x80 | 0x06 (listen one message)
	}
	else
	{
		// Set Continous Receive Mode, useful if we stay on one SF
		_state = S_RX;
		if (_hop)
			Serial.println(F("rxLoraModem:: ERROR continuous receive in hop mode"));
		opmode(OPMODE_RX); // 0x80 | 0x05 (listen)
	}

	// 9. clear all radio IRQ flags
	writeRegister(REG_IRQ_FLAGS, 0xFF);

	return;
} // rxLoraModem

// ----------------------------------------------------------------------------
// function cadScanner()
//
// CAD Scanner will scan on the given channel for a valid Symbol/Preamble signal.
// So instead of receiving continuous on a given channel/sf combination
// we will wait on the given channel and scan for a preamble. Once received
// we will set the radio to the SF with best rssi (indicating reception on that sf).
// The function sets the _state to S_SCAN
// NOTE: DO not set the frequency here but use the frequency hopper
// ----------------------------------------------------------------------------
void cadScanner()
{
	// 1. Put system in LoRa mode (which destroys all other modes)
	//opmode(OPMODE_LORA);

	// 2. Put the radio in sleep mode
	opmode(OPMODE_STANDBY); // Was old value

	// 3. Set frequency based on value in ifreq					// XXX New, might be needed when receiving down
	setFreq(freqs[ifreq].upFreq); // set to the right frequency

	// For every time we start the scanner, we set the SF to the begin value
	//sf = SF7;													// XXX 180501 Not by default

	// 4. Set spreading Factor and CRC
	setRate(sf, 0x04);

	// listen to LORA_MAC_PREAMBLE
	writeRegister(REG_SYNC_WORD, (uint8_t)0x34); // set reg 0x39 to 0x34

	// Set the interrupts we want to listen to
	writeRegister(REG_DIO_MAPPING_1, (uint8_t)(
										 MAP_DIO0_LORA_CADDONE |
										 MAP_DIO1_LORA_CADDETECT |
										 MAP_DIO2_LORA_NOP |
										 MAP_DIO3_LORA_CRC));

	// Set the mask for interrupts (we do not want to listen to) except for
	writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t) ~(
										  IRQ_LORA_CDDONE_MASK |
										  IRQ_LORA_CDDETD_MASK |
										  IRQ_LORA_CRCERR_MASK |
										  IRQ_LORA_HEADER_MASK));

	// Set the opMode to CAD
	opmode(OPMODE_CAD);

	// Clear all relevant interrupts
	//writeRegister(REG_IRQ_FLAGS, (uint8_t) 0xFF );						// May work better, clear ALL interrupts

	// If we are here. we either might have set the SF or we have a timeout in which
	// case the receive is started just as normal.
	return;

} // cadScanner

// ----------------------------------------------------------------------------
// First time initialisation of the LoRa modem
// Subsequent changes to the modem state etc. done by txLoraModem or rxLoraModem
// After initialisation the modem is put in rx mode (listen)
// ----------------------------------------------------------------------------
void initLoraModem()
{
	_state = S_INIT;
#if ESP32_ARCH == 1
	digitalWrite(pins.rst, LOW);
	delayMicroseconds(10000);
	digitalWrite(pins.rst, HIGH);
	delayMicroseconds(10000);
	digitalWrite(pins.ss, HIGH);
#if DUSB >= 1

#endif

#else
	// Reset the transceiver chip with a pulse of 10 mSec
	digitalWrite(pins.rst, HIGH);
	delayMicroseconds(10000);
	digitalWrite(pins.rst, LOW);
	delayMicroseconds(10000);
#endif
	// 2. Set radio to sleep
	opmode(OPMODE_SLEEP); // set register 0x01 to 0x00

	// 1 Set LoRa Mode
	opmode(OPMODE_LORA); // set register 0x01 to 0x80

	// 3. Set frequency based on value in freq
	setFreq(freqs[ifreq].upFreq); // set to 868.1MHz or the last saved frequency

	// 4. Set spreading Factor
	setRate(sf, 0x04);

	// Low Noise Amplifier used in receiver
	writeRegister(REG_LNA, (uint8_t)LNA_MAX_GAIN); // 0x0C, 0x23
#if _PIN_OUT == 4
	delay(1);
#endif
	uint8_t version = readRegister(REG_VERSION); // Read the LoRa chip version id
	if (version == 0x22)
	{
		// sx1272
#if DUSB >= 2
		Serial.println(F("WARNING:: SX1272 detected"));
#endif
		sx1272 = true;
	}

	else if (version == 0x12)
	{
		// sx1276?
#if DUSB >= 2
		if (debug >= 1)
			Serial.println(F("SX1276 starting"));
#endif
		sx1272 = false;
	}
	else
	{
		// Normally this means that we connected the wrong type of board and
		// therefore specified the wrong type of wiring/pins to the software
		// Maybe this issue can be resolved of we try one of the other defined
		// boards. (Comresult or Hallard or ...)
#if DUSB >= 1
		Serial.print(F("Unknown transceiver="));
		Serial.print(version, HEX);
		Serial.print(F(", pins.rst ="));
		Serial.print(pins.rst);
		Serial.print(F(", pins.ss  ="));
		Serial.print(pins.ss);
		Serial.print(F(", pins.dio0 ="));
		Serial.print(pins.dio0);
		Serial.print(F(", pins.dio1 ="));
		Serial.print(pins.dio1);
		Serial.print(F(", pins.dio2 ="));
		Serial.print(pins.dio2);
		Serial.println();
		Serial.flush();
#endif
		die(""); // Maybe first try another kind of receiver
	}
	// If we are here, the chip is recognized successfully

	// 7. set sync word
	writeRegister(REG_SYNC_WORD, (uint8_t)0x34); // set 0x39 to 0x34 LORA_MAC_PREAMBLE

	// prevent node to node communication
	writeRegister(REG_INVERTIQ, 0x27); // 0x33, 0x27; to reset from TX

	// Max Payload length is dependent on 256 byte buffer. At startup TX starts at
	// 0x80 and RX at 0x00. RX therefore maximized at 128 Bytes
	writeRegister(REG_MAX_PAYLOAD_LENGTH, MAX_PAYLOAD_LENGTH); // set 0x23 to 0x80==128 bytes
	writeRegister(REG_PAYLOAD_LENGTH, PAYLOAD_LENGTH);		   // 0x22, 0x40==64Byte long

	writeRegister(REG_FIFO_ADDR_PTR, (uint8_t)readRegister(REG_FIFO_RX_BASE_AD)); // set reg 0x0D to 0x0F
	writeRegister(REG_HOP_PERIOD, 0x00);										  // reg 0x24, set to 0x00

	// 5. Config PA Ramp up time								// set reg 0x0A
	writeRegister(REG_PARAMP, (readRegister(REG_PARAMP) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec

	// Set 0x4D PADAC for SX1276 ; XXX register is 0x5a for sx1272
	writeRegister(REG_PADAC_SX1276, 0x84); // set 0x4D (PADAC) to 0x84
	//writeRegister(REG_PADAC, readRegister(REG_PADAC) | 0x4);

	// Reset interrupt Mask, enable all interrupts
	writeRegister(REG_IRQ_FLAGS_MASK, 0x00);

	// 9. clear all radio IRQ flags
	writeRegister(REG_IRQ_FLAGS, 0xFF);
} // initLoraModem

// ----------------------------------------------------------------------------
// Void function startReceiver.
// This function starts the receiver loop of the LoRa service.
// It starts the LoRa modem with initLoraModem(), and then starts
// the receiver either in single message (CAD) of in continuous
// reception (STD).
// ----------------------------------------------------------------------------
void startReceiver()
{
	initLoraModem(); // XXX 180326, after adapting this function
	if (_cad)
	{
#if DUSB >= 1
		if ((debug >= 1) && (pdebug & P_SCAN))
		{
			Serial.println(F("S PULL:: _state set to S_SCAN"));
			if (debug >= 2)
				Serial.flush();
		}
#endif
		_state = S_SCAN;
		sf = SF7;
		cadScanner();
	}
	else
	{
		_state = S_RX;
		rxLoraModem();
	}
	writeRegister(REG_IRQ_FLAGS_MASK, (uint8_t)0x00);
	writeRegister(REG_IRQ_FLAGS, 0xFF); // Reset all interrupt flags
}
#endif

// ----------------------------------------------------------------------------
// Interrupt_0 Handler.
// Both interrupts DIO0 and DIO1 are mapped on GPIO15. Se we have to look at
// the interrupt flags to see which interrupt(s) are called.
//
// NOTE:: This method may work not as good as just using more GPIO pins on
//  the ESP8266 mcu. But in practice it works good enough
// ----------------------------------------------------------------------------
void ICACHE_RAM_ATTR Interrupt_0()
{
	_event = 1;
}

// ----------------------------------------------------------------------------
// Interrupt handler for DIO1 having High Value
// As DIO0 and DIO1 may be multiplexed on one GPIO interrupt handler
// (as we do) we have to be careful only to call the right Interrupt_x
// handler and clear the corresponding interrupts for that dio.
// NOTE: Make sure all Serial communication is only for debug level 3 and up.
// Handler for:
//		- CDDETD
//		- RXTIMEOUT
//		- (RXDONE error only)
// ----------------------------------------------------------------------------
void ICACHE_RAM_ATTR Interrupt_1()
{
	_event = 1;
}

// ----------------------------------------------------------------------------
// Frequency Hopping Channel (FHSS) dio2
// ----------------------------------------------------------------------------
void ICACHE_RAM_ATTR Interrupt_2()
{
	_event = 1;
}
