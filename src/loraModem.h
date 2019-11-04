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

#ifndef LORAMODEM_H
#define LORAMODEM_H

#ifdef _LOCALSERVER 
// Sometimes we want to decode the sensor completely as we do in the TTN server
// This means that for all nodes we want to view the data of, we need to provide
// the AppsSKey and the NwkSKey
// Define the number of known nodes here and add the same number of nodes to the 
// struct codex decodes[] in loraModem.cpp
#define KNOWN_NODES 2

struct codex
{
	uint32_t id;		// This is the device ID (coded in 4 bytes uint32_t
	char nm[32];		// A name string which is free to choose
	uint8_t nwkKey[16]; // The Network Session Key of 16 bytes
	uint8_t appKey[16]; // The Application Session Key of 16 bytes
};

extern struct codex decodes[KNOWN_NODES];
#endif

// ----------------------------------------
// Used by REG_PAYLOAD_LENGTH to set receive payload length
#define PAYLOAD_LENGTH 0x40		// 64 bytes
#define MAX_PAYLOAD_LENGTH 0x80 // 128 bytes

// In order to make the CAD behaviour dynamic we set a variable
// when the CAD functions are defined. Value of 3 is minimum frequencies a
// gateway should support to be fully LoRa compliant.
// For performance reasons, 3 is the maximum as well!
//
#define NUM_HOPS 3

// Do not change these setting for RSSI detection. They are used for CAD
// Given the correction factor of 157, we can get to -122dB with this rating
//
#define RSSI_LIMIT 35 //

// How long to wait in LoRa mode before using the RSSI value.
// This period should be as short as possible, yet sufficient
//
#define RSSI_WAIT 6 // was 25

// How long will it take when hopping before a CDONE or CDETD value
// is present and can be measured.
//
#define EVENT_WAIT 15000 // XXX 180520 was 25 milliseconds before CDDETD timeout
#define DONE_WAIT 1950   // 2000 microseconds (1/500) sec between CDDONE events

// Our code should correct the server Tramission delay settings
// long txDelay = 0x00; // tx delay time on top of server TMST
extern long txDelay;

// SPI setting. 8MHz seems to be the max
#define SPISPEED 8000000 // Set to 8 * 10E6

// Frequencies
// Set center frequency. If in doubt, choose the first one, comment all others
// Each "real" gateway should support the first 3 frequencies according to LoRa spec.
// NOTE: This means you have to specify at least 3 frequencies here for the single
//	channel gateway to work.
struct vector
{
	// Upstream messages
	uint32_t upFreq; // 4 bytes
	uint16_t upBW;   // 2 bytes
	uint8_t upLo;	// 1 bytes
	uint8_t upHi;	// 1 bytes
	// Downstream messages
	uint32_t dwnFreq; // 4 bytes Unsigned ubt Frequency
	uint16_t dwnBW;   // 2 bytes BW Specification
	uint8_t dwnLo;	// 1 bytes Spreading Factor
	uint8_t dwnHi;	// 1 bytes
};

#ifdef EU863_870
// This the the EU863_870 format as used in most of Europe
// It is also the default for most of the single channel gateway work.
// For each frequency SF7-SF12 are used.
extern vector freqs[10];
	#elif defined(EU433)
	// The following 3 frequencies should be defined/used in an EU433
	// environment. The plan is not defined for TTN yet so we use this one.
extern vector freqs[9];
#elif defined(US902_928)
// The frequency plan for USA is a difficult one. As yout can see, the uplink protocol uses
// SF7-SF10 and BW125 whereas the downlink protocol uses SF7-SF12 and BW500.
// Also the number of chanels is not equal.
extern vector freqs[9];
#elif defined(AU925_928)
// Australian plan or TTN/Lora frequencies
extern vector freqs[9];
#elif defined(CN470_510)
extern vector freqs[8];
#else
extern vector freqs[1];
#error "Sorry, but your frequency plan is not supported"
#endif

// Set the structure for spreading factor
enum sf_t
{
	SF6 = 6,
	SF7,
	SF8,
	SF9,
	SF10,
	SF11,
	SF12
};

// The state of the receiver. See Semtech Datasheet (rev 4, March 2015) page 43
// The _state is of the enum type (and should be cast when used as a number)
enum state_t
{
	S_INIT = 0,
	S_SCAN,
	S_CAD,
	S_RX,
	S_TX,
	S_TXDONE
};

extern volatile state_t _state;
extern volatile uint8_t _event;

// // rssi is measured at specific moments and reported on others
// // so we need to store the current value we like to work with
extern uint8_t _rssi;
/// \todo CAD and HOP are not supported by SX1262 (yet)
extern bool _cad; // Set to true for Channel Activity Detection, only when dio 1 connected
extern bool _hop; // experimental; frequency hopping. Only use when dio2 connected

extern unsigned long nowTime;
extern unsigned long msgTime;
extern unsigned long hopTime;
extern unsigned long detTime;

// // stat_t contains the statistics that are kept by message.
// // Each time a message is received or sent the statistics are updated.
// // In case STATISTICS==1 we define the last MAX_STAT messages as statistics
struct stat_t
{
	unsigned long tmst; // Time since 1970 in seconds
	unsigned long node; // 4-byte DEVaddr (the only one known to gateway)
	uint8_t ch;			// Channel index to freqs array
	uint8_t sf;
#if RSSI == 1
	int8_t rssi; // XXX Can be < -128
#endif
	int8_t prssi; // XXX Can be < -128
#if _LOCALSERVER == 1
	uint8_t data[23]; // For memory purposes, only 23 chars
	uint8_t datal;	// Length of decoded message 1 char
#endif
};
extern struct stat_t stat_t;

#if STATISTICS >= 1
// // statc_c contains the statistic that are gateway related and not per
// // message. Example: Number of messages received on SF7 or number of (re) boots
// // So where statr contains the statistics gathered per packet the statc_c
// // contains general statistics of the node

struct stat_c
{

	unsigned long msg_ok;
	unsigned long msg_ttl;
	unsigned long msg_down;

#if STATISTICS >= 2		// Only if we explicitly set it higher
	unsigned long sf7;  // Spreading factor 7 statistics/Count
	unsigned long sf8;  // Spreading factor 8
	unsigned long sf9;  // Spreading factor 9
	unsigned long sf10; // Spreading factor 10
	unsigned long sf11; // Spreading factor 11
	unsigned long sf12; // Spreading factor 12

	// If STATISTICS is 3, we add statistics about the channel
	// When only one channel is used, we normally know the spread of
	// statistics, but when HOP mode is selected we migth want to add this info
#if STATISTICS >= 3
	unsigned long msg_ok_0, msg_ok_1, msg_ok_2;
	unsigned long msg_ttl_0, msg_ttl_1, msg_ttl_2;
	unsigned long msg_down_0, msg_down_1, msg_down_2;

	unsigned long sf7_0, sf7_1, sf7_2;
	unsigned long sf8_0, sf8_1, sf8_2;
	unsigned long sf9_0, sf9_1, sf9_2;
	unsigned long sf10_0, sf10_1, sf10_2;
	unsigned long sf11_0, sf11_1, sf11_2;
	unsigned long sf12_0, sf12_1, sf12_2;
#endif //3

	uint16_t boots; // Number of boots
	uint16_t resets;
#endif // 2

};
extern struct stat_c statc;

// // History of received uplink messages from nodes
extern struct stat_t statr[MAX_STAT];

#else // STATISTICS==0
extern struct stat_t statr[1]; // Always have at least one element to store in
#endif

// // Define the payload structure used to separate interrupt ans SPI
// // processing from the loop() part
extern uint8_t payLoad[128];
struct LoraBuffer
{
	uint8_t *payLoad;
	uint8_t payLength;
	uint32_t tmst; // in millis()
	uint8_t sfTx;
	uint8_t powe;
	uint32_t fff;
	uint8_t crc;
	uint8_t iiq;
};

extern struct LoraBuffer LoraDown;

// // Up buffer (from Lora sensor to UDP)
// //

struct LoraUp
{
	uint8_t payLoad[128];
	uint8_t payLength;
	int prssi;
	long snr;
	int rssicorr;
	uint8_t sf;
};

extern struct LoraUp LoraUp;

// ============================================================================
// Set all definitions for Gateway
// ============================================================================
// Register definitions. These are the addresses of the RFM95, SX1276 that we
// need to set in the program.

#define REG_FIFO 0x00 // rw FIFO address
#define REG_OPMODE 0x01
// Register 2 to 5 are unused for LoRa
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PAC 0x09
#define REG_PARAMP 0x0A
#define REG_LNA 0x0C
#define REG_FIFO_ADDR_PTR 0x0D   // rw SPI interface address pointer in FIFO data buffer
#define REG_FIFO_TX_BASE_AD 0x0E // rw write base address in FIFO data buffer for TX modulator
#define REG_FIFO_RX_BASE_AD 0x0F // rw read base address in FIFO data buffer for RX demodulator (0x00)

#define REG_FIFO_RX_CURRENT_ADDR 0x10 // r  Address of last packet received
#define REG_IRQ_FLAGS_MASK 0x11
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI 0x1A // latest package
#define REG_RSSI 0x1B	 // Current RSSI, section 6.4, or  5.5.5
#define REG_HOP_CHANNEL 0x1C
#define REG_MODEM_CONFIG1 0x1D
#define REG_MODEM_CONFIG2 0x1E
#define REG_SYMB_TIMEOUT_LSB 0x1F

#define REG_PAYLOAD_LENGTH 0x22
#define REG_MAX_PAYLOAD_LENGTH 0x23
#define REG_HOP_PERIOD 0x24
#define REG_MODEM_CONFIG3 0x26
#define REG_RSSI_WIDEBAND 0x2C

#define REG_INVERTIQ 0x33
#define REG_DET_TRESH 0x37 // SF6
#define REG_SYNC_WORD 0x39
#define REG_TEMP 0x3C

#define REG_DIO_MAPPING_1 0x40
#define REG_DIO_MAPPING_2 0x41
#define REG_VERSION 0x42

#define REG_PADAC 0x5A
#define REG_PADAC_SX1272 0x5A
#define REG_PADAC_SX1276 0x4D

// ----------------------------------------
// opModes
#define SX72_MODE_SLEEP 0x80
#define SX72_MODE_STANDBY 0x81
#define SX72_MODE_FSTX 0x82
#define SX72_MODE_TX 0x83 // 0x80 | 0x03
#define SX72_MODE_RX_CONTINUOS 0x85

// ----------------------------------------
// LMIC Constants for radio registers
#define OPMODE_LORA 0x80
#define OPMODE_MASK 0x07
#define OPMODE_SLEEP 0x00
#define OPMODE_STANDBY 0x01
#define OPMODE_FSTX 0x02
#define OPMODE_TX 0x03
#define OPMODE_FSRX 0x04
#define OPMODE_RX 0x05
#define OPMODE_RX_SINGLE 0x06
#define OPMODE_CAD 0x07

// ----------------------------------------
// LOW NOISE AMPLIFIER

#define LNA_MAX_GAIN 0x23 // Max gain 0x20 | Boost 0x03
#define LNA_OFF_GAIN 0x00
#define LNA_LOW_GAIN 0x20

// CONF REG
#define REG1 0x0A
#define REG2 0x84

// ----------------------------------------
// MC1 sx1276 RegModemConfig1
#define SX1276_MC1_BW_125 0x70
#define SX1276_MC1_BW_250 0x80
#define SX1276_MC1_BW_500 0x90
#define SX1276_MC1_CR_4_5 0x02 // sx1276
#define SX1276_MC1_CR_4_6 0x04
#define SX1276_MC1_CR_4_7 0x06
#define SX1276_MC1_CR_4_8 0x08
#define SX1276_MC1_IMPLICIT_HEADER_MODE_ON 0x01

#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE 0x01 // mandated for SF11 and SF12

// ----------------------------------------
// MC2 definitions
#define SX72_MC2_FSK 0x00
#define SX72_MC2_SF7 0x70 // SF7 == 0x07, so (SF7<<4) == SX7_MC2_SF7
#define SX72_MC2_SF8 0x80
#define SX72_MC2_SF9 0x90
#define SX72_MC2_SF10 0xA0
#define SX72_MC2_SF11 0xB0
#define SX72_MC2_SF12 0xC0

// ----------------------------------------
// MC3
#define SX1276_MC3_LOW_DATA_RATE_OPTIMIZE 0x08
#define SX1276_MC3_AGCAUTO 0x04

// ----------------------------------------
// FRF
#define FRF_MSB 0xD9 // 868.1 MHz
#define FRF_MID 0x06
#define FRF_LSB 0x66

// ----------------------------------------
// DIO function mappings           		     D0D1D2D3
#define MAP_DIO0_LORA_RXDONE 0x00  // 00------ bit 7 and 6
#define MAP_DIO0_LORA_TXDONE 0x40  // 01------
#define MAP_DIO0_LORA_CADDONE 0x80 // 10------
#define MAP_DIO0_LORA_NOP 0xC0	 // 11------

#define MAP_DIO1_LORA_RXTOUT 0x00	// --00---- bit 5 and 4
#define MAP_DIO1_LORA_FCC 0x10		 // --01----
#define MAP_DIO1_LORA_CADDETECT 0x20 // --10----
#define MAP_DIO1_LORA_NOP 0x30		 // --11----

#define MAP_DIO2_LORA_FCC0 0x00 // ----00-- bit 3 and 2
#define MAP_DIO2_LORA_FCC1 0x04 // ----01-- bit 3 and 2
#define MAP_DIO2_LORA_FCC2 0x08 // ----10-- bit 3 and 2
#define MAP_DIO2_LORA_NOP 0x0C  // ----11-- bit 3 and 2

#define MAP_DIO3_LORA_CADDONE 0x00 // ------00 bit 1 and 0
#define MAP_DIO3_LORA_HEADER 0x01  // ------01
#define MAP_DIO3_LORA_CRC 0x02	 // ------10
#define MAP_DIO3_LORA_NOP 0x03	 // ------11

// FSK specific
#define MAP_DIO0_FSK_READY 0x00   // 00------ (packet sent / payload ready)
#define MAP_DIO1_FSK_NOP 0x30	 // --11----
#define MAP_DIO2_FSK_TXNOP 0x04   // ----01--
#define MAP_DIO2_FSK_TIMEOUT 0x08 // ----10--

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK 0x80 // RXTOUT
#define IRQ_LORA_RXDONE_MASK 0x40 // RXDONE after receiving the header and CRC, we receive payload part
#define IRQ_LORA_CRCERR_MASK 0x20 // CRC error detected. Note that RXDONE will also be set
#define IRQ_LORA_HEADER_MASK 0x10 // valid HEADER mask. This interrupt is first when receiving a message
#define IRQ_LORA_TXDONE_MASK 0x08 // End of TRansmission
#define IRQ_LORA_CDDONE_MASK 0x04 // CDDONE
#define IRQ_LORA_FHSSCH_MASK 0x02
#define IRQ_LORA_CDDETD_MASK 0x01 // Detect preamble channel

// ----------------------------------------
// Definitions for UDP message arriving from server
#define PROTOCOL_VERSION 0x01
#define PKT_PUSH_DATA 0x00
#define PKT_PUSH_ACK 0x01
#define PKT_PULL_DATA 0x02
#define PKT_PULL_RESP 0x03
#define PKT_PULL_ACK 0x04
#define PKT_TX_ACK 0x05

#define MGT_RESET 0x15 // Not a LoRa Gateway Spec message
#define MGT_SET_SF 0x16
#define MGT_SET_FREQ 0x17

#endif // LORAMODEM_H