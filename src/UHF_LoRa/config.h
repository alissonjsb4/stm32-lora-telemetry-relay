//
// Created by SQ5RWU on 2016-12-24.
// Modified by VK5QI in 2018-ish
//

#ifndef RS41HUP_CONFIG_H
#define RS41HUP_CONFIG_H

#ifdef USE_EXTERNAL_CONFIG
#include "config_external.h"
#else


//************GLOBAL Settings*****************
// This is the main frequency.  To send alternating with a second frequency, enable the following line
//================================================
#define TRANSMIT_FREQUENCY  434.7140f //Mhz middle frequency
//#define TRANSMIT_FREQUENCY_2ND  437.6000f //Mhz middle frequency
#define TRANSMIT_APRS_FREQUENCY  434.7140f //Mhz middle frequency
//================================================

#define BAUD_RATE  100 // RTTY & MFSK Baud rate.  PLEASE DONT CHANGE
                       // NOTE: Currently supported MFSK baud rates with horus-gui are 50 and 100 baud,
                       // with the suggested MFSK baud rate being 100 baud.

// Modulation Types - Comment out a line below to enable/disable a modulation.
//================================================
//#define RTTY_ENABLED
#define MFSK_4_ENABLED
//#define APRS_1200_ENABLED
#define LORA_ENABLED
//================================================


//*************RTTY SETTINGS******************
#define CALLSIGN "URCALL" // put your RTTY callsign here, max. 15 characters
#define RTTY_DEVIATION 0x3	// RTTY shift = RTTY_DEVIATION x 270Hz
#define RTTY_7BIT   1 // if 0 --> 5 bits
#define RTTY_USE_2_STOP_BITS   1

//*************APRS SETTINGS******************
//....................if call shorter then 6 chars, it will be filled up with a blank internally in QAPRS class init
#define APRS_CALLSIGN "DF7PN" // put your APRS callsign here max. 15 characters
#define APRS_SSID 7
#define APRS_COMMENT " RS41 Balloon"
#define APRS_RATIO 2   // Every TX loop counts one down then send a APRS packet if nbr 1 reached - so 2 means every second TX
#define APRS_DAO 1     // if DAO 1 then a better position resolution will be used (1 mtr), if set to "0" - only 13 mtr
						// DAO is placed in the transmitted packet behind the COMMENT as e.g.  RS41 Balloon  !wbZ!

//************MFSK Binary Settings************
// Binary Payload ID (0 though 255) - For your own flights, you will need to request a payload ID,
// and set this value to that. 
// Refer to the payload ID list here: https://github.com/projecthorus/horusdemodlib/blob/master/payload_id_list.txt
// Payload IDs can be reqested by either raising an Issue, or a Pull Request on https://github.com/projecthorus/horusdemodlib/
// VERSION 1 = An ID of 0 ('4FSKTEST') can be used for on-ground testing, but DO NOT use this for a flight!!!
// VERSION 2 = An ID of 256 ('4FSKTEST-V2') can be used for on-ground testing, but DO NOT use this for a flight!!!
//================================================
//#define HORUS_V1
#define HORUS_V2
//================================================

#ifdef HORUS_V1
  #define BINARY_PAYLOAD_ID 0 // Payload ID for use in Binary packets
#endif
// ...or... (dont activate both)
#ifdef HORUS_V2
  #define BINARY_PAYLOAD_ID 256 //  Payload ID for use in Binary packets
#endif

#if defined(HORUS_V1) && defined(HORUS_V2)
#error "Please select only one HORUS-Mode."
//- ONLY ON V2: - your own flight number and sonde type
// WHAT TO DO BEFORE USE?
// (1) request Mark to add to custom_field_list.json block same as DF7PN for your BINARY_PAYLOAD_ID ------------------
//     see https://github.com/projecthorus/horusdemodlib/blob/master/custom_field_list.json
//     Example: https://github.com/projecthorus/horusdemodlib/issues/208
// (2) If issue closed - every RX have to restart his software - so start your own first
// (3) now enable the fields USERFLAG_A and USERFLAG_B - fill them with your constant numbers
// (4) program a rs41 with this and wait for popup on the website  amateur.sondehub.org
// (5) click your sonde and look on the expanded tabsheet left - behind SPEED there are the new Fields FLIGHT NUMBER and SONDE TYPE

// USERFLAG_A : your personal flight number - count up one on each flight with same BINARY_PAYLOAD_ID - you have your own flight log!
//================================================
//#define USERFLAG_A 1   // FlightNumber  0..65535 (0xFFFF)
//================================================
// USERFLAG_B : declare a number out of the list what kind of sonde type your payload is
//================================================
//#define USERFLAG_B 0   // SondeType - see List below or at README in Github
//================================================
//.............SondeType..........................
/* 0 = undefined payload
 * 1 = RS41 default
 * 2 = RS41 naked
 * 3 = RS41 solar
 * 4 = RS41 solar+Bat mixed
 * 5 = Arduino - RFM
 * 6 = more??
 */
//================================================
#endif


// TX Power
//================================================
#define TX_POWER  5 // PWR 0...7 0- MIN ... 7 - MAX
//================================================
// Power Levels measured at 434.650 MHz, using a Rigol DSA815, and a 10 kHz RBW
// Power measured by connecting a short (30cm) length of RG316 directly to the
// antenna/ground pads at the bottom of the RS41 PCB.

// 0 --> -1.9dBm
// 1 --> 1.3dBm
// 2 --> 3.6dBm
// 3 --> 7.0dBm
// 4 --> 11.0dBm
// 5 --> 13.1dBm
// 6 --> 15.0dBm
// 7 --> 16.3dBm

// *********** Power Saving ******************
//
// Power Consumption Notes (all @ 3V):
// - GPS Max Performance, Transmitting @ 13 dBm = ~150 mA
// - GPS Max Performance, Not Transmitting = 70-90mA
// - GPS in PowerSave Mode, Transmitting @ 13 dBm = ~120 mA
// - GPS in PowerSave Mode, not Transmitting = 30-50mA, depending on GPS state.
// If enabled, transmit incrementing tones in the 'idle' period between packets.
// This will only function if ONLY MFSK is enabled.
// Note, you need to COMMENT this line out to disable this, not just set it to 0
//================================================
//#define CONTINUOUS_MODE 1
//================================================

// If anyway GPS Fix gets lost a counter increments each TX-Intervall
// if counter reaches NOGPS_RESET_AFTER_TXCOUNT then an SystemRestart will be fired
// Timeout can be calculated :  TX_DELAY * NOGPS_RESET_AFTER_TXCOUNT
// After this, the TX Counter starts again at nbr 1
// This works at start of the Sonde, also during flight if something disturbs GPS reception (jamming over military areas)
// DISABLE:  comment out this line //
// Recommendation:  request for reboot after 7 Minutes if TX_DELAY 60s : = 420 Seconds * 1000 (ms) / TX_DELAY (ms) - round to integer please
// if TX_DELAY smaller, make NOGPS_RESET_AFTER_TXCOUNT bigger
//================================================
#define NOGPS_RESET_AFTER_TXCOUNT 7  //tá contando 1m03s por minuto ?
//================================================

// Delay *between* transmitted packets (milliseconds)
// If you only have MFSK_4 enabled, and MFSK_CONTINUOUS (below) is disabled,
// Then the transmitter will turn off between transmissions. This saves about 50mA of power consumption.
// The maximum TX_DELAY is 65535*(1000/BAUD_RATE), so about 655.35 seconds for 100 baud
//================================================
#define TX_DELAY  15000
//================================================
//Shift TX Time within the minute up to milliseconds to avoid overlapping with other TX
//Only works if SYNC_TX_WITH_GPS is activ
//ATTENTION: do NOT set OFFSET HIGHER THEN TX_DELAY - This results in unexpected behavior
//================================================
#define TX_DELAY_OFFSET  5000
//================================================

// Try to sync the TX to start on full minute if GPSfix is available.
// Disable: insert "//" before
//================================================
#define SYNC_TX_WITH_GPS
//================================================

// If defined, transmit a short 20ms 'pip' between transmissions every X milliseconds.
// This number needs to be smaller than TX_DELAY
// IF TRANSMIT_FREQUENCY_2ND is set, then PIP changes each interval (5s) also on the 2nd frequency
// ... so PIP is heard on one frequency each 10s.
// Comment out the line to disable this.
//================================================
//#define TX_PIP  5000
//================================================
// Number of symbols to transmit the pip for. 
// At 100 baud, each symbol is 10ms
//================================================
#define TX_PIP_SYMBOLS  5
//================================================

// Enable uBlox PowerSave Mode
// Drops current consummption from the GPS somewhat.
// Positional accuracy may be slightly impacted. Suggest not using this for short flights.
// Flight-tested on 2020-12.
// if "1" then the sat-counter may be start with 100 oder 200 values. If 2xx reached, this needs half enery then in full mode
//================================================
#define UBLOX_POWERSAVE 1
//================================================

// *********** Morse Ident **********************
// If uncommented, send a morse code ident every X transmit cycles, to comply
// with amateur radio regulations, if operating under an amateur radio license.
// With continuous 4FSK transmissions, 100 transmit cycles is approx 5 minutes.
///================================================
//#define MORSE_IDENT 5
//================================================

// If enabled above, Morse message to send.
//================================================
#define MORSE_MESSAGE "VVV DE DF7PN "
// want more details in the morese message? enable MORSE_EXTENDED_MSG (recommended)
// then following data is appended to MORSE_MESSAGE:
// / Altitude in meters (A) / Satcount (S) / Locator (QRA) 8 Characters eg: JO52AX01
#define MORSE_EXTENDED_MSG
// Speed of morse transmission
#define MORSE_WPM 28
//================================================

// *********** Deep Sleep Mode ****(not tested yet by whallmann) **************
// Deep Sleep Mode intended for long duration flights only!
//
// Notes:
// - Only have MFSK transmissions enabled.
// - Disable continuous mode.
// - Disable GPS powersave mode. 
//
// Power consumption in sleep mode = 32mA @ 3V
//
// In this mode, the GPS is turned into a sleep mode in between transmissions.
// During this sleep period, we sent one 'pip' every few seconds.
// At the end of the sleep period, the GPS is powered back up, and we await the GPS
// to obtain a fix before transmitting our position. While waiting for GPS lock, we
// send a 'double pip'.

// Go to sleep for X minutes between transmissions.
// In this time, the GPS will be completely powered down.
// If defined, sleep for this many minutes between transmissions.
//================================================
//#define DEEP_SLEEP 2
//================================================

// Send a short pip every X milliseconds to let people know the transmitter is running.
//================================================
#define DEEP_SLEEP_PIPS 10000
//================================================



//***********Other Settings ******************
// Switch sonde ON/OFF via Button
// If this is a flight you might prevent sonde from powered off by button
//================================================
#define ALLOW_DISABLE_BY_BUTTON 1
//================================================


#endif

#endif //RS41HUP_CONFIG_H
