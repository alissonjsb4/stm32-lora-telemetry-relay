//
// Created by Admin on 2017-01-09.
//

#include "math.h"
#include "aprs.h"
#include "QAPRSBase.h"
#include "stdio.h"
#include "ublox.h"
#include "config.h"
//#include <cmath>

#if !defined(__OPTIMIZE__)
#error "APRS Works only when optimization enabled at level at least -O2"
#endif
QAPRSBase qaprs;

void aprs_init(){

	// fill up the callsign to at least 6 characters by blanks
	char aprs_callsign[15] = {APRS_CALLSIGN};
	uint8_t laenge = strlen(aprs_callsign);
	while (laenge < 6) {
		strcat (aprs_callsign," ");
		laenge = strlen(aprs_callsign);
	}
	qaprs.init(0, 0, (char *) aprs_callsign, (const uint8_t) APRS_SSID, (char *) "APZQAP", '0', (char *) "WIDE1-1");
}

void aprs_timer_handler() {
  qaprs.timerInterruptHandler();
}

uint8_t aprs_is_active() {
  return qaprs.enabled;
}

void calcDMH(long x, int8_t* degrees, uint8_t* minutes, uint8_t* h_minutes) {
  uint8_t sign = (uint8_t) (x > 0 ? 1 : 0);
  if (!sign) {
    x = -(x);
  }
  *degrees = (int8_t) (x / 1000000);
  x = x - (*degrees * 1000000);
  x = (x) * 60 / 10000;
  *minutes = (uint8_t) (x / 100);
  *h_minutes = (uint8_t) (x - (*minutes * 100));
  if (!sign) {
    *degrees = -*degrees;
  }
}

void aprs_test(){
  char packet_buffer[128];
  sprintf(packet_buffer,
          (":TEST1234567890")
  );
  qaprs.sendData(packet_buffer);
}

#ifdef APRS_DAO
//--- only for DAO Extension nessesary
//DAO?  see http://www.aprs.org/aprs12/datum.txt
//Code from Hansi DL9RDZ

static uint32_t truncc(double r)
{
   if (r<=0.0L) {
	   return 0UL;
   } else if (r>=2.E+9) {
	   return 2000000000UL;
   } else {
	   return (uint32_t)r;
   }
   return 0;
} /* end truncc() */

static uint32_t dao91(double x)
/* radix91(xx/1.1) of dddmm.mmxx */
{
   double a;
   a = fabs(x);
   return ((truncc((a-(double)(float)truncc(a))*6.E+5)%100UL)
                *20UL+11UL)/22UL;
} /* end dao91() */
#endif

void aprs_send_position(GPSEntry gpsData, int8_t temperature, uint16_t voltage) {
  char packet_buffer[228];
  char extension_packet_buffer[128];
  int8_t la_degrees, lo_degrees;
  uint8_t la_minutes, la_h_minutes, lo_minutes, lo_h_minutes;
  // uint8_t j;

  calcDMH(gpsData.lat_raw/10, &la_degrees, &la_minutes, &la_h_minutes);
  calcDMH(gpsData.lon_raw/10, &lo_degrees, &lo_minutes, &lo_h_minutes);

  static uint16_t aprs_packet_counter = 0;
  aprs_packet_counter ++;


#ifdef APRS_DAO
  if (APRS_DAO == 1) {
	  //________________________________________________
	  // DAO Extention for better positioning
	  float  lat = gpsData.lat_raw / 10000000.0f;
	  float  lon = gpsData.lon_raw / 10000000.0f;
	  //________________________________________________
	  uint8_t j = sprintf(extension_packet_buffer," %s",APRS_COMMENT);
	  sprintf(extension_packet_buffer + j, " !w%c%c!", (char)(33+dao91(lat)), (char)(33+dao91(lon)));
  } else {
	sprintf(extension_packet_buffer," %s",APRS_COMMENT);
  }
#else
  sprintf(extension_packet_buffer," %s",APRS_COMMENT);
#endif

  sprintf(packet_buffer,
          ("!%02d%02d.%02u%c/%03d%02u.%02u%cO/A=%06ld/P%dS%dT%dV%d%s"),
          abs(la_degrees), la_minutes, la_h_minutes,
          la_degrees > 0 ? 'N' : 'S',
          abs(lo_degrees), lo_minutes, lo_h_minutes,
          lo_degrees > 0 ? 'E' : 'W',
          (gpsData.alt_raw/1000) * 3280 / 1000,
          aprs_packet_counter,
          gpsData.sats_raw,
          temperature,
          voltage,
          extension_packet_buffer
  );
  qaprs.sendData(packet_buffer);
}

void aprs_change_tone_time(uint16_t x) {
  qaprs._toneSendTime = x;
}

void t(){
//  // Übermittlung eines Pakets des Typs Kommentar
//  packet_buffer = ":TEST TEST TEST de SQ5RWU";
//  // Änderung von Quelladresse und ssid
//  QAPRS.setFromAddress("SQ5R", '1');
//  QAPRS.sendData(packet_buffer);
//  // Übermittlung eines Pakets mit der Position und dem Symbol des Objekts
//  packet_buffer = "!5000.68N/00858.48ES#";
//  // Änderung von Quelladresse, ssid und Pfad
//  QAPRS.setFromAddress("SQ5RWU", '2');
//  QAPRS.setRelays("WIDE2-2");
//  QAPRS.sendData(packet_buffer);
//  // Übermittlung von Wetterdaten ohne Position
//  packet_buffer = "_07071805c025s009g008t030r000p000P000h00b10218";
//  // Pfadwechsel
//  QAPRS.setRelays("WIDE1-1");
//  QAPRS.sendData(packet_buffer);
//  delay(5000);
}
