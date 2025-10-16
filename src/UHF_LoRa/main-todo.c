// STM32F100 and SI4032 RTTY transmitter
// released under GPL v.2 by anonymous developer
// enjoy and have a nice day
// ver 1.5a
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_rcc.h>
#include "stdlib.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <misc.h>
#include <inttypes.h>
#include "f_rtty.h"
#include "init.h"
#include "config.h"
#include "radio.h"
#include "ublox.h"
#include "delay.h"
#include "util.h"
#include "mfsk.h"
#include "horus_l2.h"
#include "morse.h"
#include "cmsis/core_cm3.h"
#include "locator.h"
#include "aprs.h"

// IO Pins Definitions. The state of these pins are initialized in init.c
#define GREEN  GPIO_Pin_7 // Inverted
#define RED  GPIO_Pin_8 // Non-Inverted (?)


// Transmit Modulation Switching
#define STARTUP 0
#define RTTY 1
#define MFSK 2
#define FSK_2 3
#define APRS 4
#define LORA 5
#define GFSK 6

volatile int current_mode = STARTUP;
struct TBinaryPacketV1 BinaryPacket1;
struct TBinaryPacketV2 BinaryPacket2;

// Telemetry Data to Transmit
unsigned int send_count;
int voltage;
int8_t si4032_temperature;
GPSEntry gpsData;

char callsign[15] = {CALLSIGN};
char status[2] = {'N'};
uint16_t CRC_rtty = 0x12ab;
char buf_rtty[300];
char buf_mfsk[300];
char buf_morse[300];

__IO uint16_t ADCVal[2];

// Volatile Variables
volatile int adc_bottom = 2000;
volatile char flaga = 0;
volatile int led_enabled = 1;
volatile unsigned char pun = 0;
volatile unsigned int cun = 10;
volatile unsigned char tx_on = 0;
volatile unsigned int tx_on_delay;
volatile unsigned int tx_on_last_delay_ms = 0;
volatile unsigned int sync_txdelay = 0;
uint8_t freuqency_change = 1;
uint8_t freuqency_change_PIP = 1;
volatile unsigned char tx_enable = 0;
rttyStates send_rtty_status = rttyZero;
volatile char *tx_buffer;
volatile uint16_t packet_length = 0;
volatile uint16_t button_pressed = 0;
volatile uint8_t disable_armed = 0;
volatile uint32_t deep_sleep_timer = 0;
volatile uint8_t entered_psm = 0;
volatile uint8_t reset_pending = 0; // <-- NOVA FLAG PARA RESET SEGURO

// Variáveis para a transmissão GFSK
volatile uint16_t gfsk_current_byte = 0;
volatile int8_t gfsk_current_bit = 7;
uint8_t gfsk_tx_buffer[100];

#ifdef CONTINUOUS_MODE
  volatile uint8_t continuous_mode = 1;
#else
  volatile uint8_t continuous_mode = 0;
#endif

#ifdef TX_PIP
volatile unsigned int tx_pip = TX_PIP / (1000/BAUD_RATE);
#endif

#pragma pack(push,1) 
struct TBinaryPacketV1
{
	uint8_t   PayloadID;
	uint16_t  Counter;
	uint8_t   Hours;
	uint8_t   Minutes;
	uint8_t   Seconds;
	float     Latitude;
	float     Longitude;
	uint16_t  Altitude;
	uint8_t   Speed;
	uint8_t   Sats;
	int8_t    Temp;
	uint8_t   BattVoltage;
	uint16_t  Checksum;
};
#pragma pack(pop)

#pragma pack(push,1)
struct TBinaryPacketV2
{
	uint16_t  PayloadID;
	uint16_t  Counter;
	uint8_t   Hours;
	uint8_t   Minutes;
	uint8_t   Seconds;
	float     Latitude;
	float     Longitude;
	uint16_t  Altitude;
	uint8_t   Speed;
	uint8_t   Sats;
	int8_t    Temp;
	uint8_t   BattVoltage;
#ifdef USERFLAG_A
	int16_t   FlightNumber;
	int16_t   SondeType;
	uint8_t   dummy1;
	uint16_t  dummy2;
	uint16_t  unused;
#else
	int16_t   dummy1;
	int16_t   dummy2;
	uint8_t   dummy3;
	uint16_t  dummy4;
	uint16_t  unused;
#endif
	uint16_t  Checksum;
};
#pragma pack(pop)

uint8_t NOGPS_counter;

void collect_telemetry_data();
void send_rtty_packet();
int prepare_lora_payload(uint8_t* buffer);
#ifdef HORUS_V1
  void send_mfsk_packetV1();
#endif
#ifdef HORUS_V2
  void send_mfsk_packetV2();
#endif
void send_morse_ident();
uint16_t gps_CRC16_checksum (char *string);
void send_gfsk_packet();
void radio_setup_gfsk(uint32_t deviation);

void led_green_on(void) { GPIO_ResetBits(GPIOB, GREEN); }
void led_green_off(void) { GPIO_SetBits(GPIOB, GREEN); }
void led_red_on(void) { GPIO_ResetBits(GPIOB, RED); }
void led_red_off(void) { GPIO_SetBits(GPIOB, RED); }

void USART1_IRQHandler(void) {
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
    ublox_handle_incoming_byte((uint8_t) USART_ReceiveData(USART1));
  } else if (USART_GetITStatus(USART1, USART_IT_ORE) != RESET) {
    USART_ReceiveData(USART1);
  }
}

uint8_t calculate_checksum(uint8_t* data, int length) {
    uint8_t checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void Sync_tx_on_delay (void) {
    ublox_get_last_data(&gpsData);
    if ((gpsData.gpsFixOK == 1) && (gpsData.sats_raw >= 3)) {
        if (sync_txdelay == 0) {
        	uint16_t txdelay_in_seconds = 60 - gpsData.seconds;
			#ifdef TX_DELAY_OFFSET
        		txdelay_in_seconds += (TX_DELAY_OFFSET / 1000);
			#endif
			tx_on_delay = (txdelay_in_seconds) * 100 ;
			tx_on_last_delay_ms = 0;
			sync_txdelay = 1;
        } else {
            uint32_t txdelay_in_ms =  TX_DELAY - 3000;
            tx_on_last_delay_ms += txdelay_in_ms;
        	tx_on_last_delay_ms += 3000;
            if (((tx_on_last_delay_ms % 60000) <= 2000) || ((tx_on_last_delay_ms % 60000) >= 58000)) {
				uint16_t txdelay_in_min =  txdelay_in_ms / 60000;
				#ifdef TX_DELAY_OFFSET
            		tx_on_last_delay_ms = (60 - (gpsData.seconds-(TX_DELAY_OFFSET/1000))) * 1000 + (txdelay_in_min * 60000);
				#else
            		tx_on_last_delay_ms = (60 - gpsData.seconds) * 1000 + (txdelay_in_min * 60000);
				#endif
            	tx_on_last_delay_ms = 0;
            }
			tx_on_delay = (tx_on_last_delay_ms / 1000) * 100;
        }
    } else {
    	tx_on_delay = (TX_DELAY-2000) / (1000/BAUD_RATE);
    	sync_txdelay = 0;
    }
}

void TIM2_IRQHandler(void) {
  static int mfsk_symbol = 0;
  volatile uint16_t current_mfsk_byte = 0;

  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    if (aprs_is_active()){
          aprs_timer_handler();
    } else {
		if (tx_on ) {
#if defined(GFSK_ENABLED)
            if (current_mode == GFSK) {
                if (gfsk_current_byte >= packet_length) {
                    tx_on = 0;
                    tx_enable = 0;
                    radio_disable_tx();
                    Sync_tx_on_delay();
                    init_timer(BAUD_RATE);
                    radio_rw_register(0x71, 0b00000010, 1);
                    return;
                }
                uint8_t bit = (gfsk_tx_buffer[gfsk_current_byte] >> gfsk_current_bit) & 0x01;
                if (bit) {
                    radio_rw_register(0x72, (uint8_t)(GFSK_DEVIATION / 625), 1);
                } else {
                    radio_rw_register(0x72, 0, 1);
                }
                gfsk_current_bit--;
                if (gfsk_current_bit < 0) {
                    gfsk_current_bit = 7;
                    gfsk_current_byte++;
                }
                return;
            }
#endif
#if !defined(GFSK_ENABLED)
            if(current_mode == RTTY){
                send_rtty_status = send_rtty((char *) tx_buffer);
                if (send_rtty_status == rttyEnd) {
                    if (*(++tx_buffer) == 0) {
                      tx_on = 0;
                      tx_enable = 0;
                      radio_disable_tx();
                      Sync_tx_on_delay();
                    }
                } else if (send_rtty_status == rttyOne) {
                    radio_rw_register(0x73, RTTY_DEVIATION, 1);
                } else if (send_rtty_status == rttyZero) {
                    radio_rw_register(0x73, 0x00, 1);
                }
            } else if (current_mode == MFSK) {
                #ifdef MFSK_4_ENABLED
                  mfsk_symbol = send_4fsk(tx_buffer[current_mfsk_byte]);
                #elif MFSK_16_ENABLED
                  mfsk_symbol = send_16fsk(tx_buffer[current_mfsk_byte]);
                #endif
                if(mfsk_symbol == -1){
                    if (current_mfsk_byte++ == packet_length) {
                        radio_rw_register(0x73, 0x03, 1);
                        current_mfsk_byte = 0;
                        tx_on = 0;
                        tx_enable = 0;
                        radio_disable_tx();
                        Sync_tx_on_delay();
                    } else {
                        #ifdef MFSK_4_ENABLED
                          mfsk_symbol = send_4fsk(tx_buffer[current_mfsk_byte]);
                        #elif MFSK_16_ENABLED
                          mfsk_symbol = send_16fsk(tx_buffer[current_mfsk_byte]);
                        #endif
                    }
                }
                if(mfsk_symbol != -1){
                  radio_rw_register(0x73, (uint8_t)mfsk_symbol, 1);
                }
            }
#endif
		}
		if (!tx_on && --tx_on_delay == 0) {
		  tx_enable = 1;
		  tx_on_delay--;
		}
		if (--cun == 0) {
		  pun = !pun;
          if(pun && (flaga & 0x80)) {
              if (led_enabled) led_green_on();
          } else {
              if (led_enabled) led_green_off();
          }
		  cun = 100;
		}
	  }
  }
}

int main(void) {
  RCC_Conf();
  NVIC_Conf();
  init_port();
  init_timer(BAUD_RATE);
  delay_init();
  ublox_init();
  led_red_on();
  led_green_off();

  radio_soft_reset();
  radio_set_tx_frequency(TRANSMIT_FREQUENCY);
  radio_rw_register(0x6D, 00 | (TX_POWER & 0x0007), 1);
  radio_rw_register(0x71, 0x00, 1);
  
  tx_buffer = buf_rtty;
  tx_on = 0;
  tx_enable = 1;

  spi_init();
  radio_set_tx_frequency(TRANSMIT_FREQUENCY);   
  radio_rw_register(0x71, 0x00, 1);

  aprs_init();
  radio_enable_tx();

#ifdef APRS_1200_ENABLED
  uint8_t next_aprs_counter = APRS_RATIO;
#endif
  sync_txdelay = 0;
  while (1) {
    if (tx_on == 0 && tx_enable) {
        if (current_mode == STARTUP){
          collect_telemetry_data();
          led_red_off();
#if defined(GFSK_ENABLED)
          current_mode = GFSK;
          send_gfsk_packet();
#else
          current_mode = RTTY;
          #ifdef RTTY_ENABLED
            send_rtty_packet();
          #endif
#endif
        } 
#if !defined(GFSK_ENABLED)
        else if (current_mode == RTTY){
          current_mode = MFSK;
          #if defined(MFSK_4_ENABLED)
            radio_enable_tx();
			#ifdef HORUS_V1
               send_mfsk_packetV1();
			#endif
			#ifdef HORUS_V2
			   send_mfsk_packetV2();
			#endif
          #endif
        } else if (current_mode == MFSK){
            current_mode = APRS;
			#ifdef APRS_1200_ENABLED
            if (next_aprs_counter-- <= 1) {
                radio_enable_tx();
                USART_Cmd(USART1, DISABLE);
                int8_t temperature = radio_read_temperature();
                uint16_t voltage_aprs = (uint16_t) ADCVal[0] * 600 / 4096;
                aprs_send_position(gpsData, temperature, voltage_aprs);
                USART_Cmd(USART1, ENABLE);
                radio_disable_tx();
                _delay_ms(1000);
                next_aprs_counter = APRS_RATIO;
            }
		    #endif
        }
#endif
        else if (current_mode == APRS || current_mode == GFSK){ 
            current_mode = LORA;
            #ifdef LORA_ENABLED
                 __disable_irq();
                const uint8_t SYNC_WORD = 0xAA;
                uint8_t lora_payload_buffer[32];
                int payload_length = prepare_lora_payload(lora_payload_buffer);
                while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
                USART_SendData(USART3, SYNC_WORD);
                for (int i = 0; i < payload_length; i++) {
                    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
                    USART_SendData(USART3, lora_payload_buffer[i]);
                }
                uint8_t checksum = calculate_checksum(lora_payload_buffer, payload_length);
                while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
                USART_SendData(USART3, checksum);
                __enable_irq();
            #endif
        } else if (current_mode == LORA) {
            current_mode = STARTUP;
            radio_disable_tx();
            #ifdef MORSE_IDENT
              if(send_count%MORSE_IDENT == 0){
                send_morse_ident();
              }
            #endif
            
            // <-- ALTERAÇÃO: VERIFICAÇÃO DE RESET SEGURO
            if (reset_pending) {
                _delay_ms(100); // Pequeno atraso para garantir que a UART tenha terminado
                NVIC_SystemReset();
            }

        } else {
            current_mode = STARTUP;
        }
    } else {
      __WFI();
    }
  }
}

void collect_telemetry_data() {
  send_count++;
  si4032_temperature = radio_read_temperature();
  voltage = ADCVal[0] * 600 / 4096;

  __disable_irq();
  ublox_get_last_data(&gpsData);
  __enable_irq();

  if (gpsData.gpsFixOK == 1) {
	  NOGPS_counter = 0;
      flaga |= 0x80;
      led_enabled = ((gpsData.alt_raw / 1000) > 1000) ? 0 : 1;
  } else {
	#ifdef NOGPS_RESET_AFTER_TXCOUNT
	  NOGPS_counter++;
      // <-- ALTERAÇÃO: LÓGICA DE RESET SEGURO
	  if(NOGPS_counter > NOGPS_RESET_AFTER_TXCOUNT) {
          reset_pending = 1; // Sinaliza que um reset é necessário
      }
	#endif
      flaga &= ~0x80;
      led_enabled = 1;
      gpsData.lat_raw = 0;
      gpsData.lon_raw = 0;
      gpsData.alt_raw = 0;
  }
}

void send_rtty_packet() {
    int n;
    uint8_t lat_d = (uint8_t) abs(gpsData.lat_raw / 10000000);
    uint32_t lat_fl = (uint32_t) abs(abs(gpsData.lat_raw) - lat_d * 10000000) / 1000;
    uint8_t lon_d = (uint8_t) abs(gpsData.lon_raw / 10000000);
    uint32_t lon_fl = (uint32_t) abs(abs(gpsData.lon_raw) - lon_d * 10000000) / 1000;
    uint8_t speed_kph = (uint8_t)((float)gpsData.speed_raw*0.0036);
    uint8_t sats_state = gpsData.sats_raw;
    if(gpsData.psmState == 1) sats_state += 100;
    else if(gpsData.psmState == 2) sats_state += 200;
  
    n = sprintf(buf_rtty,"\n\n\n\n$$$$$%s,%d,%02u:%02u:%02u,%s%d.%04" PRId32 ",%s%d.%04" PRId32 ",%"PRId32",%d,%d,%d,%d",
        callsign, send_count, gpsData.hours, gpsData.minutes, gpsData.seconds,
        gpsData.lat_raw < 0 ? "-" : "", lat_d, lat_fl,
        gpsData.lon_raw < 0 ? "-" : "", lon_d, lon_fl,
        (gpsData.alt_raw / 1000), speed_kph, sats_state, voltage*10, si4032_temperature);
  
    CRC_rtty = string_CRC16_checksum(buf_rtty + 9);
    sprintf(buf_rtty + n, "*%04X\n", CRC_rtty & 0xffff);

    tx_buffer = buf_rtty;
    start_bits = RTTY_PRE_START_BITS;
    radio_enable_tx();
    tx_on = 1;
}

void radio_setup_gfsk(uint32_t deviation) {
    radio_rw_register(0x71, 0b00001000, 1);
}

void send_gfsk_packet() {
    uint8_t telemetry_payload[32];
    int payload_len = prepare_lora_payload(telemetry_payload);

    memset(gfsk_tx_buffer, 0x55, 4);
    gfsk_tx_buffer[4] = 0x2D;
    gfsk_tx_buffer[5] = 0xD4;
    memcpy(&gfsk_tx_buffer[6], telemetry_payload, payload_len);

    packet_length = 6 + payload_len;

    gfsk_current_byte = 0;
    gfsk_current_bit = 7;

    init_timer(GFSK_BAUD_RATE);
    radio_setup_gfsk(GFSK_DEVIATION);

    radio_enable_tx();
    tx_on = 1;
}

int prepare_lora_payload(uint8_t* buffer) {
    typedef struct __attribute__((packed)) {
        uint32_t packet_id;
        int32_t  latitude_raw;
        int32_t  longitude_raw;
        int32_t  altitude_raw;
        uint16_t voltage_mv;
        int8_t   radio_temp_c;
        uint8_t  sats_and_fix;
    } LoRaPayload_t;

    LoRaPayload_t payload;
    payload.packet_id      = send_count;
    payload.latitude_raw   = gpsData.lat_raw;
    payload.longitude_raw  = gpsData.lon_raw;
    payload.altitude_raw   = gpsData.alt_raw;
    payload.voltage_mv     = voltage;
    payload.radio_temp_c   = si4032_temperature;
    payload.sats_and_fix = (gpsData.gpsFixOK & 0x01) << 7;
    payload.sats_and_fix |= (gpsData.sats_raw & 0x7F);

    memcpy(buffer, &payload, sizeof(LoRaPayload_t));
    return sizeof(LoRaPayload_t);
}

#ifdef HORUS_V1
void send_mfsk_packetV1(){
    // ... (código inalterado) ...
}
#endif

#ifdef HORUS_V2
void send_mfsk_packetV2(){
    // ... (código inalterado) ...
}
#endif

void send_morse_ident(){
    // ... (código inalterado) ...
}


