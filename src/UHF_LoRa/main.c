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

// If enabled, print out binary packets as hex before and after coding.
//#define MFSKDEBUG 1

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

volatile int current_mode = STARTUP;
struct TBinaryPacketV1 BinaryPacket1;
struct TBinaryPacketV2 BinaryPacket2;

// Telemetry Data to Transmit - used in RTTY & MFSK packet generation functions.
unsigned int send_count;        //frame counter
int voltage;
int8_t si4032_temperature;
GPSEntry gpsData;

char callsign[15] = {CALLSIGN};
char status[2] = {'N'};
uint16_t CRC_rtty = 0x12ab;  //checksum (dummy initial value)
char buf_rtty[300];
char buf_mfsk[300];
char buf_morse[300];

__IO uint16_t ADCVal[2];

// Volatile Variables, used within interrupts.
volatile int adc_bottom = 2000;
volatile char flaga = 0; // GPS Status Flags
volatile int led_enabled = 1; // Flag to disable LEDs at altitude.

volatile unsigned char pun = 0;
volatile unsigned int cun = 10;
volatile unsigned char tx_on = 0;
volatile unsigned int tx_on_delay;
volatile unsigned int tx_on_last_delay_ms = 0;
volatile unsigned int sync_txdelay = 0;
char dummy[50] = "START *F00\n";
uint8_t freuqency_change = 1;
uint8_t freuqency_change_PIP = 1;

volatile unsigned char tx_enable = 0;
rttyStates send_rtty_status = rttyZero;
volatile char *tx_buffer;
volatile uint16_t current_mfsk_byte = 0;
volatile uint16_t packet_length = 0;
volatile uint16_t button_pressed = 0;
volatile uint8_t disable_armed = 0;

volatile uint32_t deep_sleep_timer = 0;
volatile uint8_t entered_psm = 0;
volatile uint8_t reset_pending = 0;

#ifdef CONTINUOUS_MODE
  volatile uint8_t continuous_mode = 1;
#else
  volatile uint8_t continuous_mode = 0;
#endif

#ifdef TX_PIP
volatile unsigned int tx_pip = TX_PIP / (1000/BAUD_RATE);
#endif

// Binary Packet Format
// Note that we need to pack this to 1-byte alignment, hence the #pragma flags below
// Refer: https://gcc.gnu.org/onlinedocs/gcc-4.4.4/gcc/Structure_002dPacking-Pragmas.html
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
	uint8_t   Speed; // Speed in Knots (1-255 knots)
	uint8_t   Sats;
	int8_t    Temp; // Si4032 temperature, as a signed value (-128 to +128, though sensor limited to -64 to +64 deg C)
	uint8_t   BattVoltage; // 0 = 0v, 255 = 5.0V, linear steps in-between.
	uint16_t  Checksum; // CRC16-CCITT Checksum.
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
	uint8_t   Speed; // Speed in Knots (1-255 knots)
	uint8_t   Sats;
	int8_t    Temp; // Si4032 temperature, as a signed value (-128 to +128, though sensor limited to -64 to +64 deg C)
	uint8_t   BattVoltage; // 0 = 0v, 255 = 5.0V, linear steps in-between.
	//... extra payload infos 9 bytes
#ifdef USERFLAG_A
	int16_t   FlightNumber;     // Using this, request to Mark add custom_field_list.json block same as DF7PN
	int16_t   SondeType;        // Using this, request to Mark add custom_field_list.json block same as DF7PN
	uint8_t   dummy1;       // not used
	uint16_t  dummy2;       // not used
	uint16_t  unused;       // not used
#else
	int16_t   dummy1;       // Interpreted as Ascent rate divided by 100 for the Payload ID: 4FSKTEST-V2
	int16_t   dummy2;       // External temperature divided by 10 for the Payload ID: 4FSKTEST-V2
	uint8_t   dummy3;       // External humidity for the Payload ID: 4FSKTEST-V2
	uint16_t  dummy4;       // External pressure divided by 10 for the Payload ID:  4FSKTEST-V2
	uint16_t  unused;       // 2 bytes which are not interpreted
#endif
	uint16_t  Checksum; // CRC16-CCITT Checksum.
};
#pragma pack(pop)

uint8_t NOGPS_counter;
// Function Definitions
void collect_telemetry_data();
void send_rtty_packet();
int prepare_lora_payload(uint8_t* buffer);
#ifdef HORUS_V1
  void send_mfsk_packetV1();
#endif
#ifdef HORUS_V2
  void send_mfsk_packetV2();
#endif
// Forward declaration
void send_morse_ident();
uint16_t gps_CRC16_checksum (char *string);

/**
 * Switch GREEN LED on
 */
void led_green_on(void) {
	// switch is inverted
	GPIO_ResetBits(GPIOB, GREEN);
}

/**
 * Switch GREEN LED off
 */
void led_green_off(void) {
	// switch is inverted
	GPIO_SetBits(GPIOB, GREEN);
}

/**
 * Switch RED LED on
 */
void led_red_on(void) {
	// switch is inverted
	GPIO_ResetBits(GPIOB, RED);
}

/**
 * Switch RED LED off
 */
void led_red_off(void) {
	// switch is inverted
	GPIO_SetBits(GPIOB, RED);
}




/**
 * GPS data processing
 */
void USART1_IRQHandler(void) {
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
    ublox_handle_incoming_byte((uint8_t) USART_ReceiveData(USART1));
      }else if (USART_GetITStatus(USART1, USART_IT_ORE) != RESET) {
    USART_ReceiveData(USART1);
  } else {
    USART_ReceiveData(USART1);
  }
}

uint8_t calculate_checksum(uint8_t* data, int length) {
    uint8_t checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum ^= data[i]; // XOR each byte with the checksum
    }
    return checksum;
}

/**
 * Sync TX-Delay to GPS
 */
void Sync_tx_on_delay (void) {
#ifdef SYNC_TX_WITH_GPS
    uint16_t txdelay_in_seconds = 0;
    uint16_t txdelay_in_ticks = 0;  // 1/100 second
    // handle all in milliseconds (ms) until set "tx_on_delay" for timer ticks
    //tx_on_delay = (TX_DELAY-3100) / (1000/BAUD_RATE);
//    float tx_delay_dec = TX_DELAY / 60000;
    // get newest status
    ublox_get_last_data(&gpsData);

    if ((gpsData.gpsFixOK == 1) && (gpsData.sats_raw >= 3)) {
        if (sync_txdelay == 0) {  // if still unsynced ...
        	// how many seconds up to next full minute?
        	txdelay_in_seconds = 60 - gpsData.seconds;
			#ifdef TX_DELAY_OFFSET
        		txdelay_in_seconds += (TX_DELAY_OFFSET / 1000);
			#endif
        	// if lower then 10s to next transmit then wait for miniute after this interval
        	//if (txdelay_in_seconds < 10) txdelay_in_seconds =+ 60;

        	txdelay_in_ticks = (txdelay_in_seconds) * 100 ;
			tx_on_delay = txdelay_in_ticks ;
            // internal reference without OFFSET
			tx_on_last_delay_ms = 0;
			sync_txdelay = 1;
        } else {
        	// already synced to full minute plus OFFSET
            uint32_t txdelay_in_ms =  TX_DELAY - 3000;   // in ms defined in config.h #94
            uint16_t txdelay_in_min =  txdelay_in_ms / 60000;
            // internal counter add time to next TX
            tx_on_last_delay_ms += txdelay_in_ms;
        	// at this point we already have 3 sec sent - so TXDELAY - 3000 ms left for next TX point
        	tx_on_last_delay_ms += 3000;
            //  If near a full minute - correct it to this full minute
            if (((tx_on_last_delay_ms % 60000) <= 2000) || ((tx_on_last_delay_ms % 60000) >= 58000)) {
            	// trim next interval exact to a full minute
				#ifdef TX_DELAY_OFFSET
            		tx_on_last_delay_ms = (60 - (gpsData.seconds-(TX_DELAY_OFFSET/1000))) * 1000 + (txdelay_in_min * 60000);
				#else
            		tx_on_last_delay_ms = (60 - gpsData.seconds) * 1000 + (txdelay_in_min * 60000);
				#endif
          		txdelay_in_seconds = tx_on_last_delay_ms / 1000;
            	tx_on_last_delay_ms = 0;
            } else {
            	txdelay_in_seconds = txdelay_in_ms / 1000;
            	// tx_on_last_delay_ms = tx_on_last_delay_ms % 60000;
            }

            txdelay_in_ticks = txdelay_in_seconds * 100 ;
			tx_on_delay = txdelay_in_ticks ;
        }
    } else {
    	// without gps fix calculate the old way
    	tx_on_delay = (TX_DELAY-2000) / (1000/BAUD_RATE);
    	sync_txdelay = 0;  // start sync again if fix is ok
    }
#endif
#ifndef SYNC_TX_WITH_GPS
	tx_on_delay = (TX_DELAY) / (1000/BAUD_RATE);
#endif

}


//
// Symbol Timing Interrupt
// In here symbol transmission occurs.
//

void TIM2_IRQHandler(void) {
  static int mfsk_symbol = 0;

  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    // decide if aprs OR all other other modes
    if (aprs_is_active()){
          aprs_timer_handler();
    } else {
    	// all other modes to check
		if (ALLOW_DISABLE_BY_BUTTON){
		  if (ADCVal[1] > adc_bottom){
			button_pressed++;
			if (button_pressed > (BAUD_RATE / 3)){
			  disable_armed = 1;
			  led_red_on();
			}
		  } else {
			if (disable_armed){
			  GPIO_SetBits(GPIOA, GPIO_Pin_12);
			}
			button_pressed = 0;
		  }

		  if (button_pressed == 0) {
			adc_bottom = ADCVal[1] * 1.1; // dynamical reference for power down level
		  }
		}

		if (tx_on ) {
		  // RTTY Symbol selection logic.
		  if(current_mode == RTTY){
			send_rtty_status = send_rtty((char *) tx_buffer);

			if (!disable_armed){
			  if (send_rtty_status == rttyEnd) {
				if (led_enabled) { led_red_on(); }
				if (*(++tx_buffer) == 0) {
				  tx_on = 0;
				  // Reset the TX Delay counter, which is decremented at the symbol rate.
				  // tx_on_delay = (TX_DELAY-5000) / (1000/BAUD_RATE);
				  Sync_tx_on_delay();
				  tx_enable = 0;

				  // If we're not in continuous mode, disable the transmitter now.
				  #ifndef CONTINUOUS_MODE
					radio_disable_tx();
				  #endif
				}
			  } else if (send_rtty_status == rttyOne) {
				radio_rw_register(0x73, RTTY_DEVIATION, 1);
				if (led_enabled) led_red_on();
			  } else if (send_rtty_status == rttyZero) {
				radio_rw_register(0x73, 0x00, 1);
				if (led_enabled) led_red_off();
			  }
			}
		  } else if (current_mode == MFSK) {
			// 4FSK Symbol Selection Logic
			// Get Symbol to transmit.
			#ifdef MFSK_4_ENABLED
			  mfsk_symbol = send_4fsk(tx_buffer[current_mfsk_byte]);
			#elif MFSK_16_ENABLED
			  mfsk_symbol = send_16fsk(tx_buffer[current_mfsk_byte]);
			#endif

			if(mfsk_symbol == -1){
			  // Reached the end of the current character, increment the current-byte pointer.
				if (current_mfsk_byte++ == packet_length) {
					// End of the packet. Reset Counters and stop modulation.
					radio_rw_register(0x73, 0x03, 1); // Idle at Symbol 3
					current_mfsk_byte = 0;
					tx_on = 0;
					// Reset the TX Delay counter, which is decremented at the symbol rate.
					// Set next delay to TX
					Sync_tx_on_delay();
					tx_enable = 0;

					// If we're not in continuous mode, disable the transmitter now.
					#ifndef CONTINUOUS_MODE
					  radio_disable_tx();
					#endif


				} else {
					// We've now advanced to the next byte, grab the first symbol from it.
					#ifdef MFSK_4_ENABLED
					  mfsk_symbol = send_4fsk(tx_buffer[current_mfsk_byte]);
					#elif MFSK_16_ENABLED
					  mfsk_symbol = send_16fsk(tx_buffer[current_mfsk_byte]);
					#endif
				}
			}
			// Set the symbol!
			if(mfsk_symbol != -1){
			  radio_rw_register(0x73, (uint8_t)mfsk_symbol, 1);
			}


		  } else if (current_mode == FSK_2) {
			// 2FSK Symbol Selection Logic
			// Get Symbol to transmit.
			mfsk_symbol = send_2fsk(tx_buffer[current_mfsk_byte]);

			if(mfsk_symbol == -1){
			  // Reached the end of the current character, increment the current-byte pointer.
			  if (current_mfsk_byte++ == packet_length) {
				  // End of the packet. Reset Counters and stop modulation.
				  radio_rw_register(0x73, 0x00, 1); // Idle at Symbol 0.
				  current_mfsk_byte = 0;
				  tx_on = 0;
				  // Reset the TX Delay counter, which is decremented at the symbol rate.
				  // tx_on_delay = (TX_DELAY-3500) / (1000/BAUD_RATE);
				  Sync_tx_on_delay();
				  tx_enable = 0;

			  } else {
				// We've now advanced to the next byte, grab the first symbol from it.
				mfsk_symbol = send_2fsk(tx_buffer[current_mfsk_byte]);
			  }
			}
			// Set the symbol!
			if(mfsk_symbol != -1){
			  radio_rw_register(0x73, (uint8_t)mfsk_symbol, 1);
			}
		  } else if (current_mode == APRS) {   // <----- APRS


			  /*
			  while (aprs_is_active()){
			      aprs_timer_handler();
			    }
			  tx_on = 0;
		      Sync_tx_on_delay();
			  tx_enable = 0;
			  */
		  } else {
			// No more modes until now
		  }
		} else {
		  // TX is off
		  // If we are don't have RTTY enabled, and if we have CONTINUOUS_MODE set,
		  // transmit continuous MFSK symbols.
		  #ifndef RTTY_ENABLED
			if(continuous_mode){
			  #ifdef MFSK_4_ENABLED
				mfsk_symbol = (mfsk_symbol+1)%4;
			  #elif MFSK_16_ENABLED
				mfsk_symbol = (mfsk_symbol+1)%16;
			  #endif
			  radio_rw_register(0x73, (uint8_t)mfsk_symbol, 1);
			}
		  #endif


		}

		// Delay between Transmissions Logic.
		// tx_on_delay is set at the end of a RTTY/MFSK transmission above, and counts down
		// at the interrupt rate. When it hits zero, we set tx_enable to 1, which allows
		// the main loop to continue.
		if (!tx_on && --tx_on_delay == 0) {
		  tx_enable = 1;
		  tx_on_delay--;
		}

		// Pip transmission logic.
		// Only enabled if Continuous mode is disabled!
		#ifdef TX_PIP
		#ifndef CONTINUOUS_MODE
		  if ((tx_enable == 0) && (tx_on_delay%tx_pip)==TX_PIP_SYMBOLS){
			// if alternating frequency is activ, every PIP will switch between also each PIP Interval

			#ifdef TRANSMIT_FREQUENCY_2ND
			freuqency_change_PIP = !freuqency_change_PIP;
			// setting TX frequency
			if (freuqency_change_PIP) {
				radio_set_tx_frequency(TRANSMIT_FREQUENCY_2ND);
			} else {
				radio_set_tx_frequency(TRANSMIT_FREQUENCY);
			}
			#endif

			radio_rw_register(0x73, 0x00, 1);
			radio_enable_tx();
		  } else if ((tx_enable == 0) && (tx_on_delay%tx_pip)==0){
			  radio_disable_tx();
			}
		#endif  // not CONTINUOUS_MODE
		#endif  // TX_PIP active

		// Green LED Blinking Logic
		if (--cun == 0) {
		  if (pun) {
			// Clear Green LED.
			if (led_enabled) led_green_off();
			pun = 0;
		  } else {
			// If we have GPS lock, set LED
			if (flaga & 0x80) {
			  if (led_enabled) led_green_on();
			}
			pun = 1;
		  }
		  // Wait 200 symbols.
		  cun = 100;
		}
	  }
  }
}

int main(void) {
#ifdef DEBUG
  debug();
#endif
  RCC_Conf();
  NVIC_Conf();
  init_port();

  init_timer(BAUD_RATE);

  delay_init();
  ublox_init();

  led_red_on();
  led_green_off();
  //USART_SendData(USART3,'C');

  radio_soft_reset();
  // setting TX frequency
  radio_set_tx_frequency(TRANSMIT_FREQUENCY);

  // setting TX power
  radio_rw_register(0x6D, 00 | (TX_POWER & 0x0007), 1);

  // initial RTTY modulation
  radio_rw_register(0x71, 0x00, 1);

  // Temperature Value Offset
  radio_rw_register(0x13, 0x00, 1); // Was 0xF0 (?)

  // Temperature Sensor Calibration
  radio_rw_register(0x12, 0x20, 1);

  // ADC configuration
  radio_rw_register(0x0f, 0x80, 1);
  tx_buffer = buf_rtty;
  tx_on = 0;
  tx_enable = 1;

  // Why do we have to do this again?
  spi_init();
  radio_set_tx_frequency(TRANSMIT_FREQUENCY);   
  radio_rw_register(0x71, 0x00, 1);
  init_timer(BAUD_RATE);



  // WARNING WARNING WARNING
  // As per the Si4032 datasheet, the synthesizer's VCO is only calibrated when it is enabled,
  // not continuously throughout transmissions. If it is enabled, and there is a significant temperature change,
  // the transmitter *will* drift off frequency.
  // The fix appears to be to briefly disable, then re-enable the transmitter, which forces a re-calibration.
  aprs_init();
  radio_enable_tx();

#ifdef APRS_1200_ENABLED
  uint8_t next_aprs_counter = APRS_RATIO;
#endif
  sync_txdelay = 0;
  while (1) {
    // Don't do anything until the previous transmission has finished.
    if (tx_on == 0 && tx_enable) {
      
        // ===================================================================
        // NEW: SAFE RESET HANDLER
        // ===================================================================
        // If a reset has been requested, handle it now before starting
        // a new, pointless transmission cycle.
        if (reset_pending) {
            radio_disable_tx(); // Ensure the radio is off
            _delay_ms(100);     // Brief delay for stability and to let I/O finish
            NVIC_SystemReset(); // Perform the clean reset
        }
        // ===================================================================

        if (current_mode == STARTUP){
          // Grab telemetry information.
          collect_telemetry_data();
          led_red_off();
          //USART_SendData(USART3,'C');

          // Now Startup a RTTY Transmission
          current_mode = RTTY;
          // If enabled, transmit a RTTY packet.
          #ifdef RTTY_ENABLED
            send_rtty_packet();
          #endif

        } else if (current_mode == RTTY){
          // We've just transmitted a RTTY packet, now configure for 4FSK.
          current_mode = MFSK;


          #if defined(MFSK_4_ENABLED)
            radio_enable_tx();
			#ifdef TRANSMIT_FREQUENCY_2ND
				freuqency_change = !freuqency_change;
				// setting TX frequency
				if (freuqency_change) {
					radio_set_tx_frequency(TRANSMIT_FREQUENCY_2ND);
				} else {
					radio_set_tx_frequency(TRANSMIT_FREQUENCY);
				}
			#endif

			#ifdef HORUS_V1
               send_mfsk_packetV1();
			#endif
			#ifdef HORUS_V2
			   send_mfsk_packetV2();
			#endif

          #endif
        } else if (current_mode == MFSK){
            // We've just transmitted a MFSK packet, now configure for APRS
            current_mode = APRS;

			#ifdef APRS_1200_ENABLED
            if (next_aprs_counter-- <= 1) {
                radio_enable_tx();
		        GPSEntry gpsData;
		        ublox_get_last_data(&gpsData);
		        USART_Cmd(USART1, DISABLE);
		        int8_t temperature = radio_read_temperature();
		        uint16_t voltage = (uint16_t) ADCVal[0] * 600 / 4096;
		        aprs_send_position(gpsData, temperature, voltage);
		        USART_Cmd(USART1, ENABLE);
		        radio_disable_tx();
		        _delay_ms(1000);
		        next_aprs_counter = APRS_RATIO;
            }
		    #endif

        }         // ===================================================================
        // NEW LORA STATE LOGIC
        // ===================================================================
        else if (current_mode == APRS){ // <-- MODIFIED: This was the old "else" block
            // We've just finished the APRS part of the cycle.
            // Now, prepare and send the redundant LoRa packet.
            current_mode = LORA;

            #ifdef LORA_ENABLED
                const uint8_t SYNC_WORD = 0xAA;
                uint8_t lora_payload_buffer[32];
                int payload_length = prepare_lora_payload(lora_payload_buffer);

                // --- INÍCIO DA SEÇÃO CRÍTICA ---
                // Desabilita TODAS as interrupções que podem interferir durante a transmissão.
                 __disable_irq();

                // 1. ENVIA O SYNC WORD
                while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
                USART_SendData(USART3, SYNC_WORD);

                // 2. ENVIA O PAYLOAD
                for (int i = 0; i < payload_length; i++) {
                    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
                    USART_SendData(USART3, lora_payload_buffer[i]);
                }

                // 3. CALCULA E ENVIA O CHECKSUM
                uint8_t checksum = calculate_checksum(lora_payload_buffer, payload_length);
                while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
                USART_SendData(USART3, checksum);

                // --- FIM DA SEÇÃO CRÍTICA ---
                // Reabilita globalmente as interrupções.
                __enable_irq();

            #endif

            // The original logic from the old "else" block now moves to the LORA state handler,
            // or you can handle the transition to STARTUP there.
            // Let's transition to LORA state, and LORA state will transition back to STARTUP.
            // The logic from the old "else" block should be placed after the LORA transmission.

        } else if (current_mode == LORA) {
            // We have finished all transmissions for this cycle (RTTY, MFSK, APRS, LORA).
            // Now, perform end-of-cycle tasks and reset for the next one.
            current_mode = STARTUP;
            radio_disable_tx();


            #ifdef MORSE_IDENT
              if(send_count%MORSE_IDENT == 0){
                send_morse_ident();
              }
            #endif

          #ifdef DEEP_SLEEP
          // Deep Sleep mode!

          // Only enter deep sleep mode if we have a valid GPS lock and position, and we are
          // tracking a decent amount of sats.
          if (gpsData.gpsFixOK == 1 && gpsData.sats_raw >=6){
            // Turn off Green LED
            led_green_off();
            led_enabled = 0;

            // Pause the GPS
            ublox_gps_stop();

            gpsData.lat_raw = 0;
            gpsData.lon_raw = 0;
            gpsData.fix = 0;

            deep_sleep_timer = DEEP_SLEEP*60*1000;
            while(deep_sleep_timer > 0){
              // Turn off LED
              led_enabled = 0;
              // User lowest tone for each pip.
              radio_rw_register(0x73, 0x00, 1);
              // Send a pip
              if( (deep_sleep_timer % DEEP_SLEEP_PIPS) == 0){
                radio_enable_tx();
                _delay_ms(50);
                radio_disable_tx();
              }

              // Sleep
              // How do we make this a non-busy-wait sleep?
              _delay_ms(1000);
              deep_sleep_timer = deep_sleep_timer - 1000;
            }
            // Restart the GPS
            ublox_gps_start();
            _delay_ms(1000);

            // Wait for GPS lock
            while(1){
              ublox_get_last_data(&gpsData);
              if(gpsData.gpsFixOK == 1){
                radio_enable_tx();
                _delay_ms(1000);
                break;
              }
              // Double pip to indicate we are awaiting GPS lock.
              radio_enable_tx();
              _delay_ms(20);
              radio_disable_tx();
              _delay_ms(50);
              radio_enable_tx();
              _delay_ms(20);
              radio_disable_tx();
              _delay_ms(2000);
            }
          }

          #endif

        }else {
            // This case should ideally not be reached if the state machine is correct,
            // but as a fallback, we reset to STARTUP.
            current_mode = STARTUP;
        }
    } else {
      NVIC_SystemLPConfig(NVIC_LP_SEVONPEND, DISABLE);
      __WFI();
    }
  }
}


void collect_telemetry_data() {
  // Assemble and proccess the telemetry data we need to construct our RTTY and MFSK packets.
  send_count++;
  si4032_temperature = radio_read_temperature();
  voltage = ADCVal[0] * 600 / 4096;
  ublox_get_last_data(&gpsData);

  if (gpsData.gpsFixOK == 1) {
	  NOGPS_counter = 0;
      // If we have a good fix, we can enter power-saving mode
      #ifdef UBLOX_POWERSAVE
        if ((gpsData.sats_raw >= 6) && (entered_psm == 0)){
          ubx_powersave();
          entered_psm = 1;
        }
      #endif
      flaga |= 0x80;
      // Disable LEDs if altitude is > 1000m. (Power saving? Maybe?)
      if ((gpsData.alt_raw / 1000) > 1000){
        led_enabled = 0;
      } else {
        led_enabled = 1;
      }
  } else {
      // No GPS fix.
	#ifdef NOGPS_RESET_AFTER_TXCOUNT
      NOGPS_counter++;
      if(NOGPS_counter > NOGPS_RESET_AFTER_TXCOUNT) {
          // NVIC_SystemReset(); // <-- REMOVE THIS DANGEROUS DIRECT CALL
          reset_pending = 1;      // <-- REPLACE IT WITH THE FLAG
      }
	#endif
      flaga &= ~0x80;
      led_enabled = 1; // Enable LEDs when there is no GPS fix (i.e. during startup)

      // Null out lat / lon data to avoid spamming invalid positions all over the map.
      gpsData.lat_raw = 0;
      gpsData.lon_raw = 0;
      gpsData.alt_raw = 0;
  }
}


void send_rtty_packet() {
  int n;
  // Write a RTTY packet into the tx buffer, and start transmission.

  // Convert raw lat/lon values into degrees and decimal degree values.
  uint8_t lat_d = (uint8_t) abs(gpsData.lat_raw / 10000000);
  uint32_t lat_fl = (uint32_t) abs(abs(gpsData.lat_raw) - lat_d * 10000000) / 1000;
  uint8_t lon_d = (uint8_t) abs(gpsData.lon_raw / 10000000);
  uint32_t lon_fl = (uint32_t) abs(abs(gpsData.lon_raw) - lon_d * 10000000) / 1000;

  uint8_t speed_kph = (uint8_t)((float)gpsData.speed_raw*0.0036);

  // Add onto the sats_raw value to indicate if the GPS is in regular tracking (+100)
  // or power optimized tracker (+200) modes.
  uint8_t sats_state = gpsData.sats_raw;
  if(gpsData.psmState == 1){
    sats_state += 100;
  } else if(gpsData.psmState == 2){
    sats_state += 200;
  }
 
  // Produce a RTTY Sentence (Compatible with the existing HORUS RTTY payloads)
  
  n = sprintf(buf_rtty,"\n\n\n\n$$$$$%s,%d,%02u:%02u:%02u,%s%d.%04" PRId32 ",%s%d.%04" PRId32 ",%"PRId32",%d,%d,%d,%d",
        callsign,
        send_count,
        gpsData.hours, gpsData.minutes, gpsData.seconds,
        gpsData.lat_raw < 0 ? "-" : "", lat_d, lat_fl,
        gpsData.lon_raw < 0 ? "-" : "", lon_d, lon_fl,
        (gpsData.alt_raw / 1000),
        speed_kph,
        sats_state,
        voltage*10,
        si4032_temperature
        );
  
  // Calculate and append CRC16 checksum to end of sentence.
  CRC_rtty = string_CRC16_checksum(buf_rtty + 9);
  sprintf(buf_rtty + n, "*%04X\n", CRC_rtty & 0xffff);

  // Point the TX buffer at the temporary RTTY packet buffer.
  tx_buffer = buf_rtty;

  // Enable the radio, and set the tx_on flag to 1.
  start_bits = RTTY_PRE_START_BITS;
  radio_enable_tx();
  tx_on = 1;
  // From here the timer interrupt handles things.
}

//------------------ LoRa Redundancy --------------------------------------
int prepare_lora_payload(uint8_t* buffer) {

    // Define a packed struct for the LoRa payload.
    // `__attribute__((packed))` is crucial. It tells the compiler not to add padding bytes,
    // ensuring the struct's memory layout is predictable for serialization.
    typedef struct __attribute__((packed)) {
        uint32_t packet_id;      // From send_count
        int32_t  latitude_raw;   // From gpsData.lat_raw
        int32_t  longitude_raw;  // From gpsData.lon_raw
        int32_t  altitude_raw;   // From gpsData.alt_raw
        uint16_t voltage_mv;     // From voltage
        int8_t   radio_temp_c;   // From si4032_temperature
        uint8_t  sats_and_fix;   // A combined field for satellite count and fix status
    } LoRaPayload_t;

    // Declare an instance of the payload struct.
    LoRaPayload_t payload;

    // Populate the payload struct with the global telemetry data.
    payload.packet_id      = send_count;
    payload.latitude_raw   = gpsData.lat_raw;
    payload.longitude_raw  = gpsData.lon_raw;
    payload.altitude_raw   = gpsData.alt_raw;
    payload.voltage_mv     = voltage;
    payload.radio_temp_c   = si4032_temperature;

    // Combine satellite count and fix status into a single byte to save space.
    // Bit 7: GPS Fix OK (1 = OK, 0 = No Fix)
    // Bits 6-0: Satellite Count
    payload.sats_and_fix = (gpsData.gpsFixOK & 0x01) << 7; // Put fix status in the most significant bit.
    payload.sats_and_fix |= (gpsData.sats_raw & 0x7F);   // Put sat count in the lower 7 bits.

    // Copy the raw bytes of the struct into the output buffer.
    memcpy(buffer, &payload, sizeof(LoRaPayload_t));

    // Return the size of the payload, which is exactly the size of our struct.
    return sizeof(LoRaPayload_t);
}


//------------------ HORUS V1 --------------------------------------
#ifdef HORUS_V1

void send_mfsk_packetV1(){
  // Generate a MFSK Binary Packet
  //packet_length = mfsk_test_bits(buf_mfsk);

  // Sanitize and convert some of the data.
  if(gpsData.alt_raw < 0){
    gpsData.alt_raw = 0;
  }
  float float_lat = (float)gpsData.lat_raw / 10000000.0;
  float float_lon = (float)gpsData.lon_raw / 10000000.0;

  uint8_t volts_scaled = (uint8_t)(255*(float)voltage/500.0);

  // Assemble a binary packet
  // Global defined: struct TBinaryPacketV1 BinaryPacket1;
  BinaryPacket1.PayloadID = BINARY_PAYLOAD_ID%256;
  BinaryPacket1.Counter = send_count;
  BinaryPacket1.Hours = gpsData.hours;
  BinaryPacket1.Minutes = gpsData.minutes;
  BinaryPacket1.Seconds = gpsData.seconds;
  BinaryPacket1.Latitude = float_lat;
  BinaryPacket1.Longitude = float_lon;
  BinaryPacket1.Altitude = (uint16_t)(gpsData.alt_raw/1000);
  BinaryPacket1.Speed = (uint8_t)((float)gpsData.speed_raw*0.036); // Using NAV-VELNED gSpeed, which is in cm/s. Convert to kph.

  // Temporary pDOP info, to determine suitable pDOP limits.
  // float pDop = (float)gpsData.pDOP/10.0;
  // if (pDop>255.0){
  //  pDop = 255.0;
  // }
  // BinaryPacket.Speed = (uint8_t)pDop;
  BinaryPacket1.BattVoltage = volts_scaled;
  BinaryPacket1.Sats = gpsData.sats_raw;
  BinaryPacket1.Temp = si4032_temperature;

  // Add onto the sats_raw value to indicate if the GPS is in regular tracking (+100)
  // or power optimized tracker (+200) modes.
  if(gpsData.psmState == 1){
    BinaryPacket1.Sats += 100;
  } else if(gpsData.psmState == 2){
    BinaryPacket1.Sats += 200;
  }

  BinaryPacket1.Checksum = (uint16_t)array_CRC16_checksum((char*)&BinaryPacket1,sizeof(BinaryPacket1)-2);

#ifdef MFSKDEBUG
  // Write BinaryPacket into the RTTY transmit buffer as hex
  memcpy(buf_mfsk,&BinaryPacket1,sizeof(struct TBinaryPacketV1));
  sprintf(buf_rtty,"$$$$");
  print_hex(buf_mfsk, 40, buf_rtty+4);
  // sample: $$$$0001000000000000000000000000000000001c02*309D (-12.2 dB SNR)
  CRC_rtty = string_CRC16_checksum(buf_rtty + 4);
  sprintf(buf_rtty + 4 + 40, "*%04X\n", CRC_rtty & 0xffff);

  //Configure for transmit
  tx_buffer = buf_rtty;
  // Enable the radio, and set the tx_on flag to 1.
  // RTTY SHIFT 810 Hz
  start_bits = RTTY_PRE_START_BITS;
  radio_enable_tx();
  current_mode = RTTY;
  tx_on = 1;

  // Wait until transmit has finished.
  while(tx_on){
    NVIC_SystemLPConfig(NVIC_LP_SEVONPEND, DISABLE);
    __WFI();
  }
  _delay_ms(1000);
  current_mode = MFSK;
#endif

  #ifdef CONTINUOUS_MODE
    // Write Preamble characters into mfsk buffer.
    sprintf(buf_mfsk, "\x1b\x1b\x1b\x1b");
    // Encode the packet, and write into the mfsk buffer.
    int coded_len = horus_l2_encode_tx_packet((unsigned char*)buf_mfsk+4,(unsigned char*)&BinaryPacket1,sizeof(BinaryPacket1));
  #else
    // Double length preamble to help the decoder lock-on after a quiet period.
    // Write Preamble characters into mfsk buffer.
    sprintf(buf_mfsk, "\x1b\x1b\x1b\x1b\x1b\x1b\x1b\x1b");
    // Encode the packet, and write into the mfsk buffer.
    int coded_len = horus_l2_encode_tx_packet((unsigned char*)buf_mfsk+8,(unsigned char*)&BinaryPacket1,sizeof(BinaryPacket1));
  #endif

#ifdef MFSKDEBUG_
  // Write the coded packet into the RTTY transmit buffer as hex
  sprintf(buf_rtty,"$$$$");
  print_hex(buf_mfsk+8, coded_len, buf_rtty+4);
  CRC_rtty = string_CRC16_checksum(buf_rtty + 4);
  sprintf(buf_rtty + 4 + coded_len, "_%d*%04X\n", coded_len, CRC_rtty & 0xffff);
  // $$$1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b1b2424c863122d041dc5c99b12ffc7c_45*F4DB (-12.0 dB SNR)
  //Configure for transmit
  tx_buffer = buf_rtty;
  // Enable the radio, and set the tx_on flag to 1.
  start_bits = RTTY_PRE_START_BITS;
  // RTTY SHIFT 810 Hz
  radio_enable_tx();
  current_mode = RTTY;
  tx_on = 1;

  // Wait until transmit has finished.
  while(tx_on){
    NVIC_SystemLPConfig(NVIC_LP_SEVONPEND, DISABLE);
    __WFI();
  }
  current_mode = MFSK;
  // Wait until tx_enable
  while(tx_enable == 0){
    NVIC_SystemLPConfig(NVIC_LP_SEVONPEND, DISABLE);
    __WFI();
  }
  _delay_ms(1000);
  #endif

  // Data to transmit is the coded packet length, plus the preamble.
  #ifdef CONTINUOUS_MODE
    packet_length = coded_len+4;
  #else
    packet_length = coded_len+8;
  #endif

  tx_buffer = buf_mfsk;

  // Enable the radio, and set the tx_on flag to 1.
  radio_enable_tx();
  tx_on = 1;
}
#endif

#ifdef HORUS_V2
//================== HORUS V2 =================================
void send_mfsk_packetV2(){
  // Generate a MFSK Binary Packet

  // Sanitize and convert some of the data.
  if(gpsData.alt_raw < 0){
    gpsData.alt_raw = 0;
  }
  float float_lat = (float)gpsData.lat_raw / 10000000.0;
  float float_lon = (float)gpsData.lon_raw / 10000000.0;

  uint8_t volts_scaled = (uint8_t)(255*(float)voltage/500.0);

  // Assemble a binary packet
  struct TBinaryPacketV2 BinaryPacket2;
  BinaryPacket2.PayloadID = BINARY_PAYLOAD_ID;
  BinaryPacket2.Counter = send_count;
  BinaryPacket2.Hours = gpsData.hours;
  BinaryPacket2.Minutes = gpsData.minutes;
  BinaryPacket2.Seconds = gpsData.seconds;
  BinaryPacket2.Latitude = float_lat;
  BinaryPacket2.Longitude = float_lon;
  BinaryPacket2.Altitude = (uint16_t)(gpsData.alt_raw/1000);
  BinaryPacket2.Speed = (uint8_t)((float)gpsData.speed_raw*0.036); // Using NAV-VELNED gSpeed, which is in cm/s. Convert to kph.

  // Temporary pDOP info, to determine suitable pDOP limits.
  // float pDop = (float)gpsData.pDOP/10.0;
  // if (pDop>255.0){
  //  pDop = 255.0;
  // }
  // BinaryPacket.Speed = (uint8_t)pDop;
  BinaryPacket2.BattVoltage = volts_scaled;
  BinaryPacket2.Sats = gpsData.sats_raw;
  BinaryPacket2.Temp = si4032_temperature;
#ifndef USERFLAG_A
  BinaryPacket2.dummy1 = 0;
  BinaryPacket2.dummy2 = 0;
  BinaryPacket2.dummy3 = 0;
  BinaryPacket2.dummy4 = NOGPS_counter*10;
  BinaryPacket2.unused = 0;
#else
  BinaryPacket2.FlightNumber = USERFLAG_A;
  BinaryPacket2.SondeType = USERFLAG_B;
  BinaryPacket2.dummy1 = 0;
  BinaryPacket2.dummy2 = 0;
  BinaryPacket2.unused = 0;
#endif
  // Add onto the sats_raw value to indicate if the GPS is in regular tracking (+100)
  // or power optimized tracker (+200) modes.
  if(gpsData.psmState == 1){
    BinaryPacket2.Sats += 100;
  } else if(gpsData.psmState == 2){
    BinaryPacket2.Sats += 200;
  }

  BinaryPacket2.Checksum = (uint16_t)array_CRC16_checksum((char*)&BinaryPacket2,sizeof(BinaryPacket2)-2);

#define MFSKDEBUGxx
#ifdef MFSKDEBUG1
  // Write BinaryPacket into the RTTY transmit buffer as hex
//  memcpy(buf_mfsk,&BinaryPacket2,sizeof(BinaryPacket2));
  sprintf(buf_rtty,"$$$$");
  //print_hex(buf_mfsk, sizeof(BinaryPacket2), buf_rtty+4);
  strlcat(buf_rtty+4,dummy,49);
//  CRC_rtty = string_CRC16_checksum(buf_rtty + 4);
//  sprintf(buf_rtty+4+20,"*%04X\n", CRC_rtty & 0xffff);
//  sprintf(buf_rtty + 4 + sizeof(BinaryPacket2)*2, "__*%04X\n", CRC_rtty & 0xffff);
  // $$$$e701010000000000000000000000000000000022020000000000000000006e8e__*01C0 (-9.1 dB SNR)

  //Configure for transmit
  tx_buffer = buf_rtty;
  // Enable the radio, and set the tx_on flag to 1.
  start_bits = RTTY_PRE_START_BITS;
  radio_enable_tx();
  current_mode = RTTY;
  tx_on = 1;

  // Wait until transmit has finished.
  while(tx_on){
    NVIC_SystemLPConfig(NVIC_LP_SEVONPEND, DISABLE);
    __WFI();
  }
  _delay_ms(1000);
  current_mode = MFSK;
#endif

  #ifdef CONTINUOUS_MODE
    // Write Preamble characters into mfsk buffer.
    sprintf(buf_mfsk, "\x1b\x1b\x1b\x1b");
    // Encode the packet, and write into the mfsk buffer.
    int coded_len = horus_l2_encode_tx_packet((unsigned char*)buf_mfsk+4,(unsigned char*)&BinaryPacket2,sizeof(BinaryPacket2));
  #else
    // Double length preamble to help the decoder lock-on after a quiet period.
    // Write Preamble characters into mfsk buffer.
    sprintf(buf_mfsk, "\x1b\x1b\x1b\x1b\x1b\x1b\x1b\x1b");
    // Encode the packet, and write into the mfsk buffer.
    int coded_len = horus_l2_encode_tx_packet((unsigned char*)buf_mfsk+8,(unsigned char*)&BinaryPacket2,sizeof(BinaryPacket2));
  #endif

#ifdef MFSKDEBUG2
  // Write the coded packet into the RTTY transmit buffer as hex
  //sprintf(buf_rtty,"$$$$");
  strcpy(buf_rtty,"$$$$");
  // ATTENTION: while the horus-Gui truncates in RAW line everything over some length, the full line cant be send in one packet
  //            This sample dumps only the last 40 bytes
  print_hex(buf_mfsk+8+40, coded_len-40, buf_rtty+4);
  CRC_rtty = string_CRC16_checksum(buf_rtty + 4);
  sprintf(buf_rtty+4 + (coded_len*2)-80,"*%04X\n", CRC_rtty & 0xffff);
  // The first two x24 chars are the ** Prefix for Horus-Format
  // 2424c16f102c0c1dc5c99316edcecd9455af7f3c2011d80c5a85fb230359c1fad0431c31c9d456df7ed8205a983b2a935f2df81d8289a1a6f87ac2a311b9cc72d5*1C71
  tx_buffer = buf_rtty;
  // Enable the radio, and set the tx_on flag to 1.
  start_bits = RTTY_PRE_START_BITS;
  radio_enable_tx();
  current_mode = RTTY;
  tx_on = 1;

  // Wait until transmit has finished.
  while(tx_on){
    NVIC_SystemLPConfig(NVIC_LP_SEVONPEND, DISABLE);
    __WFI();
  }
  current_mode = MFSK;
  // Wait until tx_enable
  while(tx_enable == 0){
    NVIC_SystemLPConfig(NVIC_LP_SEVONPEND, DISABLE);
    __WFI();
  }
  _delay_ms(1000);
#endif

  // Data to transmit is the coded packet length, plus the preamble.
  #ifdef CONTINUOUS_MODE
    packet_length = coded_len+4;
  #else
    packet_length = coded_len+8;
  #endif

  tx_buffer = buf_mfsk;

  // Enable the radio, and set the tx_on flag to 1.
  radio_enable_tx();
  tx_on = 1;
}

#endif

void send_morse_ident(){
    char buf_temp[100];
    char buf_qra[14];
	continuous_mode = 0;
	radio_rw_register(0x73, 0x00, 1);
	radio_inhibit_tx();
	_delay_ms(500);

	// Expand the MORSE MESSAGE
	//================================================
	memset(buf_morse, '\0', sizeof(buf_morse));
	strcpy(buf_morse,MORSE_MESSAGE);
#ifdef MORSE_EXTENDED_MSG
	memset(buf_temp, '\0', sizeof(buf_temp));
	memset(buf_qra, '\0', sizeof(buf_qra));
	if ((gpsData.gpsFixOK == 1) && (gpsData.sats_raw > 0)) {
		locator_from_lonlat(gpsData.lon_raw,gpsData.lat_raw,4,buf_qra);
		sprintf(buf_temp," / A %ldm / S %d / QRA %s / V %d / T %d +",gpsData.alt_raw/1000,gpsData.sats_raw,buf_qra,voltage,si4032_temperature);
	} else {
		#ifdef NOGPS_RESET_AFTER_TXCOUNT
		sprintf(buf_temp," / No GPS / boot ON / V %d / T %d +",voltage,si4032_temperature);
		#else
		sprintf(buf_temp," / No GPS / boot OFF / V %d / T %d +",voltage,si4032_temperature);
		#endif
	}
	strcat(buf_morse, buf_temp);
#endif
	//================================================

	sendMorse(buf_morse); // MORSE_MESSAGE);
	_delay_ms(500);
	#ifdef CONTINUOUS_MODE
	continuous_mode = 1;
	#endif
}


#ifdef  DEBUG
void assert_failed(uint8_t* file, uint32_t line)
{
    while (1);
}
#endif
