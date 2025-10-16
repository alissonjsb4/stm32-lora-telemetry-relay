#include <stdio.h>
#include <stdint.h>
#include "mfsk.h"

int send_16fsk(char current_char) {
	// Step through a byte, and return 4FSK symbols.
	static uint8_t nr_nibble = 0;
	// Local copy of the character.
	char _c = current_char;

	if (nr_nibble == 2){
		// Reached the end of the byte.
		nr_nibble = 0;
		// Return -1 to indicate we have finished with this byte.
		return -1;
	} else {
		// Shift it left to the nibble we are up to.
		for (int _i=0;_i<nr_nibble;_i++){
			_c = _c << 4;
		}
		// Get the current symbol (the 2-bits we need).
		uint8_t symbol = ((uint8_t)_c & 0xF0) >> 4;

		nr_nibble++;
		return (int)symbol;
	}
}

int send_4fsk(char current_char) {
	// Step through a byte, and return 4FSK symbols.
	static uint8_t nr_nibble = 0;
	// Local copy of the character.
	char _c = current_char;

	if (nr_nibble == 4){
		// Reached the end of the byte.
		nr_nibble = 0;
		// Return -1 to indicate we have finished with this byte.
		return -1;
	} else {
		// Shift it left to the nibble we are up to.
		for (int _i=0;_i<nr_nibble;_i++){
			_c = _c << 2;
		}
		// Get the current symbol (the 2-bits we need).
		uint8_t symbol = ((uint8_t)_c & 0xC0) >> 6;

		nr_nibble++;
		return (int)symbol;
	}
}

int send_2fsk(char current_char){
	// Step through a byte, and return 2FSK symbols.
	static uint8_t nr_nibble = 0;
	// Local copy of the character.
	char _c = current_char;

	if (nr_nibble == 8){
		// Reached the end of the byte.
		nr_nibble = 0;
		// Return -1 to indicate we have finished with this byte.
		return -1;
	} else {
		// Shift it left to the nibble we are up to.
		for (int _i=0;_i<nr_nibble;_i++){
			_c = _c << 1;
		}
		// Get the current symbol (the MSB).
		uint8_t symbol = ((uint8_t)_c & 0x80) >> 7;
		nr_nibble++;
		if(symbol){
			return 2;
		} else {
			return 0;
		}

	}
}

/*
int mfsk_test_bits(char *buffer){
  // Add some test bits into the MFSK buffer, compatible with the first 200 test bits produced by fsk_get_test_bits from codec2-dev.
  // Decode using (with GQRX outputting data to UDP): nc -l -u localhost 7355 | ./fsk_demod 4 48000 100 - - | ./fsk_put_test_bits -
  sprintf(buffer, "\x1b\x1b\x1b\x1b\xd8\x2c\xa2\x56\x3f\x4c\x08\xa2\xba\x6f\x84\x07\x6d\x82\xca\x25\x63\xf4\xc0\x8a\x2b\xa6\xf8\x40\x76");
  return 29;
}
*/
