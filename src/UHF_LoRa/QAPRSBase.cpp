/*
	Copyright (C) 2013 Lukasz Nidecki SQ5RWU

    This file is part of ArduinoQAPRS.

    ArduinoQAPRS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    ArduinoQAPRS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ArduinoQAPRS; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

    
 */

/**
 * @file
 * @brief Implementacja klasy QAPRSBase
 */

#include "QAPRSBase.h"
#include "delay.h"
#include "radio.h"
#include "init.h"

QAPRSBase * QAPRSGlobalObject;
/**
 * Czy można wysyłać dane?
 * @return 1 jesli transmisja jest mozliwa, 0 w przeciwnym wypadku
 */
uint8_t QAPRSBase::canTransmit(){
	return 1;
}

/**
 * Rozpocznij nadawanie pakietu ax.25
 * Dodatkowo:
 * 	- włącz nadawanie @see enableTransmission
 * 	- zainicjalizuj crc i ton
 * Ilosć bajtów synchronizacji okresla @see ax25HeaderFlagFieldCount
 */
void QAPRSBase::ax25SendHeaderBegin() {
	this->enableTransmission();
	timerInterruptHandler();
	this->ax25CRC = 0xFFFF;
	this->currentTone = QAPRSMark;
	//this->currentTone = QAPRSSpace;

	for (uint8_t i=0;i<this->ax25HeaderFlagFieldCount;i++)
	{
		this->ax25SendByte(this->ax25FlagFieldValue);
	}
}

/**
 * Wyslij zakończenie pakietu, to jest 2 bajty sumy kontrolnej i znacznik końca pakietu @see QAPRSBase::ax25FlagFieldValue
 */
void QAPRSBase::ax25SendFooter() {
	/**
	 * 16-bit CRC-CCITT
	 * MBS + LE!!!
	 * @see: http://www.tapr.org/pub_ax25.html#2.2.7
	 * @see: http://www.tapr.org/pub_ax25.html#2.2.8
	 */
	static uint8_t tmp;
	tmp = (uint8_t) ((ax25CRC >> 8) ^ 0xff);

	this->ax25SendByte((uint8_t) ((this->ax25CRC) ^ 0xff));
	this->ax25SendByte(tmp);
	this->ax25SendByte(this->ax25FlagFieldValue);
	this->disableTranssmision();
}

/**
 * Wysyła bajt ax.25
 * @param byte Bajt do wysłania
 */
void QAPRSBase::ax25SendByte(uint8_t byte) {
	static uint8_t i;
	static uint8_t ls_bit;
	static uint8_t is_flag;
	static uint8_t bit_stuffing_counter;


	// zapisujemy sobie czy nadawany bit nie jest aby flagą - bo ją nadajemy w specjalny sposób
	is_flag = (uint8_t) (byte == this->ax25FlagFieldValue);

	for(i=0;i<8;i++){
		// vom niedrigstwertigen Bit aus übertragen
		ls_bit = (uint8_t) (byte & 1);

		if (is_flag){
			bit_stuffing_counter = 0;
		} else {
			this->ax25CalcCRC(ls_bit);
		}

		if (ls_bit){
			bit_stuffing_counter++;
			if (bit_stuffing_counter == 5){
				// wenn wir 5 Bits mit dem Wert 1 in einer Reihe haben, dann
				delayuSeconds(this->_toneSendTime);
				this->toggleTone();
				bit_stuffing_counter = 0;
			}
		} else {
			bit_stuffing_counter = 0;
			this->toggleTone();
		}
		byte >>= 1;
		delayuSeconds(this->_toneSendTime);
	}
}

/**
 * Dodaje bit do sumy kontrolnej ramki
 * @param ls_bit Bit danych
 */
void QAPRSBase::ax25CalcCRC(uint8_t ls_bit) {
	static unsigned short	crc_tmp;

	crc_tmp = this->ax25CRC ^ ls_bit;				// XOR lsb of CRC with the latest bit
	this->ax25CRC >>= 1;									// Shift 16-bit CRC one bit to the right

	if (crc_tmp & 0x0001)					// If XOR result from above has lsb set
	{
		this->ax25CRC ^= 0x8408;							// Shift 16-bit CRC one bit to the right
	}
}
/**
 * Przełącz aktualnie generowany ton. @see QAPRSSendingTone
 */
inline void QAPRSBase::toggleTone() {
	this->currentTone = (this->currentTone == QAPRSSpace) ? QAPRSMark : QAPRSSpace;
  this->timer1StartValue = (this->currentTone == QAPRSMark) ? MarkTimerValue : SpaceTimerValue;
}

/**
 * Inicjalizuj Timer1 który jest używany do generowania MARK i SPACE
 */
void QAPRSBase::initializeTimer1() {
    #if defined(__arm__)
    //TODO: Umschreiben auf STM32
    #else
        noInterrupts();
        TIMSK1 |= _BV(TOIE1);
        TCCR1A = 0;
        TCCR1C = 0;
        interrupts();
    #endif

}

/**
 * Inicjalizuj radio i piny.
 */
void QAPRSBase::initializeRadio() {
    #if defined(__arm__)
    //TODO: przepisać na STM32
    #else
        if (this->sensePin){
            pinMode(abs(this->sensePin), INPUT);
            digitalWrite(abs(this->sensePin), LOW);
        }
        pinMode(abs(this->txEnablePin), OUTPUT);
    #endif

	this->disableTranssmision();
	this->initializeTimer1();
}

/**
 * Übertragung freigeben. Die Methode implementiert zusätzlich eine Sendeverzögerung, wenn sie auf..?. @see QAPRSBase::setTxDelay
 */
void QAPRSBase::enableTransmission() {
    #if defined(__arm__)
    //TODO: przepisać na STM32
    #else
        digitalWrite(abs(this->txEnablePin), (this->txEnablePin > 0) ? HIGH : LOW);
    #endif

  radio_set_tx_frequency(TRANSMIT_APRS_FREQUENCY);
  radio_rw_register(0x72, 5, 1);
  GPIO_SetBits(GPIOC, radioNSELpin);
  radio_rw_register(0x71, 0b00010010, 1);
  spi_deinit();
  this->enabled = 1;
	this->doTxDelay();
}

/**
 * Wyłącz nadawanie.
 */
void QAPRSBase::disableTranssmision() {
    #if defined(__arm__)
    //TODO: przepisać na STM32
    #else
	    digitalWrite(abs(this->txEnablePin), (this->txEnablePin > 0) ? LOW : HIGH);
    #endif
  spi_init();
  this->enabled = 0;
  radio_set_tx_frequency(TRANSMIT_FREQUENCY);
  radio_rw_register(0x71, 0x00, 1);
  init_timer(100);
}

/**
 * Wyslij bufor danych. @warning Ta metoda zakończy pracę na bajcie o wartosci 0 jesli wystąpi.
 * @param buffer Bufor z danymi do wysłania
 * @return
 */
QAPRSReturnCode QAPRSBase::send(char * buffer) {
	return this->send(buffer, strlen(buffer));
}

/**
 * Wyslij dane APRS.
 * @param from_addr Adres [znak] źródłowy. Max 6 znaków!
 * @param from_ssid SSID stacji źródłowej: bajty od '0' do 'F'
 * @param to_addr Adres [znak] docelowy. Max 6 znaków!
 * @param to_ssid SSID stacji docelowej: bajty od '0' do 'F'
 * @param packet_content Bufor z danymi pakietu APRS
 * @return
 */
QAPRSReturnCode QAPRSBase::send(char * from_addr, uint8_t from_ssid, char * to_addr, uint8_t to_ssid, char * packet_content) {
	ax25BaseFrame * bf;
	bf = (ax25BaseFrame *)tmpData;
	memset(bf->from, ' ', 6);
	strncpy(bf->from, from_addr,6);
	memset(bf->to, ' ', 6);
	strncpy(bf->to, to_addr, 6);
	bf->to_ssid = to_ssid;
	bf->from_ssid = (uint8_t) (from_ssid > '@' ? from_ssid - 6 : from_ssid);;

	strcpy(bf->packet_content, packet_content);

	bf->control_field = 0x03;
	bf->protocolID = 0xf0;


	uint8_t i;
	for(i=0;i<14;i++){
		tmpData[i] = SHIFT_BYTE(tmpData[i]);
	}
	tmpData[13] |= 1;

	return this->send((char*)tmpData);
}

/**
 * Wysyła bufor danych o oznaczonej długosci.
 * @param buffer Bufor z danymi do wysłania
 * @param length Długosc bufora
 * @return
 */
QAPRSReturnCode QAPRSBase::send(char* buffer, size_t length) {
	int16_t timeout = this->channelFreeWaitingMS;

	while(timeout > 0){
		// jesli mozna nadawac to nadajemy
		if (this->canTransmit()){
			this->ax25SendHeaderBegin();
			while(length--){
				this->ax25SendByte((uint8_t) *buffer);
				buffer++;
			}
			this->ax25SendFooter();
			return QAPRSReturnOK;
		} else {
			// jesli nie mozna to czekamy 100ms i sprawdzamy ponownie
			// maks. czas oczekiwania to channelFreeWaitingMS
            #if defined(__arm__)
                _delay_ms(100);
            #else
                delay(100);
            #endif
			timeout -= 100;
		}
	}
	return QAPRSReturnErrorTimeout;
}

/**
 * Wyslij dane APRS.
 * @param from_addr Adres [znak] źródłowy. Max 6 znaków!
 * @param from_ssid SSID stacji źródłowej: bajty od '0' do 'F'
 * @param to_addr Adres [znak] docelowy. Max 6 znaków!
 * @param to_ssid SSID stacji docelowej: bajty od '0' do 'F'
 * @param relays Bufor z danymi przekaźników. Np. "WIDE1 1" @warning Jeżeli mają być podane 2 lub więcej pozycji to zapisujemy je BEZ przecinków itp. np. "WIDE1 1WIDE2 1"
 * @param packet_content Bufor z danymi pakietu APRS
 * @return
 */
QAPRSReturnCode QAPRSBase::send(char* from_addr, uint8_t from_ssid, char* to_addr, uint8_t to_ssid, char* relays, char* packet_content) {
	return this->send(from_addr, from_ssid, to_addr, to_ssid, relays, packet_content, strlen(packet_content));
}

/**
 * Wyslij dane APRS.
 * @param from_addr Adres [znak] źródłowy. Max 6 znaków!
 * @param from_ssid SSID stacji źródłowej: bajty od '0' do 'F'
 * @param to_addr Adres [znak] docelowy. Max 6 znaków!
 * @param to_ssid SSID stacji docelowej: bajty od '0' do 'F'
 * @param relays Bufor z danymi przekaźników. Np. "WIDE1 1" @warning Jeżeli mają być podane 2 lub więcej pozycji to zapisujemy je BEZ przecinków itp. np. "WIDE1 1WIDE2 1"
 * @param packet_content Bufor z danymi pakietu APRS
 * @param length Długosć packet_content
 * @return
 */
QAPRSReturnCode QAPRSBase::send(char* from_addr, uint8_t from_ssid, char* to_addr, uint8_t to_ssid, char* relays, char* packet_content, size_t length) {
	ax25CustomFrameHeader * bf;
	bf = (ax25CustomFrameHeader *)tmpData;
	memset(bf->from, ' ', 6);
	strncpy(bf->from, from_addr,6);
	memset(bf->to, ' ', 6);
	strncpy(bf->to, to_addr, 6);
	bf->to_ssid = to_ssid;
	bf->from_ssid = (uint8_t) (from_ssid > '@' ? from_ssid - 6 : from_ssid);

	uint8_t relay_size = (uint8_t) strlen(relays);
	strcpy((char*)(tmpData+sizeof(ax25CustomFrameHeader)), relays);

	uint8_t i;
	for(i=0;i<sizeof(ax25CustomFrameHeader)+relay_size;i++){
		tmpData[i] = SHIFT_BYTE(tmpData[i]);
	}
	// ostatni bit w adresach musi byc ustawiony na 1
	tmpData[sizeof(ax25CustomFrameHeader)+relay_size - 1] |= 1;

	// control_field
	tmpData[(sizeof(ax25CustomFrameHeader)+relay_size)] = 0x03;
	// protocolID
	tmpData[(sizeof(ax25CustomFrameHeader)+relay_size+1)] = 0xf0;


	strncpy((char*)(tmpData+sizeof(ax25CustomFrameHeader)+relay_size+2), packet_content, length);

	return this->send((char*)tmpData, (sizeof(ax25CustomFrameHeader)+relay_size+2+length));
}

/**
 * Wysyła dane APRS @warning Ta metoda zakończy pracę na bajcie o wartosci 0 jesli wystąpi.
 * @param buffer Bufor z częscią APRSową ramki - tzn. sam czyste dane APRS, bez SSIDów itp.
 * @return Status operacji
 */
QAPRSReturnCode QAPRSBase::sendData(char* buffer) {
	return this->send((char*)this->from_addr, this->from_ssid, (char*)this->to_addr, this->to_ssid, (char*)this->relays, buffer);
}

/**
 * Wysyła dane APRS
 * @param buffer Bufor z częscią APRSową ramki - tzn. sam czyste dane APRS, bez SSIDów itp.
 * @param length Długosć bufora
 * @return Status operacji
 */
QAPRSReturnCode QAPRSBase::sendData(char* buffer, size_t length) {
	return this->send((char*)this->from_addr, this->from_ssid, (char*)this->to_addr, this->to_ssid, (char*)this->relays, buffer, length);
}

/**
 * Inicjalizacja biblioteki
 * @param sensePin Pin [we] Arduino na którym 0 oznacza zgodę na nadwanie a 1 jej brak.
			Podanie 0 jako numeru pinu powoduje WY�ACZENIE wykrywania nadawania i zmusza programistę do samodzielnej jego obsługi!
 * @param txEnablePin Pin [wy] Arduino na którym 1 oznacza nadawanie
 */
void QAPRSBase::init(int8_t sensePin, int8_t txEnablePin) {
	QAPRSGlobalObject = this;
	this->sensePin = sensePin;
	this->txEnablePin = txEnablePin;
	this->txDelay = this->defaultTxDelay;
	this->setVariant();
  this->timer1StartValue = MarkTimerValue;

	this->initializeRadio();
}

/**
 * Zainicjuj bibliotekę na potrzeby metody sendData @see QAPRSBase::sendData
 * @param sensePin sensePin Pin [we] Arduino na którym 0 oznacza zgodę na nadwanie a 1 jej brak.
			Podanie 0 jako numeru pinu powoduje WY�ACZENIE wykrywania nadawania i zmusza programistę do samodzielnej jego obsługi!
 * @param txEnablePin Pin [wy] Arduino na którym 1 oznacza nadawanie
 * @param from_addr Adres [znak] źródłowy. Max 6 znaków!
 * @param from_ssid SSID stacji źródłowej: bajty od '0' do 'F'
 * @param to_addr Adres [znak] docelowy. Max 6 znaków!
 * @param to_ssid SSID stacji docelowej: bajty od '0' do 'F'
 * @param relays Ścieżki (relaye) np. "WIDE1-1,WIDE2-1". max 3 człony oddzielone przecinkami!
 */
void QAPRSBase::init(int8_t sensePin, int8_t txEnablePin, char* from_addr, uint8_t from_ssid, char* to_addr, uint8_t to_ssid, char* relays) {
	this->init(sensePin, txEnablePin);
	this->setFromAddress(from_addr, from_ssid);
	this->setToAddress(to_addr, to_ssid);
	this->setRelays(relays);
}

/**
 * Ustaw adres docelowy na potrzeby metody sendData
 * @param from_addr Adres [znak] źródłowy. Max 6 znaków!
 * @param from_ssid SSID stacji źródłowej: bajty od '0' do 'F'
 */
void QAPRSBase::setFromAddress(char* from_addr, uint8_t from_ssid) {
	memset(this->from_addr, ' ', 6);
	strcpy(this->from_addr, from_addr);
	this->from_ssid = from_ssid;
}

/**
 * Ustaw adres docelowy na potrzeby metody sendData
 * @param to_addr Adres [znak] docelowy. Max 6 znaków!
 * @param to_ssid SSID stacji docelowej: bajty od '0' do 'F'
 */
void QAPRSBase::setToAddress(char* to_addr, uint8_t to_ssid) {
	memset(this->to_addr, ' ', 6);
	strcpy(this->to_addr, to_addr);
	this->to_ssid = to_ssid;
}

/**
 * Przetwórz zapis scieżek (relayów) z formatu ludzkiego na format do ramki ax.25
 * @param relays np. "WIDE1-1,WIDE2-1". max 3 człony oddzielone przecinkami! @see http://www.aprs.pl/sciezka.htm
 * @param dst
 */
void QAPRSBase::parseRelays(const char* relays, char* dst) {
	uint8_t relays_len = (uint8_t) strlen(relays);
	uint8_t relays_ptr = 0;
	uint8_t dst_ptr = 0;
	uint8_t fill_length = 0;

	for(relays_ptr=0;relays_ptr<relays_len;relays_ptr++){
		if (relays[relays_ptr] == ',' || relays_ptr == relays_len-1){
			if (relays[relays_ptr] != ','){
				dst[dst_ptr] = relays[relays_ptr] == '-' ? ' ' :  relays[relays_ptr]; // zamiana ',' na ' '
				dst_ptr++;
			}
			// koniec elementu
			if (dst_ptr < 7){
				fill_length = (uint8_t) (7 - dst_ptr);
			} else if (dst_ptr > 7 && dst_ptr < 7+7){
				fill_length = (uint8_t) (7 + 7 - dst_ptr);
			} else if(dst_ptr > 7+7 && dst_ptr < 7+7+7){
				fill_length = (uint8_t) (7 + 7 + 7 - dst_ptr);
			}
			while(fill_length){
				dst[dst_ptr] = ' ';
				fill_length--;
				dst_ptr++;
			}
		} else {
			dst[dst_ptr] = relays[relays_ptr] == '-' ? ' ' :  relays[relays_ptr]; // zamiana ',' na ' '
			dst_ptr++;
		}
	}
	dst[dst_ptr] = 0;
}

/**
 * Realizuje opóźnienie jeżeli zostało ustawione
 */
void QAPRSBase::doTxDelay() {
	if (this->txDelay){
        #if defined(__arm__)
            _delay_ms(this->txDelay);
        #else
            delay(this->txDelay);
        #endif
	}
}

void QAPRSBase::setVariant(QAPRSVariant variant) {
	this->variant = variant;
	switch(variant){
	case QAPRSVHF:
		this->_toneSendTime = this->toneSendTime1200;
		this->ax25HeaderFlagFieldCount = this->ax25HeaderFlagFieldCount1200;
		break;
	case QAPRSHF:
		this->_toneSendTime = this->toneSendTime300;
		this->ax25HeaderFlagFieldCount = this->ax25HeaderFlagFieldCount300;
		break;
	}
}

/**
 * Ustaw scieżki (relaye) na potrzeby metody sendData @see QAPRSBase::sendData
 * @param relays
 */
void QAPRSBase::setRelays(char* relays) {
	this->parseRelays(relays, (char*)this->relays);
}


/**
 * Funkcja opóźniająca. W odróżnieniu do delayMicroseconds z Arduino ta funkcja sprawdza prawdziwy czas.
 * @param us
 */
void QAPRSBase::delayuSeconds(uint16_t us) {
    #if defined(__arm__)
        _delay_us(us, 1);
    #else
        unsigned long time = micros();
        while(micros() - time < us){
            //asm("nop");
        }
    #endif
}


/**
 * Ustaw czas zwłoki pomiędzy włączeniem nadawania a rozpoczęciem generowania sygnału
 * @param txDelay Czas w ms
 */
void QAPRSBase::setTxDelay(uint16_t txDelay) {
	this->txDelay = txDelay;
}

void QAPRSBase::timerInterruptHandler() {
  this->togglePin();
  TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));

  TIM2->ARR = this->timer1StartValue;
  TIM2->CNT = 0;

  TIM2->CR1 |= TIM_CR1_CEN;
}

void QAPRSBase::togglePin() {
  if (this->pin){
    this->pin = 0;
    GPIO_ResetBits(GPIOB, radioSDIpin);
  } else {
    this->pin = 1;
    GPIO_SetBits(GPIOB, radioSDIpin);
  }
}

#if defined(__arm__)
//TODO: przepisać na STM32
#else
ISR (TIMER1_OVF_vect)  // timer1 overflow interrupt
{
	QAPRSGlobalObject->timerInterruptHandler();
}
#endif
