// NAME: PN5180ISO14443.h
//
// DESC: ISO14443 protocol on NXP Semiconductors PN5180 module for Arduino.
//
// Copyright (c) 2019 by Dirk Carstensen. All rights reserved.
//
// This file is part of the PN5180 library for the Arduino environment.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// #define DEBUG 1

#include <Arduino.h>
#include "PN5180ISO14443.h"
#include <PN5180.h>
#include "Debug.h"

PN5180ISO14443::PN5180ISO14443(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin, SPIClass& spi) 
              : PN5180(SSpin, BUSYpin, RSTpin, spi) {
}

PN5180ISO14443::PN5180ISO14443(uint8_t _nss, Adafruit_MCP23X08 *_mcp, uint8_t _address, SPIClass& _spi) 
              : PN5180(_nss, _mcp, _address, _spi) {
}

bool PN5180ISO14443::setupRF() {
  PN5180DEBUG(F("Loading RF-Configuration...\n"));
  if (loadRFConfig(0x00, 0x80)) {  // ISO14443 parameters
    PN5180DEBUG(F("done.\n"));
  }
  else return false;

  PN5180DEBUG(F("Turning ON RF field...\n"));
  if (setRF_on()) {
    PN5180DEBUG(F("done.\n"));
  }
  else return false;

  return true;
}



uint16_t PN5180ISO14443::rxBytesReceived() {
	uint32_t rxStatus;
	uint16_t len = 0;
	readRegister(RX_STATUS, &rxStatus);
	// Lower 9 bits has length
	
	len = (uint16_t)(rxStatus & 0x000001ff);
	return len;
}



/*
* buffer : must be 10 byte array
* buffer[0-1] is ATQA
* buffer[2] is sak
* buffer[3..6] is 4 byte UID
* buffer[7..9] is remaining 3 bytes of UID for 7 Byte UID tags
* kind : 0  we send REQA, 1 we send WUPA
*
* return value: the uid length:
* -	zero if no tag was recognized
* - -1 general error
* - -2 card in field but with error
* -	single Size UID (4 byte)
* -	double Size UID (7 byte)
* -	triple Size UID (10 byte) - not yet supported
*/

void printBuffer(uint8_t *buffer, uint8_t len){
	return;
	for(int i = 0; i < len; i++){
		Serial.print(buffer[i]);
		Serial.print(" ");
	}
	Serial.println();
} 
#define hackyReadDebug //Serial.print
#define hackyReadTimingDebug //Serial.print
uint32_t printTime(const char* object, uint32_t time){
	hackyReadTimingDebug(object);
	hackyReadTimingDebug(" time -- ");
	hackyReadTimingDebug(millis() - time);
	hackyReadTimingDebug('\n');
	return millis();
}

uint8_t *PN5180ISO14443::getTagData(){
	// Serial.print("From within 14443: ");
	// for(int i = 0; i < 4; i++){
	// 	Serial.print(tagData[i]);
	// 	Serial.print(" ");
	// }
	// Serial.println();
	return tagData;
}

int8_t PN5180ISO14443::hackyRead(){
	return hackyRead(rawTagData);
}

bool PN5180ISO14443::update(){ // return true if updated
	bool updated = false;
	static uint8_t lastValidTag[4];
	static uint8_t tagRemovedCounter = 0;
	const uint8_t timesBeforeTagRemoved = 50;
	uint8_t prevTagData[4];
	for(int i = 0; i < 4; i++){
		prevTagData[i] = tagData[i];
	}
	int8_t readState = hackyRead();
	if(readState == 1){
		for(int i = 0; i < 4; i++){
			tagData[i] = rawTagData[i];
			lastValidTag[i] = rawTagData[i];
		}
		if(tagData[0] == 0){
			Serial.println("0 showing up as valid read?");
		}
		tagRemovedCounter = 0;
	}
	else if(readState == -12){ // -12 is returned when cards overlap (generally, I think?)
		// for(int i = 0; i < 4; i++){
		// 	tagData[i] = 255;
		// }
	}
	else if(readState == 0){
		tagRemovedCounter++;
		Serial.print("Tag removed counter -- ");
		Serial.println(tagRemovedCounter);
		if(tagRemovedCounter > timesBeforeTagRemoved){
			Serial.println("so...");
			for(int i = 0; i < 4; i++){
				tagData[i] = 0;
			}
		}
	}
	else{
		errorCounter++;
		Serial.print("error counter: ");
		Serial.println(errorCounter);
	}
	setRF_off();
	for(int i = 0; i < 4; i++){
		if(tagData[i] != prevTagData[i]){
			updated = true;
			break;
		}
	}
	return updated;
}

int8_t PN5180ISO14443::hackyRead(uint8_t *buffer){
	for(int i = 0; i < 8; i++){
		buffer[i] = 0;
	}

	uint8_t cmd[7];
	uint8_t uidLength = 0;
	uint32_t timer = millis();
	reset();
	if(!setRF_on()){
		errorCounter++;
		return -1;
	}
	
	if (!loadRFConfig(0x0, 0x80)) {
		errorCounter++;
		return -2;
	}
	
	// clear RX CRC
	if (!writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE)) {
		hackyReadDebug(F("*** ERROR: Clear RX CRC failed!\n"));
		errorCounter++;
		return -3;
	}
	timer = printTime("rx crc", timer);
	// clear TX CRC
	if (!writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE)) {
		hackyReadDebug(F("*** ERROR: Clear TX CRC failed!\n"));
		errorCounter++;
		return -4;
	}
	timer = printTime("tx crc", timer);
	  // activate TRANSCEIVE routine  
	if (!writeRegisterWithOrMask(SYSTEM_CONFIG, 0x00000003)) {
		hackyReadDebug(F("*** ERROR: Activates TRANSCEIVE routine failed!\n"));
		errorCounter++;
		return -6;
	}
	timer = printTime("transceive routine", timer);
	//Send REQA/WUPA, 7 bits in last byte
	cmd[0] = 0x26;
	if (!sendData(cmd, 1, 0x07)) {
		hackyReadDebug(F("*** ERROR: Send REQA/WUPA failed!\n"));
		errorCounter++;
		return -8;
	}
	timer = printTime("reqa", timer);
	// READ 2 bytes ATQA into  buffer
	if (!readData(2, buffer)) {
		hackyReadDebug(F("*** ERROR: READ 2 bytes ATQA failed!\n"));
		errorCounter++;
		return -9;
	}
	timer = printTime("atqa", timer);
	if(buffer[0] == 255){
		errorCounter = 0;
		return 0;
	}
	else if(buffer[0] != 4 && buffer[0] != 7 && buffer[0] != 10){
		errorCounter++;
		return -1;
	}
	
	cmd[0] = 0x93;
	cmd[1] = 0x20;
	if (!sendData(cmd, 2, 0x00)) {
		hackyReadDebug(F("*** ERROR: Send Anti collision 1 failed!\n"));
		errorCounter++;
		return -11;
	}
	timer = printTime("anti collision", timer);
	uint8_t numBytes = rxBytesReceived();
	if (numBytes != 5) {
		hackyReadDebug(F("*** ERROR: Read 5 bytes sak failed!\n"));
		
		return -12;
	};
	timer = printTime("sak", timer);
	if (!readData(5, cmd+2)) {
		hackyReadDebug("Read 5 bytes failed!");
		errorCounter++;
		return -13;
	}
	timer = printTime("read data", timer);
	for (int i = 0; i < 4; i++) buffer[i] = cmd[2 + i];
	errorCounter = 0;
	return 1;
}




int8_t PN5180ISO14443::activateTypeA(uint8_t *buffer, uint8_t kind) {
	uint8_t cmd[7];
	uint8_t uidLength = 0;
	uint32_t timer = millis();
	// Load standard TypeA protocol already done in reset()
	if (!loadRFConfig(0x0, 0x80)) {
		// PN5180DEBUG(F("*** ERROR: Load standard TypeA protocol failed!\n"));
		return -1;
	}
	
	// activate RF field
	setRF_on();
	delay(10);
	timer = millis();
	// OFF Crypto
	if (!writeRegisterWithAndMask(SYSTEM_CONFIG, 0xFFFFFFBF)) {
		// PN5180DEBUG(F("*** ERROR: OFF Crypto failed!\n"));
		return -1;
	}
	// clear RX CRC
	if (!writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE)) {
		// PN5180DEBUG(F("*** ERROR: Clear RX CRC failed!\n"));
		return -1;
	}
	// clear TX CRC
	if (!writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE)) {
		// PN5180DEBUG(F("*** ERROR: Clear TX CRC failed!\n"));
		return -1;
	}

	// set the PN5180 into IDLE state  
	if (!writeRegisterWithAndMask(SYSTEM_CONFIG, 0xFFFFFFF8)) {
		// PN5180DEBUG(F("*** ERROR: set IDLE state failed!\n"));
		return -1;
	}
		
	  // activate TRANSCEIVE routine  
	if (!writeRegisterWithOrMask(SYSTEM_CONFIG, 0x00000003)) {
		// PN5180DEBUG(F("*** ERROR: Activates TRANSCEIVE routine failed!\n"));
		return -1;
	}
	// Serial.print("time after the 4 whatevers -- ");
	// Serial.println(millis() - timer);
	// timer = millis();
	
	// wait for wait-transmit state
	PN5180TransceiveStat transceiveState = getTransceiveState();
	if (PN5180_TS_WaitTransmit != transceiveState) {
		// PN5180DEBUG(F("*** ERROR: Transceiver not in state WaitTransmit!?\n"));
		return -1;
	}
	// Serial.print("time after wait-transmit -- ");
	// Serial.println(millis() - timer);
	// timer = millis();
/*	uint8_t irqConfig = 0b0000000; // Set IRQ active low + clear IRQ-register
    writeEEprom(IRQ_PIN_CONFIG, &irqConfig, 1);
    // enable only RX_IRQ_STAT, TX_IRQ_STAT and general error IRQ
    writeRegister(IRQ_ENABLE, RX_IRQ_STAT | TX_IRQ_STAT | GENERAL_ERROR_IRQ_STAT);  
*/

	// clear all IRQs
	clearIRQStatus(0xffffffff); 
	// Serial.print("time after clear irq -- ");
	// Serial.println(millis() - timer);
	// timer = millis();
	//Send REQA/WUPA, 7 bits in last byte
	cmd[0] = (kind == 0) ? 0x26 : 0x52;
	if (!sendData(cmd, 1, 0x07)) {
		// PN5180DEBUG(F("*** ERROR: Send REQA/WUPA failed!\n"));
		return 0;
	}
	// Serial.print("time after send reqa -- ");
	// Serial.println(millis() - timer);
	
	
	// wait some mSecs for end of RF receiption
	delay(10);
	// timer = millis();
	
	// READ 2 bytes ATQA into  buffer
	if (!readData(2, buffer)) {
		// PN5180DEBUG(F("*** ERROR: READ 2 bytes ATQA failed!\n"));
		return 0;
	}
	// Serial.print("time after aqta-- ");
	// Serial.println(millis() - timer);
	// timer = millis();
	// printBuffer(buffer, 10);
	
	// 
	unsigned long startedWaiting = millis();
	while (PN5180_TS_WaitTransmit != getTransceiveState()) {   
		if (millis() - startedWaiting > 10) {
			// PN5180DEBUG(F("*** ERROR: timeout in PN5180_TS_WaitTransmit!\n"));
			return -1; 
		}	
	}
	// Serial.print("time after wait-transmit num 2 -- ");
	// Serial.println(millis() - timer);
	// timer = millis();
	// clear all IRQs
	clearIRQStatus(0xffffffff); 
	// Serial.print("time after clear irq 2 -- ");
	// Serial.println(millis() - timer);
	// timer = millis();
	// send Anti collision 1, 8 bits in last byte
	cmd[0] = 0x93;
	cmd[1] = 0x20;
	if (!sendData(cmd, 2, 0x00)) {
		// PN5180DEBUG(F("*** ERROR: Send Anti collision 1 failed!\n"));
		return -2;
	}
	// Serial.print("time after anti-collision 1 -- ");
	// Serial.println(millis() - timer);
	
	// wait some mSecs for end of RF receiption
	delay(5);
	// timer = millis();
	uint8_t numBytes = rxBytesReceived();
	if (numBytes != 5) {
		// PN5180DEBUG(F("*** ERROR: Read 5 bytes sak failed!\n"));
		return -2;
	};
	// Serial.print("time after rx bytes received -- ");
	// Serial.println(millis() - timer);
	// timer = millis();
	// read 5 bytes sak, we will store at offset 2 for later usage
	if (!readData(5, cmd+2)) {
		// Serial.println("Read 5 bytes failed!");
		return -2;
	}
	// Serial.print("time after read 5 bytes sak -- ");
	// Serial.println(millis() - timer);
	// timer = millis();
	// We do have a card now! enable CRC and send anticollision
	// save the first 4 bytes of UID
	for (int i = 0; i < 4; i++) buffer[i] = cmd[2 + i];
	// printBuffer(buffer, 10);
	
	//Enable RX CRC calculation
	if (!writeRegisterWithOrMask(CRC_RX_CONFIG, 0x01)) 
	  return -2;
	//Enable TX CRC calculation
	if (!writeRegisterWithOrMask(CRC_TX_CONFIG, 0x01)) 
	  return -2;
	// Serial.print("time after tx/rx crc -- ");
	// Serial.println(millis() - timer);
	// timer = millis();

	//Send Select anti collision 1, the remaining bytes are already in offset 2 onwards
	cmd[0] = 0x93;
	cmd[1] = 0x70;
	if (!sendData(cmd, 7, 0x00)) {
		// no remaining bytes, we have a 4 byte UID
		return 4;
	}
	// Serial.print("time after anti collision 1 (2?) -- ");
	// Serial.println(millis() - timer);
	// timer = millis();
	//Read 1 byte SAK into buffer[2]
	if (!readData(1, buffer+2)) 
	  return -2;
	// Serial.print("time after 1 byte sak -- ");
	// Serial.println(millis() - timer);
	// timer = millis();
	// printBuffer(buffer, 10);
	// Check if the tag is 4 Byte UID or 7 byte UID and requires anti collision 2
	// If Bit 3 is 0 it is 4 Byte UID
	if ((buffer[2] & 0x04) == 0) {
		// Take first 4 bytes of anti collision as UID store at offset 3 onwards. job done
		for (int i = 0; i < 4; i++) buffer[3+i] = cmd[2 + i];
		// Serial.println("I don't think I get here..?");
		// printBuffer(buffer, 10);
		uidLength = 4;
	}
	
	else {
		// Take First 3 bytes of UID, Ignore first byte 88(CT)
		if (cmd[2] != 0x88)
		  return 0;
		for (int i = 0; i < 3; i++) buffer[3+i] = cmd[3 + i];
		// printBuffer(buffer, 10);
		// Clear RX CRC
		timer = millis();
		if (!writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE)) 
	      return -2;
		// Clear TX CRC
		if (!writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE)) 
	      return -2;
		// Serial.print("time after tx/rx crc thing in else -- ");
		// Serial.println(millis() - timer);
		// timer = millis();
		// Do anti collision 2
		cmd[0] = 0x95;
		cmd[1] = 0x20;
		if (!sendData(cmd, 2, 0x00)) 
	      return -2;
		// Serial.print("time after anti collision 2 -- ");
		// Serial.println(millis() - timer);
		// timer = millis();
		//Read 5 bytes. we will store at offset 2 for later use
		if (!readData(5, cmd+2)) 
	      return -2;
		// Serial.print("time after read 5 bytes -- ");
		// Serial.println(millis() - timer);
		// timer = millis();
		// first 4 bytes belongs to last 4 UID bytes, we keep it.
		for (int i = 0; i < 4; i++) {
		  buffer[6 + i] = cmd[2+i];
		}
		// printBuffer(buffer, 10);
		//Enable RX CRC calculation
		if (!writeRegisterWithOrMask(CRC_RX_CONFIG, 0x01)) 
	      return -2;
		//Enable TX CRC calculation
		if (!writeRegisterWithOrMask(CRC_TX_CONFIG, 0x01)) 
	      return -2;
		// Serial.print("time after another tx/rx thing -- ");
		// Serial.println(millis() - timer);
		// timer = millis();
		//Send Select anti collision 2 
		cmd[0] = 0x95;
		cmd[1] = 0x70;
		if (!sendData(cmd, 7, 0x00)) 
	      return -2;
		// Serial.print("time after anti collision 2 (2) -- ");
		// Serial.println(millis() - timer);
		// timer = millis();
		//Read 1 byte SAK into buffer[2]
		if (!readData(1, buffer + 2)) 
	      return -2;
		// printBuffer(buffer, 10);
		// Serial.print("time after 1 byte sak (last thing) -- ");
		// Serial.println(millis() - timer);
		// timer = millis();
		uidLength = 7;
	}
    return uidLength;
}



bool PN5180ISO14443::mifareBlockRead(uint8_t blockno, uint8_t *buffer) {
	bool success = false;
	uint16_t len;
	uint8_t cmd[2];
	// Send mifare command 30,blockno
	cmd[0] = 0x30;
	cmd[1] = blockno;
	if (!sendData(cmd, 2, 0x00))
	  return false;
	//Check if we have received any data from the tag
	delay(5);
	len = rxBytesReceived();
	if (len == 16) {
		// READ 16 bytes into  buffer
		if (readData(16, buffer))
		  success = true;
	}
	return success;
}


uint8_t PN5180ISO14443::mifareBlockWrite16(uint8_t blockno, uint8_t *buffer) {
	uint8_t cmd[2];
	// Clear RX CRC
	writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE);

	// Mifare write part 1
	cmd[0] = 0xA0;
	cmd[1] = blockno;
	sendData(cmd, 2, 0x00);
	readData(1, cmd);

	// Mifare write part 2
	sendData(buffer,16, 0x00);
	delay(10);

	// Read ACK/NAK
	readData(1, cmd);

	//Enable RX CRC calculation
	writeRegisterWithOrMask(CRC_RX_CONFIG, 0x1);
	return cmd[0];
}

bool PN5180ISO14443::mifareHalt() {
	uint8_t cmd[2];
	//mifare Halt
	cmd[0] = 0x50;
	cmd[1] = 0x00;
	sendData(cmd, 2, 0x00);	
	return true;
}



int8_t PN5180ISO14443::readCardSerial(uint8_t *buffer) {
  
    uint8_t response[10];
	int8_t uidLength;
	// Always return 10 bytes
    // Offset 0..1 is ATQA
    // Offset 2 is SAK.
    // UID 4 bytes : offset 3 to 6 is UID, offset 7 to 9 to Zero
    // UID 7 bytes : offset 3 to 9 is UID
    for (int i = 0; i < 10; i++) response[i] = 0;
	// try to activate Type A until response or timeout
	uidLength = activateTypeA(response, 0);

	
	if (uidLength <= 0)
	  return uidLength;
	// UID length must be at least 4 bytes
	if (uidLength < 4)
	  return 0;
	if ((response[0] == 0xFF) && (response[1] == 0xFF))
	  uidLength = 0;
		
	// first UID byte should not be 0x00 or 0xFF
	if ((response[3] == 0x00) || (response[3] == 0xFF)) 
		uidLength = 0;
		
	// check for valid uid, skip first byte (0x04)
	// 0x04 0x00 0xFF 0x00 => invalid uid
	bool validUID = false;
	for (int i = 1; i < uidLength; i++) {
		if ((response[i+3] != 0x00) && (response[i+3] != 0xFF)) {
			validUID = true;
			break;
		}
	}
	if (uidLength == 4) {
		if ((response[3] == 0x88)) {
			// must not be the CT-flag (0x88)!
			validUID = false;
		};
	}
	if (uidLength == 7) {
		if ((response[6] == 0x88)) {
			// must not be the CT-flag (0x88)!
			validUID = false;
		};
		if ((response[6] == 0x00) && (response[7] == 0x00) && (response[8] == 0x00) && (response[9] == 0x00)) {
			validUID = false;
		};
		if ((response[6] == 0xFF) && (response[7] == 0xFF) && (response[8] == 0xFF) && (response[9] == 0xFF)) {
			validUID = false;
		};
	};
//	mifareHalt();
	if (validUID) {
		for (int i = 0; i < uidLength; i++) buffer[i] = response[i+3];
		return uidLength;
	} else {
		return 0;
	}
}

bool PN5180ISO14443::isCardPresent() {

    uint8_t buffer[10];
	return (readCardSerial(buffer) >=4);
}

