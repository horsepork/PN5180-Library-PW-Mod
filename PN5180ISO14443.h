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
#ifndef PN5180ISO14443_H
#define PN5180ISO14443_H

#include "PN5180.h"


class PN5180ISO14443 : public PN5180 {

public:
  PN5180ISO14443(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin, SPIClass& spi=SPI);
  PN5180ISO14443(uint8_t _nss, Adafruit_MCP23X08 *_mcp, uint8_t _address, SPIClass& _spi);
public:
  uint16_t rxBytesReceived();
  uint32_t GetNumberOfBytesReceivedAndValidBits();
public:
  // Mifare TypeA
  int8_t activateTypeA(uint8_t *buffer, uint8_t kind);
  int8_t hackyRead(uint8_t *buffer); // sorta the same as activateTypeA
  int8_t hackyRead(); // same as above but uses rawTagData as buffer
  void update();
  bool mifareBlockRead(uint8_t blockno,uint8_t *buffer);
  uint8_t mifareBlockWrite16(uint8_t blockno, uint8_t *buffer);
  bool mifareHalt();
  /*
  
   * Helper functions
   */
public:   
  bool setupRF();
  int8_t readCardSerial(uint8_t *buffer);    
  bool isCardPresent();
  uint8_t *getTagData();
private:
  uint8_t rawTagData[4];
  uint8_t tagData[4];
  // void showIRQStatus(uint32_t irqStatus) {
  //   Serial.print(F("IRQ-Status 0x"));
  //   Serial.print(irqStatus, HEX);
  //   Serial.print(": [ ");
  //   if (irqStatus & (1<< 0)) Serial.print(F("RQ "));
  //   if (irqStatus & (1<< 1)) Serial.print(F("TX "));
  //   if (irqStatus & (1<< 2)) Serial.print(F("IDLE "));
  //   if (irqStatus & (1<< 3)) Serial.print(F("MODE_DETECTED "));
  //   if (irqStatus & (1<< 4)) Serial.print(F("CARD_ACTIVATED "));
  //   if (irqStatus & (1<< 5)) Serial.print(F("STATE_CHANGE "));
  //   if (irqStatus & (1<< 6)) Serial.print(F("RFOFF_DET "));
  //   if (irqStatus & (1<< 7)) Serial.print(F("RFON_DET "));
  //   if (irqStatus & (1<< 8)) Serial.print(F("TX_RFOFF "));
  //   if (irqStatus & (1<< 9)) Serial.print(F("TX_RFON "));
  //   if (irqStatus & (1<<10)) Serial.print(F("RF_ACTIVE_ERROR "));
  //   if (irqStatus & (1<<11)) Serial.print(F("TIMER0 "));
  //   if (irqStatus & (1<<12)) Serial.print(F("TIMER1 "));
  //   if (irqStatus & (1<<13)) Serial.print(F("TIMER2 "));
  //   if (irqStatus & (1<<14)) Serial.print(F("RX_SOF_DET "));
  //   if (irqStatus & (1<<15)) Serial.print(F("RX_SC_DET "));
  //   if (irqStatus & (1<<16)) Serial.print(F("TEMPSENS_ERROR "));
  //   if (irqStatus & (1<<17)) Serial.print(F("GENERAL_ERROR "));
  //   if (irqStatus & (1<<18)) Serial.print(F("HV_ERROR "));
  //   if (irqStatus & (1<<19)) Serial.print(F("LPCD "));
  //   Serial.println("]");
  // }
};

#endif /* PN5180ISO14443_H */
