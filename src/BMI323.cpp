/*
  BMI323.cpp - Library for Description
  Created by Tommaso, 25/02/2024.
  Brief description: -
*/

#include "BMI323.h"

BMI323::BMI323(SPIClass* _spi) : spi(_spi) {
    pinMode(BMI323_CS_PIN, OUTPUT);
    digitalWrite(BMI323_CS_PIN, LOW); // Select the device
    digitalWrite(BMI323_CS_PIN, HIGH); // Deselect the device
}

BMI323::~BMI323() {
  // Destructor implementation
}

// Add private method implementations here

// Add public method implementations here

void BMI323::read_who_am_i() {
    spi->beginTransaction(SPISettings(1000*1000, MSBFIRST, SPI_MODE0)); // Configure SPI settings
    digitalWrite(BMI323_CS_PIN, LOW); // Select the device    
    
    uint16_t whoAmI;
    spi->transfer(0x00 | 0x80); // Read command   
    whoAmI = spi->transfer16(0x00);       // Dummy byte to read data

    digitalWrite(BMI323_CS_PIN, HIGH); // Deselect the device
    spi->endTransaction();

    
    // Print the value of WHO_AM_I register
    Serial.print("WHO_AM_I BMI323 register value: 0x");
    Serial.println(whoAmI, HEX);
}