/*
  LIS3MDL.cpp - Library for Description
  Created by Tommaso, 25/02/2024.
  Brief description: -
*/

#include "LIS3MDL.h"

LIS3MDL::LIS3MDL(SPIClass* _spi) : spi(_spi) {
  pinMode(LIS3MDL_CS_PIN, OUTPUT);
  // pinMode(LIS3MDL_CS_PIN, INPUT_PULLDOWN);
  // digitalWrite(LIS3MDL_CS_PIN, HIGH); // Deselect the device
}

LIS3MDL::~LIS3MDL() {
  // Destructor implementation
}

// Add private method implementations here
void LIS3MDL::reset(){
  register_value |= LIS3MDL_REBOOT_ENABLE << 3; //Define register value

  digitalWrite(LIS3MDL_CS_PIN, LOW); // Select the device

  spi->transfer(LIS3MDL_REG_CTRL_REG2 & 0x7F); // Read command
  spi->transfer(register_value);       // Dummy byte to read data

  digitalWrite(LIS3MDL_CS_PIN, HIGH); // Deselect the device

  register_value = 0x00; // Reset register_value

  delay(10);

  register_value |= LIS3MDL_SOFT_RESET_ENABLE << 2; //Define register value

  digitalWrite(LIS3MDL_CS_PIN, LOW); // Select the device

  spi->transfer(LIS3MDL_REG_CTRL_REG2 & 0x7F); // Read command
  spi->transfer(register_value);       // Dummy byte to read data

  digitalWrite(LIS3MDL_CS_PIN, HIGH); // Deselect the device

  register_value = 0x00; // Reset register_value

  delay(10);
}
void LIS3MDL::writeCtrlReg1(){
  register_value |= LIS3MDL_TEMPERATURE_DISABLE << 7; //Define register value
  register_value |= LIS3MDL_ULTRAHIGHMODE << 5;      //Define register value
  register_value |= LIS3MDL_DATARATE_1000_HZ << 2;    //Define register value
  register_value |= LIS3MDL_FAST_ODR_ENABLE << 1;     //Define register value
  register_value |= LIS3MDL_SELFTEST_DISABLE;         //Define register value

  digitalWrite(LIS3MDL_CS_PIN, LOW); // Select the device

  spi->transfer(LIS3MDL_REG_CTRL_REG1 & 0x7F); // Read command
  spi->transfer(register_value);       // Dummy byte to read data

  digitalWrite(LIS3MDL_CS_PIN, HIGH); // Deselect the device

  register_value = 0x00; // Reset register_value
}
void LIS3MDL::writeCtrlReg2(){
  register_value |= LIS3MDL_RANGE_8_GAUSS << 5;      //Define register value
  
  digitalWrite(LIS3MDL_CS_PIN, LOW); // Select the device

  spi->transfer(LIS3MDL_REG_CTRL_REG2 & 0x7F); // Read command
  spi->transfer(register_value);       // Dummy byte to read data

  digitalWrite(LIS3MDL_CS_PIN, HIGH); // Deselect the device

  register_value = 0x00; // Reset register_value
}
void LIS3MDL::writeCtrlReg3(){
  register_value |= LIS3MDL_CONTINUOUSMODE;      //Define register value
  
  digitalWrite(LIS3MDL_CS_PIN, LOW); // Select the device

  spi->transfer(LIS3MDL_REG_CTRL_REG3 & 0x7F); // Read command
  spi->transfer(register_value);       // Dummy byte to read data

  digitalWrite(LIS3MDL_CS_PIN, HIGH); // Deselect the device

  register_value = 0x00; // Reset register_value
}
void LIS3MDL::writeCtrlReg4(){
  register_value |= LIS3MDL_ULTRAHIGHMODE << 2;      //Define register value
  
  digitalWrite(LIS3MDL_CS_PIN, LOW); // Select the device

  spi->transfer(LIS3MDL_REG_CTRL_REG4 & 0x7F); // Read command
  spi->transfer(register_value);               // Dummy byte to read data

  digitalWrite(LIS3MDL_CS_PIN, HIGH); // Deselect the device

  register_value = 0x00; // Reset register_value
}
void LIS3MDL::readStatusReg(){
  digitalWrite(LIS3MDL_CS_PIN, LOW); // Select the device

  spi->transfer(LIS3MDL_REG_STATUS | 0x80); // Read command
  status_register = spi->transfer(0x01);    // Dummy byte to read data

  digitalWrite(LIS3MDL_CS_PIN, HIGH); // Deselect the device
}
void LIS3MDL::readDataRaw(){
  LIS3MDL::readStatusReg();

  // Check that new X, Y and Z data are available
  if (status_register & 0x08){
    digitalWrite(LIS3MDL_CS_PIN, LOW); // Select the device

    spi->transfer(LIS3MDL_REG_OUT_X_L | 0x80); // Read command
    for (uint8_t i=0; i<6; i++){
      data_raw[i] = spi->transfer(0x01);       // Register is autoincrementing
    }
    status_register = spi->transfer(0x01);    // Dummy byte to read data

    digitalWrite(LIS3MDL_CS_PIN, HIGH); // Deselect the device
  }
}

// Add public method implementations here
void LIS3MDL::begin(){
  LIS3MDL::reset();
  LIS3MDL::writeCtrlReg1();
  LIS3MDL::writeCtrlReg2();
  LIS3MDL::writeCtrlReg3();
  LIS3MDL::writeCtrlReg4();
}
void LIS3MDL::readWhoAmI() {

  spi->beginTransaction(SPISettings(1000*1000, MSBFIRST, SPI_MODE0)); // Configure SPI settings
  digitalWrite(LIS3MDL_CS_PIN, LOW); // Select the device
  
  uint8_t whoAmI[1];
  spi->transfer(LIS3MDL_REG_WHO_AM_I | 0x80); // Read command
  spi->transfer(whoAmI, 0x01);       // Dummy byte to read data

  digitalWrite(LIS3MDL_CS_PIN, HIGH); // Deselect the device
  spi->endTransaction(); // End SPI transaction
  
  // Print the value of WHO_AM_I register
  Serial.print("WHO_AM_I LIS3MDL register value: 0x");
  Serial.println(whoAmI[0], HEX);
}
void LIS3MDL::readData(){
  LIS3MDL::readDataRaw(); // First read raw data

  // Check that new X, Y and Z data are available
  if (status_register & 0x08){
    for (uint8_t i=0; i<3; i++){
      data[i] = data_raw[i+1]<<8 | data_raw[i]; // Combine data to form 16 bits output
    }

    Serial.print("data[0]: ");
    Serial.println(data[0], DEC);
    Serial.print("data[1]: ");
    Serial.println(data[1], DEC);
    Serial.print("data[2]: ");
    Serial.println(data[2], DEC);
    //TODO: apply conversione to data to go from uint16_t to Gauss
  }
}