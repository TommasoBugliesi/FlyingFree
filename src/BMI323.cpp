/*
  BMI323.cpp - Library for Description
  Created by Tommaso, 25/02/2024.
  Brief description: -
*/
#include <EEPROM.h>

#include "BMI323.h"
#include "Global.h"

BMI323::BMI323(SPIClass* _spi) : spi(_spi) {
    pinMode(BMI323_CS_PIN, OUTPUT);
    digitalWrite(BMI323_CS_PIN, LOW); // Select the device
    digitalWrite(BMI323_CS_PIN, HIGH); // Deselect the device
}

BMI323::~BMI323() {
  // Destructor implementation
}

// Add private method implementations here
void BMI323::readRegisters(uint8_t reg, uint16_t *buffer, size_t length){
  spi->beginTransaction(SPISettings(BMI323_SPI_SPEED, MSBFIRST, SPI_MODE0)); // Configure SPI settings
  digitalWrite(BMI323_CS_PIN, LOW); // Select the device
  
  spi->transfer(reg | 0x80); // Read command
  for (size_t  i=0; i<length; i++){
    if (i==0){
      spi->transfer(0x00);                    // Dummy[7:0]  
    }
    buffer_LSB = spi->transfer(0x00);       // Data_1[7:0](Address + i)
    buffer_MSB = spi->transfer(0x00);       // Data_1[15:8](Address + i)
    buffer[i] = (buffer_MSB << 8) | (buffer_LSB);
  }

  digitalWrite(BMI323_CS_PIN, HIGH); // Deselect the device
  spi->endTransaction(); // End SPI transaction
}
void BMI323::readRegisters(uint8_t reg, int16_t *buffer, size_t length){
  spi->beginTransaction(SPISettings(BMI323_SPI_SPEED, MSBFIRST, SPI_MODE0)); // Configure SPI settings
  digitalWrite(BMI323_CS_PIN, LOW); // Select the device
  
  spi->transfer(reg | 0x80); // Read command
  for (size_t  i=0; i<length; i++){
    if (i==0){
      spi->transfer(0x00);                    // Dummy[7:0]  
    }
    buffer_LSB = spi->transfer(0x00);       // Data_1[7:0](Address + i)
    buffer_MSB = spi->transfer(0x00);       // Data_1[15:8](Address + i)
    buffer[i] = (buffer_MSB << 8) | (buffer_LSB);
  }

  digitalWrite(BMI323_CS_PIN, HIGH); // Deselect the device
  spi->endTransaction(); // End SPI transaction
}

void BMI323::writeRegister(uint8_t reg, uint16_t reg_value){
  spi->beginTransaction(SPISettings(BMI323_SPI_SPEED, MSBFIRST, SPI_MODE0)); // Configure SPI settings
  digitalWrite(BMI323_CS_PIN, LOW); // Select the device
  
  buffer_LSB = (reg_value & 0xFF);
  buffer_MSB = (reg_value >> 8) & 0xFF;
  register_value = buffer_LSB << 8 | buffer_MSB;
  spi->transfer(reg & 0x7F); // Write command
  spi->transfer(buffer_LSB); // Send register
  spi->transfer(buffer_MSB); // Send register
  register_value = 0x00; 

  digitalWrite(BMI323_CS_PIN, HIGH); // Deselect the device
  spi->endTransaction(); // End SPI transaction
}

void BMI323::reset(){
  register_value = BMI323_CMD_SOFT_RESET; // Set register value
  BMI323::writeRegister(BMI323_REG_CMD, BMI323_CMD_SOFT_RESET); // Write register value
  register_value = 0x00; // Reset register_value reset

  delay(1000);
}

void BMI323::writeAccConfig(){
  register_value |= BMI323_ACC_OPERATION_HIGH_PERFORMANCE << 12;       //Define register value
  register_value |= BMI323_ACC_AVERAGING_4 << 8;    //Define register value
  register_value |= BMI323_ACC_BANDWIDTH_ODR_2 << 7;     //Define register value
  register_value |= BMI323_ACC_RANGE_4G << 4;     //Define register value
  register_value |= BMI323_ODR_3_2KHZ;         //Define register value
  BMI323::writeRegister(BMI323_REG_ACC_CONF, register_value);
  register_value = 0x00; // Reset register_value
}

void BMI323::readAccConfig(){
  uint16_t buffer[1];
  BMI323::readRegisters(BMI323_REG_ACC_CONF, buffer);

  if ((buffer[0]>>4 & 0x03) == 0x00){
    acc_gain = 2 * 9.8067 / 32768; // m/s^2 per bit constant
  }
  else if ((buffer[0]>>4 & 0x03) == 0x01) {
    acc_gain = 4 * 9.8067 / 32768; // m/s^2 per bit constant
  }
  else if ((buffer[0]>>4 & 0x03) == 0x02) {
    acc_gain = 8 * 9.8067 / 32768; // m/s^2 per bit constant
  }
  else if ((buffer[0]>>4 & 0x03) == 0x03) {
    acc_gain = 16 * 9.8067 / 32768; // m/s^2 per bit constant
  }
  else {
    Serial.print("Value register for BMI323_REG_ACC_CONF unknown!");
  }
}

void BMI323::writeGyrConfig(){
  register_value |= BMI323_GYRO_OPERATION_HIGH_PERFORMANCE << 12;       //Define register value
  register_value |= BMI323_GYRO_AVERAGING_4 << 8;    //Define register value
  register_value |= BMI323_GYRO_BANDWIDTH_ODR_2 << 7;     //Define register value
  register_value |= BMI323_GYRO_RANGE_500DPS << 4;     //Define register value
  register_value |= BMI323_GYRO_ODR_3_2KHZ;         //Define register value
  BMI323::writeRegister(BMI323_REG_GYR_CONF, register_value);
  register_value = 0x00; // Reset register_value
}

void BMI323::readGyrConfig(){
  uint16_t buffer[1];
  BMI323::readRegisters(BMI323_REG_GYR_CONF, buffer);
  if (buffer[0]>>4 & (0x00)){
    gyro_gain = 125 * PI / 180 / 32768; // rad/s per bit constant
  }
  else if (buffer[0]>>4 & (0x01)) {
    gyro_gain = 250 * PI / 180 / 32768; // rad/s per bit constant
  }
  else if (buffer[0]>>4 & (0x02)) {
    gyro_gain = 500 * PI / 180 / 32768; // rad/s per bit constant
  }
  else if (buffer[0]>>4 & (0x03)) {
    gyro_gain = 1000 * PI / 180 / 32768; // rad/s per bit constant
  }
  else if (buffer[0]>>4 & (0x04)) {
    gyro_gain = 2000 * PI / 180 / 32768; // rad/s per bit constant
  }
  else {
    Serial.print("Value register for BMI323_REG_GYR_CONF unknown!");
  }
}

void BMI323::readStatusReg(){
  BMI323::readRegisters(BMI323_REG_STATUS, status_register);
  // Serial.print("Read Register Status: ");  //DEBUG
  // Serial.println(status_register[0], HEX); //DEBUG
}

void BMI323::readErrorReg(){
  BMI323::readRegisters(BMI323_REG_ERR_REG, status_register);
  Serial.print("Read Register BMI323 Error: ");  
  Serial.println(status_register[0], DEC); 
}

void BMI323::readDataRaw(){
  BMI323::readRegisters(BMI323_REG_ACC_DATA_X, data_raw, 6);
}

// Add public method implementations here
void BMI323::begin(){
  BMI323::reset(); // Soft Reset
  BMI323::readWhoAmI(); // A rising edge on CSB is needed before starting the SPI communicatio
  BMI323::readErrorReg(); // Check for errors

  BMI323::writeAccConfig(); // Write accelerometer configuration
  BMI323::writeGyrConfig(); // Write gyro configuration

  // Read configuration from the device
  BMI323::readAccConfig();
  BMI323::readGyrConfig();

  // Initialize EEPROM with predefined size
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Failed to initialize EEPROM");
  }

  // // Perform calibration of offsets
  // BMI323::calibrateAcc();
  // BMI323::calibrateGyr();

  // Read data from EEPROM to initialize calibration values
  EEPROM.get(VALUE_ADDR_GYR_0, gyro_offset[0]);
  EEPROM.get(VALUE_ADDR_GYR_1, gyro_offset[1]);
  EEPROM.get(VALUE_ADDR_GYR_2, gyro_offset[2]);
  // Serial.printf("Gyroscope offset x-axis is: %f \n", gyro_offset[0]);
  // Serial.printf("Gyroscope offset y-axis is: %f \n", gyro_offset[1]);
  // Serial.printf("Gyroscope offset z-axis is: %f \n", gyro_offset[2]);

  EEPROM.get(VALUE_ADDR_ACC_0, acc_offset[0]);
  EEPROM.get(VALUE_ADDR_ACC_1, acc_offset[1]);
  EEPROM.get(VALUE_ADDR_ACC_2, acc_offset[2]);
  // Serial.printf("Accelerometer offset x-axis is: %f \n", acc_offset[0]);
  // Serial.printf("Accelerometer offset y-axis is: %f \n", acc_offset[1]);
  // Serial.printf("Accelerometer offset z-axis is: %f \n", acc_offset[2]);
}

void BMI323::readWhoAmI() {
    uint16_t whoAmI[1];
    BMI323::readRegisters(BMI323_REG_CHIP_ID, whoAmI, 1);

    // Print the value of WHO_AM_I register
    Serial.print("WHO_AM_I BMI323 register value: 0x");
    Serial.println(whoAmI[0] & 0xFF, HEX);
}

void BMI323::readData(){
  BMI323::readStatusReg();

  // Check if new gyro or acc data available
  if ((status_register[0] & 0x80) | (status_register[0] & 0x40)){
    BMI323::readDataRaw();

    // Update global pointer
    memcpy(globalStructPtr->bmi323DataPrv.accData, data, 3 * sizeof(float)); // Copy first 3 elements
    memcpy(globalStructPtr->bmi323DataPrv.gyroData, data + 3, 3 * sizeof(float)); // Copy last 3 elements

    if (status_register[0] & 0x80){
      for (size_t i=0; i<3; i++){
        data[i] = (float)data_raw[i]*acc_gain + acc_offset[i]; // output is m/s^2
      }
    }
    if (status_register[0] & 0x40){
      for (size_t ii=3; ii<6; ii++){
        data[ii] = (float)data_raw[ii]*gyro_gain + gyro_offset[ii-3]; // output is rad/s
      }
    }

    // Update global pointer
    memcpy(globalStructPtr->bmi323Data.accData, data, 3 * sizeof(float)); // Copy first 3 elements
    memcpy(globalStructPtr->bmi323Data.gyroData, data + 3, 3 * sizeof(float)); // Copy last 3 elements

  }
  // Serial.printf("%f,%f,%f\n",data[0],data[1],data[2]);

}

void BMI323::calibrateAcc(){
  float tmp_data[3] = {0,0,0};
  int counter = 0;

  for (size_t n=0; n<2000; n++){
    BMI323::readData();
    if (status_register[0] & 0x80){
      counter ++; // Incremente counter for averaging
      for (size_t i=0; i<3; i++){
        tmp_data[i] = data[i]+tmp_data[i]; // output is m/s^2
      }
    }
    delay(0.1);
  }

  if (counter > 100){
    for (size_t i=0; i<3; i++){
      acc_offset[i] = -(float)tmp_data[i]/counter; // take average value
    }
    acc_offset[2] = acc_offset[2] + 9.8067; // Z-axis is subject to gravity

    // Write data to EEPROM
    EEPROM.put(VALUE_ADDR_ACC_0, acc_offset[0]);
    EEPROM.put(VALUE_ADDR_ACC_1, acc_offset[1]);
    EEPROM.put(VALUE_ADDR_ACC_2, acc_offset[2]);

    // Commit changes to EEPROM
    EEPROM.commit();
  }
  else {
    Serial.print("Running acc calibration again! Not enough samples.");
    BMI323::calibrateAcc(); // Repeat calibration
  }

  //TODO: missing logic for gain calibration
}

void BMI323::calibrateGyr(){
  float tmp_data[3] = {0,0,0};
  int counter = 0;

  for (size_t n=0; n<2000; n++){
    BMI323::readData();
    if (status_register[0] & 0x40){
      counter ++; // Incremente counter for averaging
      for (size_t i=0; i<3; i++){
        tmp_data[i] = data[i+3]+tmp_data[i]; // output is rad/s
      }
    }
    delay(0.1);
  }

  if (counter > 100){
    for (size_t i=0; i<3; i++){
      gyro_offset[i] = -(float)tmp_data[i]/counter; // take average value
    }
    // Write data to EEPROM
    EEPROM.put(VALUE_ADDR_GYR_0, gyro_offset[0]);
    EEPROM.put(VALUE_ADDR_GYR_1, gyro_offset[1]);
    EEPROM.put(VALUE_ADDR_GYR_2, gyro_offset[2]);

    // Commit changes to EEPROM
    EEPROM.commit();
  }
  else {
    Serial.print("Running gyro calibration again! Not enough samples.");
    BMI323::calibrateGyr(); // Repeat calibration
  }

  //TODO: missing logic for gain calibration
}

