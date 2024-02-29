
/*
// GPIO:
// IO015   :   CS_H
// IO013   :   MOSI_H
// IO012   :   MISO_H
// IO014   :   CLK_H
// IO027   :   PWM12
// IO026   :   PWM11
// IO025   :   PWM21
// IO033   :   LED
// IO004   :   PWM22
// IO016   :   RX2
// IO032   :   BATT_ADC
// IO035   :   INPUT
// IO034   :   INPUT
// IO036   :   INPUT
// IO039   :   INPUT
// IO017   :   TX2
// IO005   :   CS_V
// IO018   :   CLK_V
// IO019   :   MISO_V
// IO021   :   SDA
// IO022   :   SCL
// IO023   :   MOSI_V
*/
#include <Arduino.h>
#include <SPI.h>
#include "LIS3MDL.h"
#include "BMI323.h"

// Declare pointers to SPIClass objects
SPIClass *hspi = new SPIClass(HSPI);
SPIClass *vspi = new SPIClass(VSPI);

// Initialize LIS3MDL object
LIS3MDL lis3mdl(vspi);
// Initialize BMI323 object
BMI323 bmi323(hspi);

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);

  // Initialize HSPI using specific pins
  hspi->begin();  // CLK, MISO, MOSI, CS for HSPI

  // Initialize VSPI using specific pins
  vspi->begin();  // CLK, MISO, MOSI, CS for VSPI
  lis3mdl.begin();

  lis3mdl.readWhoAmI();
}

void loop() {  
  // Read WHO_AM_I register
  // lis3mdl.readData();
  lis3mdl.readWhoAmI();
  bmi323.read_who_am_i();
  delay(1000);
}