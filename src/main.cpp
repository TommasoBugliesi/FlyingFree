
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
#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <EEPROM.h>

#include "Filters.h"

#include "LIS3MDL.h"
#include "BMI323.h"
#include "AHRS.h"
#include "ESPNOW.h"
#include "Global.h"
#include "MotorControl.h"
#include "DataLogging.h"

// Declare pointers to SPIClass objects
SPIClass *hspi = new SPIClass(HSPI);
// SPIClass *vspi = new SPIClass(VSPI);

// Initialize objects
// LIS3MDL lis3mdl(vspi);
BMI323 bmi323(hspi);
AHRS ahrs;
ESPNOW espnow;
MotorControl motor;

unsigned long ahrsLast, motorLast, dataLast;
unsigned long dt;

void setup() {
  // Initialize Serial communication
  Serial.begin(115200); 
  delay(1000);

  // Create the mutex
  globalMutex = xSemaphoreCreateMutex();
  if (globalMutex == NULL) {
      printf("Failed to create mutex");
      while(true) {delay(1000);}
  }

  // Init global structure
  initGlobal();
  initUart();

  // Initialize HSPI/VSPI
  hspi->begin();  // CLK, MISO, MOSI, CS for HSPI
  // vspi->begin();  // CLK, MISO, MOSI, CS for VSPI

  // Init sensors with default configuration 
  // lis3mdl.begin();
  espnow.begin();
  bmi323.begin();
  motor.begin();

  delay(1000);
}

void loop() { 
  if ((micros()-ahrsLast)> 1000){
    bmi323.readData();
    dt = micros()-ahrsLast;
    globalStructPtr-> ahrsTiming = (float)(dt);

    // lp_1st(globalStructPtr->bmi323Data.gyroData, 3, 100, dt, globalStructPtr->bmi323Data_lp1st.gyroData);
    // lp_1st(globalStructPtr->bmi323Data.accData, 3, 50, dt, globalStructPtr->bmi323Data_lp1st.accData);
    ahrs.computeKalman(globalStructPtr->bmi323Data.accData, globalStructPtr->bmi323Data.gyroData);
    ahrs.computeMahony(globalStructPtr->bmi323Data.accData, globalStructPtr->bmi323Data.gyroData);
    // ahrs.weightedAverageFilt();

    // Update task time
    ahrsLast = micros();
  }

  if ((micros()-motorLast)> 2500){
    dt = micros()-motorLast;
    globalStructPtr-> motorTiming = (float)(dt);

    // Motor control algorithm
    motor.motorControl(globalStructPtr->ahrsDataMahony);

    // Update task time
    motorLast = micros();
  }

  if ((micros()-dataLast)> 2000){  
    dt = micros()-dataLast;
    globalStructPtr-> motorTiming = (float)(dt);

    // Send data through UART
    sendUart();

    // Update task time
    dataLast = micros();
  }

}

