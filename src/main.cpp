
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

#include "LIS3MDL.h"
#include "BMI323.h"
#include "AHRS.h"
#include "ESPNOWSender.h"
#include "Global.h"
#include "MotorControl.h"

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"


// Declare pointers to SPIClass objects
SPIClass *hspi = new SPIClass(HSPI);
// SPIClass *vspi = new SPIClass(VSPI);

// Initialize objects
// LIS3MDL lis3mdl(vspi);
BMI323 bmi323(hspi);
AHRS ahrs;
ESPNOWSender espnow;

// // Task handles
// TaskHandle_t task1_Handle;
// TaskHandle_t task2_Handle;
// void task1(void *parameter);
// void task2(void *parameter);
// unsigned long task1StartTime, task1EndTime;
// unsigned long task2StartTime, task2EndTime;

unsigned long StartTime, EndTime;

MotorControl motor;

void setup() {
  // Initialize Serial communication
  Serial.begin(115200); 
  delay(100);

  // Create the mutex
  globalMutex = xSemaphoreCreateMutex();
  if (globalMutex == NULL) {
      printf("Failed to create mutex");
      while(true) {delay(1000);}
  }

  espnow.begin();

  // Init global structure
  initGlobal();

  // Initialize HSPI/VSPI
  hspi->begin();  // CLK, MISO, MOSI, CS for HSPI
  // // vspi->begin();  // CLK, MISO, MOSI, CS for VSPI

  // Init sensors with default configuration 
  // lis3mdl.begin();
  bmi323.begin();

  motor.begin();

  // // Create Task 1 on core 1
  // xTaskCreatePinnedToCore(task1,     // Task function
  //                         "Task 1",  // Task name
  //                         2000*20,      // Stack size (words)
  //                         NULL,      // Task parameters
  //                         1,         // Priority (0 is the lowest)
  //                         &task1_Handle, // Task handle
  //                         0);        // Core 1

  // // Create Task 2 on core 2
  // xTaskCreatePinnedToCore(task2,     // Task function
  //                         "Task 2",  // Task name
  //                         2000*20,      // Stack size (words)
  //                         NULL,      // Task parameters
  //                         1,         // Priority (0 is the lowest)
  //                         &task2_Handle, // Task handle
  //                         1);        // Core 2

}

void loop() { 
  StartTime = micros();

  bmi323.readData();
  ahrs.computeKalman();
  // printf("%f,%f\n",globalStructPtr->ahrsData.angData[0], globalStructPtr->ahrsData.angData[1]);
  motor.motorControl();

  EndTime = micros();

  globalStructPtr-> task1Timing = (float)(EndTime-StartTime);
  globalStructPtr-> task1Frequency = 1000000./globalStructPtr-> task1Timing;

  printf("Task1 operations time %f: \n", globalStructPtr-> task1Timing);
}

// // Task 1 function running at 4kHz
// void task1(void *parameter) {  
//   TickType_t lastWakeTime_T1 = xTaskGetTickCount();
//   const TickType_t frequency_T1 = 10; // ( xTimeInMs * configTICK_RATE_HZ ) / 1000 

//   while (1) {
//     // Record the start time
//     task1StartTime = micros();

//     /*
//     Start of task1 Logic
//     */

//     // bmi323.readData();
//     // ahrs.computeKalman();
//     // printf("%f,%f\n",globalStructPtr->ahrsData.angData[0], globalStructPtr->ahrsData.angData[1]);

//     /*
//     End of task1 Logic
//     */

//     // Record the end time
//     task1EndTime = micros();
//     globalStructPtr-> task1Timing = (float)(task1EndTime-task1StartTime);
//     globalStructPtr-> task1Frequency = 1000000./globalStructPtr-> task1Timing;

//     // printf("Task1 operations time %f: \n", globalStructPtr-> task1Timing);

//     if (globalStructPtr-> task1Timing > 1000){ 
//       printf("WARNING: task1 Overflow, elapsed time is: %f [us] \n", globalStructPtr-> task2Timing);
//     }

//     // Delay to achieve the desired frequency
//     vTaskDelayUntil(&lastWakeTime_T1, frequency_T1);
//   }
// }

// // Task 2 function running at 20ms
// void task2(void *parameter) {
//   TickType_t lastWakeTime_T2 = xTaskGetTickCount();
//   const TickType_t frequency_T2 = 8; // ( xTimeInMs * configTICK_RATE_HZ ) / 1000 
//   int counter = 0;
//   while (1) {
//     // Record the start time
//     task2StartTime = micros();

//     /*
//     Start of task1 Logic
//     */
    
//     motor.motorControl();

//     /*
//     End of task1 Logic
//     */

//     // Record the end time
//     task2EndTime = micros();
//     globalStructPtr-> task2Timing = (float)(task2EndTime-task2StartTime);
//     globalStructPtr-> task2Frequency = 1000000./globalStructPtr-> task2Timing;

//     // printf("Task2 operations time %f: \n", globalStructPtr-> task2Timing);

//     if (globalStructPtr-> task2Timing > 9000){ 
//       printf("WARNING: task2 Overflow, elapsed time is: %f [us] \n", globalStructPtr-> task2Timing);
//     }

//     // Delay to achieve the desired frequency
//     vTaskDelayUntil(&lastWakeTime_T2, frequency_T2);
//   }
// }