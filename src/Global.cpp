/*
  Global.cpp - Library for Description
  Created by Tommaso Bugliesi, 03/03/2024.
  Brief description: Global variables header file.
*/

#include "Global.h"

// Define the global pointer to GlobalData
GlobalStruct* globalStructPtr = nullptr;

// Define the global mutex
SemaphoreHandle_t globalMutex;

void initGlobal() {
  if (!globalStructPtr) {
      globalStructPtr = new GlobalStruct;

      // Initialize globalDataPtr if necessary
      memset(globalStructPtr, 0, sizeof(globalStructPtr));
  }
  globalStructPtr->motorData.KpPitch = 0.0;
  globalStructPtr->motorData.KiPitch = 0.0;
  globalStructPtr->motorData.KdPitch = 0.0;
  globalStructPtr->motorData.KpRoll  = 0.0;
  globalStructPtr->motorData.KiRoll  = 0.0;
  globalStructPtr->motorData.KdRoll  = 0.0;
  
  globalStructPtr->gpioData.T1_In = 0;
  globalStructPtr->gpioData.T2_In = 0;
  globalStructPtr->gpioData.T3_In = 0;
  globalStructPtr->gpioData.T4_In = 0;
  globalStructPtr->gpioData.B1_In = 0;
  globalStructPtr->gpioData.B2_In = 0;
  globalStructPtr->gpioData.Bx_In = 0;
  globalStructPtr->gpioData.By_In = 0;
  globalStructPtr->gpioData.Bz_In = 0;
  globalStructPtr->gpioData.Bw_In = 0;
  globalStructPtr->gpioData.J1x_In = 2048;
  globalStructPtr->gpioData.J1y_In = 2048;
  globalStructPtr->gpioData.J2x_In = 2048;
  globalStructPtr->gpioData.J2y_In = 2048;
  globalStructPtr->gpioData.P1_In = 0;
  globalStructPtr->gpioData.P2_In = 0;
}

// // Function to update global data (thread-safe)
// void updateGlobalData(int newValue1, float newValue2) {
//     std::lock_guard<std::mutex> lock(globalDataMutex);
//     if (globalDataPtr) {
//         globalDataPtr->value1 = newValue1;
//         globalDataPtr->value2 = newValue2;
//     }
// }

// // Function to read global data (thread-safe)
// GlobalData readGlobalData() {
//     std::lock_guard<std::mutex> lock(globalDataMutex);
//     return *globalDataPtr;
// }

