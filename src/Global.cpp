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

