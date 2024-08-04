/*
  Global.h - Library for Description
  Created by Tommaso Bugliesi, 03/03/2024.
  Brief description: Global variables header file.
*/

#ifndef Global_h
#define Global_h

#include <Arduino.h>
#include <freertos/semphr.h>

struct BMI323Struct {
  float gyroData[3];
  float gyroDataPrv[3];
  float accData[3];
  float accDataPrv[3];
};

struct AHRSStruct {
  float angData[3];
  float angDataPrv[3];
};

struct GPIOStruct {
  bool T1_In;
  bool T2_In;
  bool T3_In;
  bool T4_In;
  bool B1_In;
  bool B2_In;
  bool Bx_In;
  bool By_In;
  bool Bz_In;
  bool Bw_In;
  uint16_t J1x_In;
  uint16_t J1y_In;
  uint16_t J2x_In;
  uint16_t J2y_In;
  uint16_t P1_In;
  uint16_t P2_In;
};

struct MotorStruct {
  float KpRoll;
  float KiRoll;
  float KdRoll;
  float KpPitch;
  float KiPitch;
  float KdPitch;
};

struct GlobalStruct {
    BMI323Struct bmi323Data;
    AHRSStruct   ahrsData;
    GPIOStruct   gpioData;
    MotorStruct  motorData;
    float task1Frequency;
    float task2Frequency; 
    float task1Timing;  // us
    float task2Timing;  // us
};

// Declare the global pointer to GlobalData
extern GlobalStruct* globalStructPtr;

// Declare the global mutex as extern in the header file
extern SemaphoreHandle_t globalMutex;

/* Function declaration to initialize globalDataPtr */
void initGlobal();


#endif // Global_h
