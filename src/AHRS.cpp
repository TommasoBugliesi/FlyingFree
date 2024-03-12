/*
  AHRS.cpp - Library for Description
  Created by Tommaso Bugliesi, 03/03/2024.
  Brief description: The library implement filters to estimate the drone orientation from its sensors
*/

#include "AHRS.h"
#include "Global.h"
#include <cmath> 

AHRS::AHRS() {
    // Allocate memory for matrices
  for (size_t i = 0; i < AHRS_ROWS; ++i) {
    F[i] = new float[AHRS_COLS];
    K[i] = new float[AHRS_COLS];
  }

  for (size_t i = 0; i < AHRS_ROWS; ++i) {
    X_11[i] = new float[1];
    X_21[i] = new float[1];
    X_22[i] = new float[1];
    Z[i] = new float[1];
    Y[i] = new float[1];
  }

  // Init values
  F[0][0] = 1.; F[0][1] = 0; F[0][2] = 0.; F[0][3] = 0.; 
  F[1][0] = 0.; F[1][1] = 1.; F[1][2] = 0.; F[1][3] = 0.;
  F[2][0] = 0.; F[2][1] = 0.; F[2][2] = 0.; F[2][3] = 0.; 
  F[3][0] = 0.; F[3][1] = 0.; F[3][2] = 0.; F[3][3] = 0.;

  K[0][0] = 0.095; K[0][1] = 0.; K[0][2] = 0.; K[0][3] = 0.; 
  K[1][0] = 0.; K[1][1] = 0.095; K[1][2] = 0.; K[1][3] = 0.; 
  K[2][0] = 0.; K[2][1] = 0.; K[2][2] = 0.75; K[2][3] = 0.; 
  K[3][0] = 0.; K[3][1] = 0.; K[3][2] = 0.; K[3][3] = 0.75;  

  X_11[0][0] = 0.; X_11[1][0] = 0.; X_11[2][0] = 0.; X_11[3][0] = 0.; 
  X_21[0][0] = 0.; X_21[1][0] = 0.; X_21[2][0] = 0.; X_21[3][0] = 0.; 
  X_22[0][0] = 0.; X_22[1][0] = 0.; X_22[2][0] = 0.; X_22[3][0] = 0.; 
  Y[0][0] = 0.; Y[1][0] = 0.; Y[2][0] = 0.; Y[3][0] = 0.; 
  Z[0][0] = 0.; Z[1][0] = 0.; Z[2][0] = 0.; Z[3][0] = 0.;

  memset(gyroData, 0, sizeof(gyroData));
  memset(accData, 0, sizeof(accData));
  memset(angData, 0, sizeof(angData));
  memset(angOut, 0, sizeof(angOut));
}

AHRS::~AHRS() {
  // Destructor implementation
}

// Add private method implementations here

// Add public method implementations here
void AHRS::computeKalman(){
  memcpy(gyroData, globalStructPtr->bmi323Data.gyroData, sizeof(globalStructPtr->bmi323Data.gyroData));
  memcpy(accData, globalStructPtr->bmi323Data.accData, sizeof(globalStructPtr->bmi323Data.accData));

  angData[0] = atan2(accData[1], accData[2]);
  angData[1] = atan2(-accData[0], sqrt(accData[1]*accData[1] + accData[2]*accData[2]));
  dt = globalStructPtr->task1Timing / 1000000; // delta time TODO: take task time 

  F[0][2] = dt*cos(angOut[1])*cos(angOut[2]);
  F[0][3] = -dt*sin(angOut[2]);
  F[1][2] = dt*sin(angOut[2])*cos(angOut[1]);
  F[1][3] = dt*cos(angOut[2]);

  Z[0][0] = angData[0]; 
  Z[1][0] = angData[1]; 
  Z[2][0] = gyroData[0]; 
  Z[3][0] = gyroData[1];

  // State Prediction
  X_21[0][0] = F[0][0]*X_22[0][0] + F[0][1]*X_22[0][0] + F[0][2]*X_22[0][0] + F[0][3]*X_22[0][0];
  X_21[1][0] = F[1][0]*X_22[1][0] + F[1][1]*X_22[1][0] + F[1][2]*X_22[1][0] + F[1][3]*X_22[1][0];
  X_21[2][0] = F[2][0]*X_22[2][0] + F[2][1]*X_22[2][0] + F[2][2]*X_22[2][0] + F[2][3]*X_22[2][0];
  X_21[3][0] = F[3][0]*X_22[3][0] + F[3][1]*X_22[3][0] + F[3][2]*X_22[3][0] + F[3][3]*X_22[3][0];

  // Measure Error 
  Y[0][0] = Z[0][0]-X_21[0][0];
  Y[1][0] = Z[1][0]-X_21[1][0];
  Y[2][0] = Z[2][0]-X_21[2][0];
  Y[3][0] = Z[3][0]-X_21[3][0];

  // State update
  X_11[0][0] = K[0][0]*Y[0][0] + K[0][1]*Y[0][0] + K[0][2]*Y[0][0] + K[0][3]*Y[0][0];
  X_11[1][0] = K[1][0]*Y[1][0] + K[1][1]*Y[1][0] + K[1][2]*Y[1][0] + K[1][3]*Y[1][0];
  X_11[2][0] = K[2][0]*Y[2][0] + K[2][1]*Y[2][0] + K[2][2]*Y[2][0] + K[2][3]*Y[2][0];
  X_11[3][0] = K[3][0]*Y[3][0] + K[3][1]*Y[3][0] + K[3][2]*Y[3][0] + K[3][3]*Y[3][0];

  X_22[0][0] = X_11[0][0]+X_21[0][0];
  X_22[1][0] = X_11[1][0]+X_21[1][0];
  X_22[2][0] = X_11[2][0]+X_21[2][0];
  X_22[3][0] = X_11[3][0]+X_21[3][0];

  // Update global pointer
  memcpy(globalStructPtr->ahrsData.angDataPrv, angOut, sizeof(angOut)); // Copy first 3 elements

  angOut[0] = X_22[0][0];
  angOut[1] = X_22[1][0];
  angOut[2] = 0.;

  memcpy(globalStructPtr->ahrsData.angData, angOut, sizeof(angOut)); // Copy first 3 elements
}
