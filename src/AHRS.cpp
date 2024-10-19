/*
  AHRS.cpp - Library for Description
  Created by Tommaso Bugliesi, 03/03/2024.
  Brief description: The library implement filters to estimate the drone orientation from its sensors
*/

#include "AHRS.h"
#include "Global.h"
#include <cmath> 
//test
AHRS::AHRS() {
  // Kalman fitler  
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

    K[0][0] = 0.2; K[0][1] = 0.; K[0][2] = 0.; K[0][3] = 0.; 
    K[1][0] = 0.; K[1][1] = 0.2; K[1][2] = 0.; K[1][3] = 0.; 
    K[2][0] = 0.; K[2][1] = 0.; K[2][2] = 0.8; K[2][3] = 0.; 
    K[3][0] = 0.; K[3][1] = 0.; K[3][2] = 0.; K[3][3] = 0.8;  

    X_11[0][0] = 0.; X_11[1][0] = 0.; X_11[2][0] = 0.; X_11[3][0] = 0.; 
    X_21[0][0] = 0.; X_21[1][0] = 0.; X_21[2][0] = 0.; X_21[3][0] = 0.; 
    X_22[0][0] = 0.; X_22[1][0] = 0.; X_22[2][0] = 0.; X_22[3][0] = 0.; 
    Y[0][0] = 0.; Y[1][0] = 0.; Y[2][0] = 0.; Y[3][0] = 0.; 
    Z[0][0] = 0.; Z[1][0] = 0.; Z[2][0] = 0.; Z[3][0] = 0.;

  // Filter
    // Initialize angDataFilt with zeros
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < BUFFERSIZE; ++j) {
            angDataFilt[i][j] = 0.0f;
        }
    }
}

AHRS::~AHRS() {
  // Destructor implementation
}

// Add private method implementations here
void AHRS::quat2euler(float *q, float *out){
  // Convert quaternion to Euler angles (roll, pitch, yaw)
  out[0] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])); // Roll
  out[1] = asin(2.0f * (q[0] * q[2] - q[3] * q[1]));  // Pitch
  out[2] = atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));  // Yaw
}

// Add public method implementations here
void AHRS::computeKalman(float *p_acc, float *p_gyro){
  memcpy(gyroData, p_gyro, sizeof(gyroData));
  memcpy(accData, p_acc, sizeof(accData));
  dt = globalStructPtr->ahrsTiming / 1000000;  

  angDataKalman[0] = atan2(accData[1], accData[2]);
  angDataKalman[1] = atan2(-accData[0], sqrt(accData[1]*accData[1] + accData[2]*accData[2]));

  F[0][2] = dt*cos(angOutKalman[1])*cos(angOutKalman[2]);
  F[0][3] = -dt*sin(angOutKalman[2]);
  F[1][2] = dt*sin(angOutKalman[2])*cos(angOutKalman[1]);
  F[1][3] = dt*cos(angOutKalman[2]);

  Z[0][0] = angDataKalman[0]; 
  Z[1][0] = angDataKalman[1]; 
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
  globalStructPtr->ahrsDataKalman.angDataPrv[0] = -angOutKalman[1];
  globalStructPtr->ahrsDataKalman.angDataPrv[1] = angOutKalman[0];
  globalStructPtr->ahrsDataKalman.angDataPrv[2] = angOutKalman[2];

  // Define new angle for the next iteration
  angOutKalman[0] = X_22[0][0];
  angOutKalman[1] = X_22[1][0];
  angOutKalman[2] = 0.;

  // Convert sensor frame to drone frame 
  globalStructPtr->ahrsDataKalman.angData[0] = -angOutKalman[1];
  globalStructPtr->ahrsDataKalman.angData[1] = angOutKalman[0];
  globalStructPtr->ahrsDataKalman.angData[2] = angOutKalman[2];
}

void AHRS::computeMahony(float *p_acc, float *p_gyro){
  memcpy(gyroData, p_gyro, sizeof(gyroData));
  memcpy(accData, p_acc, sizeof(accData));
  dt = globalStructPtr->ahrsTiming / 1000000;  

  // Normalize accelerometer data
  float accNorm = sqrt(accData[0] * accData[0] + accData[1] * accData[1] + accData[2] * accData[2]);
  accData[0] /= accNorm;
  accData[1] /= accNorm;
  accData[2] /= accNorm;

  // Compute expected direction of gravity (from the current quaternion)
  float vx = 2.0f * (qMahony[1] * qMahony[3] - qMahony[0] * qMahony[2]);
  float vy = 2.0f * (qMahony[0] * qMahony[1] + qMahony[2] * qMahony[3]);
  float vz = qMahony[0] * qMahony[0] - qMahony[1] * qMahony[1] - qMahony[2] * qMahony[2] + qMahony[3] * qMahony[3];

  // Error is the cross product between estimated and measured gravity
  float ex = (accData[1] * vz - accData[2] * vy);
  float ey = (accData[2] * vx - accData[0] * vz);
  float ez = (accData[0] * vy - accData[1] * vx);

  // Apply integral feedback if enabled (helps to reduce gyro bias)
  integraleMahony[0] += kiMahony * ex * dt;  // dt is time step (delta time)
  integraleMahony[1] += kiMahony * ey * dt;
  integraleMahony[2] += kiMahony * ez * dt;

  // Apply proportional feedback
  float gx = gyroData[0] + kpMahony * ex + integraleMahony[0];  // Corrected gyroscope x
  float gy = gyroData[1] + kpMahony * ey + integraleMahony[1];  // Corrected gyroscope y
  float gz = gyroData[2] + kpMahony * ez + integraleMahony[2];  // Corrected gyroscope z

  // Quaternion derivative due to gyroscope (based on corrected gyroscope data)
  float qDot[4];
  qDot[0] = 0.5f * (-qMahony[1] * gx - qMahony[2] * gy - qMahony[3] * gz);
  qDot[1] = 0.5f * ( qMahony[0] * gx + qMahony[2] * gz - qMahony[3] * gy);
  qDot[2] = 0.5f * ( qMahony[0] * gy - qMahony[1] * gz + qMahony[3] * gx);
  qDot[3] = 0.5f * ( qMahony[0] * gz + qMahony[1] * gy - qMahony[2] * gx);

  // Integrate quaternion rate of change
  qMahony[0] += qDot[0] * dt;
  qMahony[1] += qDot[1] * dt;
  qMahony[2] += qDot[2] * dt;
  qMahony[3] += qDot[3] * dt;

  // Normalize quaternion to avoid drift
  float norm = sqrt(qMahony[0] * qMahony[0] + qMahony[1] * qMahony[1] + qMahony[2] * qMahony[2] + qMahony[3] * qMahony[3]);
  qMahony[0] /= norm;
  qMahony[1] /= norm;
  qMahony[2] /= norm;
  qMahony[3] /= norm;

  // Update global pointer
  globalStructPtr->ahrsDataMahony.angDataPrv[0] = -angOutMahony[1];
  globalStructPtr->ahrsDataMahony.angDataPrv[1] = angOutMahony[0];
  globalStructPtr->ahrsDataMahony.angDataPrv[2] = angOutMahony[2];

  // Quaternion to Euler
  quat2euler(qMahony, angOutMahony);

  // Convert sensor frame to drone frame 
  globalStructPtr->ahrsDataMahony.angData[0] = -angOutMahony[1];
  globalStructPtr->ahrsDataMahony.angData[1] = angOutMahony[0];
  globalStructPtr->ahrsDataMahony.angData[2] = angOutMahony[2];
}

void AHRS::weightedAverageFilt(){
  // Update angDataFilt
  if (samples < BUFFERSIZE) {
      angDataFilt[0][samples] = -angOutKalman[1];
      angDataFilt[1][samples] = angOutKalman[0];
      angDataFilt[2][samples] = angOutKalman[2];
      samples++;
  } else {
      // Buffer is full, remove the oldest sample
      for (int i = 0; i < BUFFERSIZE - 1; ++i) {
          angDataFilt[0][i] = angDataFilt[0][i + 1];
          angDataFilt[1][i] = angDataFilt[1][i + 1];
          angDataFilt[2][i] = angDataFilt[2][i + 1];
      }
      angDataFilt[0][BUFFERSIZE - 1] = -angOutKalman[1];
      angDataFilt[1][BUFFERSIZE - 1] = angOutKalman[0];
      angDataFilt[2][BUFFERSIZE - 1] = angOutKalman[2];
  }

  if (samples>0) {
    float sumRoll = 0, sumPitch = 0, sumYaw = 0;
    for (int i = 0; i < samples; ++i) {
        sumRoll += angDataFilt[0][i];
        sumPitch += angDataFilt[1][i];
        sumYaw += angDataFilt[2][i];
    }
    globalStructPtr->ahrsDataFilt.angData[0] = sumRoll / samples;
    globalStructPtr->ahrsDataFilt.angData[1] = sumPitch / samples;
    globalStructPtr->ahrsDataFilt.angData[2] = sumYaw / samples;
  }
} 