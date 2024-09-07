/*
  AHRS.h - Library for Description
  Created by Tommaso Bugliesi, 03/03/2024.
  Brief description: The library implement filters to estimate the drone orientation from its sensors.
*/

#ifndef AHRS_h
#define AHRS_h

#include <Arduino.h>

#define AHRS_ROWS 4
#define AHRS_COLS 4
#define BUFFERSIZE 5

class AHRS {
  private:
    // Kalman filter
    float** F = new float*[AHRS_ROWS];
    float** K = new float*[AHRS_ROWS];

    float** X_11 = new float*[AHRS_ROWS];
    float** X_21 = new float*[AHRS_ROWS];
    float** X_22 = new float*[AHRS_ROWS];
    float** Y = new float*[AHRS_ROWS];
    float** Z = new float*[AHRS_ROWS];

    float angDataKalman[3] = {0.0f, 0.0f, 0.0f};  // Angle in rad Roll and Pitch from sensors
    float angOutKalman[3] = {0.0f, 0.0f, 0.0f};   // Angle output in rad

    // Mahony filter
    float qMahony[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // Initial quaternion: [w, x, y, z]
    float kiMahony = 0.01f; // Integral gain (tune this value)
    float kpMahony = 2.0f; // Prop gain (tune this value)
    float integraleMahony[3] = {0.0f, 0.0f, 0.0f};

    float angOutMahony[3] = {0.0f, 0.0f, 0.0f};   // Angle output in rad
    
    // Temporary variables inside each algorithm 
    float gyroData[3] = {0.0f, 0.0f, 0.0f};
    float accData[3] = {0.0f, 0.0f, 0.0f};
    float dt;

    // Filter
    int samples = 0;
    float angDataFilt[3][BUFFERSIZE];

  public:
    AHRS(); // Constructor
    ~AHRS(); // Destructor

    // Add public methods here
    /**
     * @brief Compute roll and pitch angles with Kalman filter. 
     * This code includes the equations for a discrete-time linear Kalman filter.
     * The filter operates on a system with a linear dynamic model and linear measurement model.
     * Prediction Step (Time Update):
     * ------------------------------
     * 1. State Prediction:
     *    x̂ₖ|ₖ₋₁ = Fₖ x̂ₖ₋₁|ₖ₋₁ + Bₖ uₖ
     * 2. Error Covariance Prediction:
     *    P̂ₖ|ₖ₋₁ = Fₖ P̂ₖ₋₁|ₖ₋₁ Fₖᵀ + Qₖ
     *
     * Update Step (Measurement Update):
     * ------------------------------
     * 3. Measurement Residual:
     *    yₖ = zₖ - Hₖ x̂ₖ|ₖ₋₁
     * 4. Kalman Gain:
     *    Kₖ = P̂ₖ|ₖ₋₁ Hₖᵀ (Hₖ P̂ₖ|ₖ₋₁ Hₖᵀ + Rₖ)⁻¹
     * 5. State Update:
     *    x̂ₖ|ₖ = x̂ₖ|ₖ₋₁ + Kₖ yₖ
     * 6. Covariance Update:
     *    P̂ₖ|ₖ = (Id - Kₖ Hₖ) P̂ₖ|ₖ₋₁
     *
     * Definitions:
     * ------------
     * - x̂ₖ|ₖ₋₁: Predicted state vector at time step k given measurements up to time step k-1.
     * - Fₖ: State transition matrix.
     * - Bₖ: Control-input matrix (if control input uₖ is present).
     * - uₖ: Control input.
     * - P̂ₖ|ₖ₋₁: Predicted error covariance matrix.
     * - P̂ₖ₋₁|ₖ₋₁: Error covariance matrix at time step k-1.
     * - Qₖ: Process noise covariance matrix.
     * - yₖ: Measurement residual (difference between the actual measurement zₖ and the predicted measurement Hₖ x̂ₖ|ₖ₋₁).
     * - Hₖ: Measurement matrix.
     * - Kₖ: Kalman gain.
     * - Rₖ: Measurement noise covariance matrix.
     * - x̂ₖ|ₖ: Updated state estimate.
     * - Id: Identity matrix.
     * - P̂ₖ|ₖ: Updated error covariance matrix.
     *
     * Note: Adjust matrices (Fₖ, Bₖ, Hₖ, Qₖ, Rₖ) based on the characteristics of the specific system being modeled.
     */
    void computeKalman(float *p_acc, float *p_gyro);

    /**
     * @brief Mahony filter for computing roll, pitch, and yaw using accelerometer and gyroscope data. 
     * The Mahony filter is a sensor fusion algorithm that combines accelerometer and gyroscope data to estimate orientation.
     * It leverages proportional and integral feedback mechanisms to correct for gyroscope drift and improve accuracy.
     *
     * Algorithm Overview:
     * ------------------------------
     * 1. Normalize accelerometer data:
     *    The accelerometer data is first normalized to remove any magnitude effects and focus on the direction.
     * 
     * 2. Compute estimated gravity direction from quaternion:
     *    The quaternion is used to calculate the expected direction of gravity in the sensor frame.
     *
     * 3. Compute the error between measured and estimated gravity:
     *    The error is computed by taking the cross product between the measured gravity (from the accelerometer) and the estimated gravity.
     *
     * 4. Apply proportional and integral feedback:
     *    Proportional feedback corrects for the orientation error, while integral feedback helps correct gyroscope bias over time.
     * 
     * 5. Update quaternion:
     *    The quaternion is updated by integrating the corrected gyroscope data over time, which represents the rate of change of orientation.
     *
     * 6. Normalize quaternion:
     *    The quaternion is normalized after each update step to maintain stability and avoid numerical drift.
     *
     */
    void computeMahony(float *p_acc, float *p_gyro);

    // Perform a weighted average of the input angles (to reduce angle deviation
    void weightedAverageFilt();

  private:
    // Add private variables and methods here
    void quat2euler(float *q, float *out);
};

#endif // AHRS_h
