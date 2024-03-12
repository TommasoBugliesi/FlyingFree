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

class AHRS {
  private:
    float** F = new float*[AHRS_ROWS];
    float** K = new float*[AHRS_ROWS];

    float** X_11 = new float*[AHRS_ROWS];
    float** X_21 = new float*[AHRS_ROWS];
    float** X_22 = new float*[AHRS_ROWS];
    float** Y = new float*[AHRS_ROWS];
    float** Z = new float*[AHRS_ROWS];

    float gyroData[3];
    float accData[3];
    float angData[3];  // Angle in rad Roll and Pitch from sensors
    float angOut[3];   // Angle output in rad
    float dt;

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
    void computeKalman();

  private:
    // Add private variables and methods here
};

#endif // AHRS_h
