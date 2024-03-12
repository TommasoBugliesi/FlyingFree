// Kalman.cpp
#include "Kalman.h"
/*
 * Kalman Filter Equations:
 * ------------------------
 * This code includes the equations for a discrete-time linear Kalman filter.
 * The filter operates on a system with a linear dynamic model and linear measurement model.
 *
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
 *    P̂ₖ|ₖ = (I - Kₖ Hₖ) P̂ₖ|ₖ₋₁
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
 * - I: Identity matrix.
 * - P̂ₖ|ₖ: Updated error covariance matrix.
 *
 * Note: Adjust matrices (Fₖ, Bₖ, Hₖ, Qₖ, Rₖ) based on the characteristics of the specific system being modeled.
 */

// Constructor definition
Kalman::Kalman(float delta_time) : delta_time(delta_time) {
  this->delta_time = delta_time;

  // Allocate memory for matrices
  for (int i = 0; i < rows; ++i) {
    F[i] = new float[cols];
    P[i] = new float[cols];
    Pplus[i] = new float[cols];
    Q[i] = new float[cols];
    H[i] = new float[cols];
    R[i] = new float[cols];
    K[i] = new float[cols];
  }

  for (int i = 0; i < rows; ++i) {
    X[i] = new float[1];
    Xplus[i] = new float[1];
    Xcorr[i] = new float[1];
    Z[i] = new float[1];
    Y[i] = new float[1];
  }

  // Init values
  F[0][0] = 1.; F[0][1] = 0; F[0][2] = 0.; F[0][3] = 0.; 
  F[1][0] = 0.; F[1][1] = 1.; F[1][2] = 0.; F[1][3] = 0.;
  F[2][0] = 0.; F[2][1] = 0.; F[2][2] = 0.; F[2][3] = 0.; 
  F[3][0] = 0.; F[3][1] = 0.; F[3][2] = 0.; F[3][3] = 0.;

  P[0][0] = 0.1; P[0][1] = 0.; P[0][2] = 0.; P[0][3] = 0.;
  P[1][0] = 0.; P[1][1] = 0.1; P[1][2] = 0.; P[1][3] = 0.;
  P[2][0] = 0.; P[2][1] = 0.; P[2][2] = 0.1; P[2][3] = 0.;
  P[3][0] = 0.; P[3][1] = 0.; P[3][2] = 0.; P[3][3] = 0.1; 

  Pplus[0][0] = 0.1; Pplus[0][1] = 0.; Pplus[0][2] = 0.; Pplus[0][3] = 0.;
  Pplus[1][0] = 0.; Pplus[1][1] = 0.1; Pplus[1][2] = 0.; Pplus[1][3] = 0.;
  Pplus[2][0] = 0.; Pplus[2][1] = 0.; Pplus[2][2] = 0.1; Pplus[2][3] = 0.;
  Pplus[3][0] = 0.; Pplus[3][1] = 0.; Pplus[3][2] = 0.; Pplus[3][3] = 0.1; 

  Q[0][0] = 0.1; Q[0][1] = 0.; Q[0][2] = 0.; Q[0][3] = 0.;
  Q[1][0] = 0.; Q[1][1] = 0.05; Q[1][2] = 0.; Q[1][3] = 0.;
  Q[2][0] = 0.; Q[2][1] = 0.; Q[2][2] = 0.05; Q[2][3] = 0.;
  Q[3][0] = 0.; Q[3][1] = 0.; Q[3][2] = 0.; Q[3][3] = 0.2;

  H[0][0] = 1.; H[0][1] = 0.; H[0][2] = 0.; H[0][3] = 0.;
  H[1][0] = 0.; H[1][1] = 1.; H[1][2] = 0.; H[1][3] = 0.;
  H[2][0] = 0.; H[2][1] = 0.; H[2][2] = 1.; H[2][3] = 0.;
  H[3][0] = 0.; H[3][1] = 0.; H[3][2] = 0.; H[3][3] = 1.;

  R[0][0] = 2; R[0][1] = 0.; R[0][2] = 0.; R[0][3] = 0.;
  R[1][0] = 0.; R[1][1] = 4.; R[1][2] = 0.; R[1][3] = 0.;
  R[2][0] = 0.; R[2][1] = 0.; R[2][2] = 1.; R[2][3] = 0.;
  R[3][0] = 0.; R[3][1] = 0.; R[3][2] = 0.; R[3][3] = 1.;

  K[0][0] = 0.; K[0][1] = 0.; K[0][2] = 0.; K[0][3] = 0.; 
  K[1][0] = 0.; K[1][1] = 0.; K[1][2] = 0.; K[1][3] = 0.; 
  K[2][0] = 0.; K[2][1] = 0.; K[2][2] = 0.; K[2][3] = 0.; 
  K[3][0] = 0.; K[3][1] = 0.; K[3][2] = 0.; K[3][3] = 0.;  

  X[0][0] = 0.; X[1][0] = 0.; X[2][0] = 0.; X[3][0] = 0.; 
  Xplus[0][0] = 0.; Xplus[1][0] = 0.; Xplus[2][0] = 0.; Xplus[3][0] = 0.; 
  Xcorr[0][0] = 0.; Xcorr[1][0] = 0.; Xcorr[2][0] = 0.; Xcorr[3][0] = 0.; 
  Y[0][0] = 0.; Y[1][0] = 0.; Y[2][0] = 0.; Y[3][0] = 0.; 
  Z[0][0] = 0.; Z[1][0] = 0.; Z[2][0] = 0.; Z[3][0] = 0.;
}

// Member function definitions

void Kalman::compute_roll_pitch(void) {
      angles[0] = atan2(a[1], a[2]) * 180.0 / M_PI;
      angles[1] = atan2(-a[0], sqrt(a[1]*a[1] + a[2]*a[2])) * 180.0 / M_PI;
}

void Kalman::compute_kalman_roll_pitch(void) {
      // Update Variables
      /*
      F[4][4] = [[1,0,dt*cos(θ_y)*cos(θ_z),-dt*sin(θ_z)],
                [0,1,dt*sin(θ_z)cos(θ_y),dt*cos(θ_z)],
                [0,0,0,0],
                [0,0,0,0]]
      */
      F[0][2] = delta_time*cos(angles_kalman[1]/180*M_PI)*cos(angles_kalman[2]/180*M_PI);
      F[0][3] = -delta_time*sin(angles_kalman[2]/180*M_PI);
      F[1][2] = delta_time*sin(angles_kalman[2]/180*M_PI)*cos(angles_kalman[1]/180*M_PI);
      F[1][3] = delta_time*cos(angles_kalman[2]/180*M_PI);

      Z[0][0] = angles[0]; Z[1][0] = angles[1]; Z[2][0] = g[0]; Z[3][0] = g[1];

      // State Prediction
      matrix_multiply(F, rows, cols, X, rows, 1, Xplus); // Xplus

      matrix_multiply_transpose(F, rows, cols, P, rows, cols, Pplus);
      matrix_addition(Pplus, rows, cols, Q, rows, cols, Pplus); // Pplus

      // Measure Error 
      matrix_subtraction(Z, rows, 1, Xplus, rows, 1, Y); // Y

      // Kalman gain
      float** tmp1 = new float*[rows];
      float** tmp2 = new float*[rows];
      for (int i = 0; i < rows; ++i) {
        tmp1[i] = new float[cols];
        tmp2[i] = new float[cols];
      }

      matrix_addition(Pplus, rows, cols, R, rows, cols, tmp1); 
      matrix_inversion(tmp1, rows, tmp2);
      matrix_multiply(Pplus, rows, cols, tmp2, rows, cols, K); // K

      K[0][0] = 0.095; K[0][1] = 0.; K[0][2] = 0.; K[0][3] = 0.; 
      K[1][0] = 0.; K[1][1] = 0.095; K[1][2] = 0.; K[1][3] = 0.; 
      K[2][0] = 0.; K[2][1] = 0.; K[2][2] = 0.75; K[2][3] = 0.; 
      K[3][0] = 0.; K[3][1] = 0.; K[3][2] = 0.; K[3][3] = 0.75; 
      
      for (int i = 0; i < rows; ++i) {
        delete[] tmp1[i];
        delete[] tmp2[i];
      }
      delete[] tmp1;
      delete[] tmp2;

      // State update
      matrix_multiply(K, rows, cols, Y, rows, 1, Xcorr); // Correction to estimate state
      matrix_addition(Xplus, rows, 1, Xcorr, rows, 1, X); // New X stat: include kalman angles
      angles_kalman[0] = X[0][0];
      angles_kalman[1] = X[1][0];

      // Covariance update
      matrix_subtraction(H, rows, cols, K, rows, cols, K); // Pplus
      matrix_multiply(K, rows, cols, Pplus, rows, cols, P); // Correction to estimate state
}

void Kalman::sensor2class(sensors_vec_t& acc, sensors_vec_t& gyr) {
      set_a(acc.v);
      set_g(gyr.v);
      // set_m();
      compute_roll_pitch();
      compute_kalman_roll_pitch();
}

const float* Kalman::get_a() const {
    return a;
}

void Kalman::set_a(float acc[3]) {
        a[0] = acc[0];
        a[1] = acc[1];
        a[2] = acc[2];
}

const float* Kalman::get_g() const {
    return g;
}

void Kalman::set_g(float gyr[3]) {    
        g[0] = gyr[0]*180/M_PI; // Convert to deg/s
        g[1] = gyr[1]*180/M_PI;
        g[2] = gyr[2]*180/M_PI;
}

const float* Kalman::get_angles() const {
    return angles;
}

const float* Kalman::get_angles_kalman() const {
    return angles_kalman;
}

/*
// Destructor definition (if needed)
Kalman::~Kalman() {
    // ... implementation of the destructor ...
}
*/