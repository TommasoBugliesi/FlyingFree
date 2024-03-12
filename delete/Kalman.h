#ifndef Kalman_h
#define Kalman_h

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <cmath> 
#include "Math.h" 

// DEBUG
// #include <Arduino.h>

class Kalman {
  public:
    Kalman(float delta_time=0.010); //Constructor declaration

    //General class functions
    void sensor2class(sensors_vec_t& acc, sensors_vec_t& gyr); 

    //Setter/Getter class functions
    const float* get_a() const;
    void set_a(float acc[3]);
    const float* get_g() const;
    void set_g(float gyr[3]);
    const float* get_angles() const;
    const float* get_angles_kalman() const;

  private:
    float delta_time = 0.010; //s

    float a[3] = {0,0,0};
    float g[3] = {0,0,0};
    float angles[3] = {0,0,0}; // RPY angles from Accelerometer
    float angles_kalman[3] = {0,0,0}; // RPW angle from Kalman

    int rows = 4;
    int cols = 4;

    // Dynamically allocate memory for the matrices
    float** F = new float*[rows];
    float** P = new float*[rows];
    float** Pplus = new float*[rows];
    float** Q = new float*[rows];
    float** H = new float*[rows];
    float** R = new float*[rows];
    float** K = new float*[rows];

    float** X = new float*[rows];
    float** Xplus = new float*[rows];
    float** Xcorr = new float*[rows];
    float** Y = new float*[rows];
    float** Z = new float*[rows];

    void compute_roll_pitch(void);  
    void compute_kalman_roll_pitch(void);
};

#endif