/*
  Filters.cpp - Library for Description
  Created by Tommaso Bugliesi, 21/08/2024.
  Brief description: The library integrates sensor fusion functions, filter and state estimation algorithms
*/

#include "Filters.h"

void lp_1st(float *p_data, size_t size, float w_c, unsigned long dt, float *p_out){
  // Calculate alpha
  float dt_sec = dt / 1000000.0f;  // Convert dt from microseconds to seconds
  float alpha = (w_c * dt_sec) / (w_c * dt_sec + 2.0f);

  // Apply the filter to each element in the input array
  for (size_t k = 0; k < size; k++) {
      p_out[k] = (1 - alpha) * p_out[k] + alpha * p_data[k];
  }
}

void hp_1st(float *p_data, float *p_prv, size_t size, float w_c, unsigned long dt, float *p_out){
    // Calculate alpha
    float dt_sec = dt / 1000000.0f;  // Convert dt from microseconds to seconds
    float alpha = (w_c * dt_sec) / (w_c * dt_sec + 2.0f);

    // Apply the filter to each element in the input array
    for (size_t k = 0; k < size; k++) {
        p_out[k] = alpha * (p_out[k] + p_data[k] - p_prv[k]);
        p_prv[k] = p_out[k];  // Update the previous value for the next iteration
    }
}