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