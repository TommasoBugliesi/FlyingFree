/*
  Filters.h - Library for Description
  Created by Tommaso Bugliesi, 21/08/2024.
  Brief description: The library integrates sensor fusion functions, filter and state estimation algorithms
*/

#ifndef Filters_h
#define Filters_h

#include <Arduino.h>

/**
 * @param p_data : input data time k+1
 * @param size : array size
 * @param w_c : cutoff frequency in Hz
 * @param dt : delta time between subsequent samples in us
 * @param p_out : pointer to output array
 * @brief The function applies to pointer to arrays with dimension of size. The filter is a low pass first order filter based on the transfer function H(s)=ω_c/(s+ω_c​)​​
 */
void lp_1st(float *p_data, size_t size, float w_c, unsigned long dt, float *p_out); 

#endif // Filters_h
