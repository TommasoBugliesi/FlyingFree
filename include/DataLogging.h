/*
  DataLogging.h - Library for Description
  Created by Tommaso Bugliesi, 02/09/2024.
  Brief description: Logics to access the MCU via UART with an external device on the I2C pins.
*/

#ifndef DataLogging_h
#define DataLogging_h

#include <Arduino.h>

#define RX_PIN GPIO_NUM_21  // Originally I2C SDA
#define TX_PIN GPIO_NUM_22  // Originally I2C SCL
#define BAUD_RATE 115200
#define BUFFER_SIZE_BIT 1024 // bits
#define BUFFER_SIZE_BYTE 128 // bytes
#define TICK_TO_WAIT 

#define TWO_TO_15 32768.0f

/**
 * Init the UART bus on the I2C channel for logging purposes
 */
void initUart();

/**
 * Send a UART message through the UART port without waiting for the command to complete the transfer
 */
int sendUart();

/**
 * Encode float values into 16 bit sequence 
 */
void encode_float16(float value, float min, float max, char* buffer);

#endif // DataLogging_h
