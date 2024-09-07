/*
  DataLogging.cpp - Library for Description
  Created by Tommaso Bugliesi, 02/09/2024.
  Brief description: Logics to access the MCU via UART with an external device on the I2C pins.
*/

#include "DataLogging.h"

#include "driver/uart.h"
#include "Global.h"


float accRangeMin = -40.0f;
float accRangeMax = 40.0f;
float gyroRangeMin = -10.0f;
float gyroRangeMax = 10.0f;
float angRangeMin = -3.14f;
float angRangeMax = 3.14f;


void initUart(){
    // UART configuration
    const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      // .rx_flow_ctrl_thresh = 122,
    };

    // Configure UART parameters
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Install the UART driver, without an RX buffer and without FreeRTOS queue
    esp_err_t ret = uart_driver_install(UART_NUM_2, BUFFER_SIZE_BIT, BUFFER_SIZE_BIT, 0, NULL, 0);
    if (ret != ESP_OK) {
      Serial.println("UART driver not installed properly");
    }

    // Set RX pin to output mode and drive it low
    pinMode(RX_PIN, OUTPUT);
    digitalWrite(RX_PIN, LOW); // Connect to the ground of the raspberry pi, and Tx of ESP32 to the Rx of the raspberry pi
}

int sendUart(){
  char data[BUFFER_SIZE_BYTE];
  size_t dataSize = 0;

  char buffer_16[2];

  // Encode accelerations
  // encode_float16(globalStructPtr->bmi323Data.accData[0], accRangeMin, accRangeMax, buffer_16);
  // memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  // dataSize += sizeof(buffer_16);
  // encode_float16(globalStructPtr->bmi323Data.accData[1], accRangeMin, accRangeMax, buffer_16);
  // memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  // dataSize += sizeof(buffer_16);
  // encode_float16(globalStructPtr->bmi323Data.accData[2], accRangeMin, accRangeMax, buffer_16);
  // memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  // dataSize += sizeof(buffer_16);

  // Encode gyro
  // encode_float16(globalStructPtr->bmi323Data.gyroData[0], gyroRangeMin, gyroRangeMax, buffer_16);
  // memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  // dataSize += sizeof(buffer_16);
  // encode_float16(globalStructPtr->bmi323Data.gyroData[1], gyroRangeMin, gyroRangeMax, buffer_16);
  // memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  // dataSize += sizeof(buffer_16);
  // encode_float16(globalStructPtr->bmi323Data.gyroData[2], gyroRangeMin, gyroRangeMax, buffer_16);
  // memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  // dataSize += sizeof(buffer_16);
  
  // encode_float16(globalStructPtr->bmi323Data_lp1st.gyroData[0], gyroRangeMin, gyroRangeMax, buffer_16);
  // memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  // dataSize += sizeof(buffer_16);
  // encode_float16(globalStructPtr->bmi323Data_lp1st.gyroData[1], gyroRangeMin, gyroRangeMax, buffer_16);
  // memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  // dataSize += sizeof(buffer_16);
  // encode_float16(globalStructPtr->bmi323Data_lp1st.gyroData[2], gyroRangeMin, gyroRangeMax, buffer_16);
  // memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  // dataSize += sizeof(buffer_16);

  // Encode ang
  encode_float16(globalStructPtr->ahrsDataKalman.angData[0], angRangeMin, angRangeMax, buffer_16);
  memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  dataSize += sizeof(buffer_16);
  encode_float16(globalStructPtr->ahrsDataKalman.angData[1], angRangeMin, angRangeMax, buffer_16);
  memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  dataSize += sizeof(buffer_16);
  encode_float16(globalStructPtr->ahrsDataKalman.angData[2], angRangeMin, angRangeMax, buffer_16);
  memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  dataSize += sizeof(buffer_16);
  
  encode_float16(globalStructPtr->ahrsDataMahony.angData[0], angRangeMin, angRangeMax, buffer_16);
  memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  dataSize += sizeof(buffer_16);
  encode_float16(globalStructPtr->ahrsDataMahony.angData[1], angRangeMin, angRangeMax, buffer_16);
  memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  dataSize += sizeof(buffer_16);
  encode_float16(globalStructPtr->ahrsDataMahony.angData[2], angRangeMin, angRangeMax, buffer_16);
  memcpy(data+dataSize, buffer_16, sizeof(buffer_16));
  dataSize += sizeof(buffer_16);

  // Add newline character
    if (dataSize < BUFFER_SIZE_BYTE - 1) {
        data[dataSize] = '\n';
        dataSize += 1;
    }

  return uart_write_bytes(UART_NUM_2, data, dataSize);
}

void encode_float16(float value, float min, float max, char* buffer) {
    float resolution = (max - min) / TWO_TO_15; // 2^15
    uint16_t scaled_value = (uint16_t)round((value - min) / resolution + resolution/2);
    
    buffer[0] = (scaled_value >> 8) & 0xFF; // High byte
    buffer[1] = scaled_value & 0xFF;        // Low byte
}
