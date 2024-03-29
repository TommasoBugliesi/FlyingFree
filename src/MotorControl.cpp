/*
  MotorControl.cpp - Library for Description
  Created by TommasoBugliesi, 11/03/2024.
  Brief description: Library to control motors depending on user inputs.

  DSHOT protocol based on Gabriel Zerbib DSHOT library:
  MIT License

  Copyright (c) 2023 Gabriel Zerbib (Moddingear)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.


  MIT License

  Copyright (c) 2021 Jakub Karbowski

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "MotorControl.h"

#define MAP_UINT16(input, minOut, maxOut) ((input) / (MAX_UINT16-MIN_UINT16) * ((maxOut) - (minOut)) + (minOut))

MotorControl::MotorControl() {
  // DShot constructor
  _rmtChannel[0] = RMT_CHANNEL_MAX;
  _rmtChannel[1] = RMT_CHANNEL_MAX;
  _rmtChannel[2] = RMT_CHANNEL_MAX;
  _rmtChannel[3] = RMT_CHANNEL_MAX;
	// initialize cmd buffer
	DShotSetData(0);

	// DShot packet delay + RMT end marker
	_dshotCmd[16].duration0 = dt_pause;
	_dshotCmd[16].level0 = 0;
	_dshotCmd[16].duration1 = 0;
	_dshotCmd[16].level1 = 0;

  for (int i = 0; i < 4; ++i) {
      throttle[i] = DSHOT_THROTTLE_MIN;
  }
}

MotorControl::~MotorControl() {
  // Destructor implementation
}

// Add private method implementations here
void MotorControl::updateData(){
  if (xSemaphoreTake(globalMutex, portMAX_DELAY) == pdTRUE) {
    memcpy(&remoteData, &(globalStructPtr->gpioData), sizeof(remoteData));
    xSemaphoreGive(globalMutex);
  }
}

void MotorControl::motorControl(){
  // Get data from remote controller and save it to local structure
  updateData();

  // Temporary throttle testing
  float tmp = MAP_UINT16((float)remoteData.P1_In, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
  
  // Startup logic for motors
  if (remoteData.T2_In | (!remoteData.T2_In & !remoteData.T1_In)){

    // Reset flags and throttle value
    B_resetMotors = 0;
    B_lowThrottle = 0;

    // If throttle value is not 0 reset throttle
    if (!B_resetThrottle){
      for (int i = 0; i < 4; ++i) {
        throttle[i] = DSHOT_THROTTLE_MIN;
      }
      DShotSendThrottle(throttle);
      B_resetThrottle = 1;
    }
  }

  if (!B_resetMotors & remoteData.T1_In){

    // Reinit motors with reset sequence and send 0 throttle, next step send throttle
    DShotReset();

    // Send motor direction
    DShotSetReversed(false);
    B_resetMotors = 1;

    // Send DSHOT_THROTTLE_MIN throttle
    for (int i = 0; i < 4; ++i) {
      throttle[i] = DSHOT_THROTTLE_MIN;
    }
    while (U_armCounter < 100){
      printf("Power up ESC step %d of 101\n", U_armCounter);
      // Send 0 throttle at startup with 1kHz freqeuncy then rest
      if (U_armCounter<50){
        DShotSendThrottle(throttle);
      }
      U_armCounter ++;
    }

    // Reset arm counter and change flag for the next step
    U_armCounter = 0;
    B_resetThrottle = 1;
  }
  else if (B_resetMotors & remoteData.T1_In & !B_lowThrottle){

    // Put throttle value into throttle
    for (size_t i = 0; i < 4; i++) {
      throttle[i] = (uint16_t)floor(tmp);
    }

    // TODO: avoid entering normal operation with throttle not zero
    if ((throttle[0] + throttle[1] + throttle[2] + throttle[3])/4 == DSHOT_THROTTLE_MIN){
      B_lowThrottle = 1;
      delay(500);
    }
    else {
      Serial.print("Remove throttle\n");
      delay(500);
    }
  }
  else if (B_resetMotors & remoteData.T1_In & B_lowThrottle){

    // If motors resetted normal operation for motors
    for (size_t i = 0; i < 4; i++) {
      throttle[i] = (uint16_t)floor(tmp);
    }
    DShotSendThrottle(throttle);

    // Throttle is no more zero, reset 0 value when T1!=1
    B_resetThrottle = 0;
  }
}

void MotorControl::begin(){
  gpio_num_t pwmOut[4] = {GPIO_NUM_26,GPIO_NUM_27,GPIO_NUM_25,GPIO_NUM_4};
  rmt_channel_t rmtChannels[4] = {(rmt_channel_t)1,(rmt_channel_t)2,(rmt_channel_t)3,(rmt_channel_t)4};
  
  DShotInit(pwmOut,rmtChannels); 
  DShotReset();
}

esp_err_t MotorControl::DShotInit(gpio_num_t *gpio, rmt_channel_t *rmtChannel, unsigned long frequency, uint8_t rmtdivider){
  // Calculate bit timing for RMT peripheral
	double core_ticks_per_bit = APB_CLK_FREQ/frequency;   //This line calculates the number of clock ticks per bit for the DSHOT protocol.
  dt_tpb = core_ticks_per_bit/rmtdivider;               // Ticks per bit
	dt_t0h = core_ticks_per_bit/rmtdivider/3;             // Ticks per bit 0 High 
	dt_t1h = core_ticks_per_bit/rmtdivider*2/3;           // Ticks per bit 1 High
	dt_t0l = dt_tpb-dt_t0h;                               // Ticks per bit 0 Low
	dt_t1l = dt_tpb-dt_t1h;                               // Ticks per bit 1 Low 
	dt_pause = dt_tpb*200;
	divider = rmtdivider;

  // Define rmt_config_t input for rmt channel
  rmt_config_t config[4];
  esp_err_t out[4];

  // Assign rmtChannel to internal object variable
  for (int h = 0; h<4; h++){
    _rmtChannel[h] = rmtChannel[h];
    config[h].channel = rmtChannel[h];                       // rmt channel to be linked to gpio
    config[h].rmt_mode = RMT_MODE_TX;                        // rmt as output
    config[h].gpio_num = gpio[h];                            // GPIO number 
    config[h].mem_block_num = 1;                             // Link memory block to channel 
    config[h].clk_div = rmtdivider;                          // Source clock is 80MHz, divided by that number, gives 0.0875us/tick
    config[h].tx_config.loop_en = false;                     // Transmit data only once
    config[h].tx_config.carrier_en = false;                  // No carrier wave
    config[h].tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;    
    config[h].tx_config.idle_output_en = true;
    config[h].flags = RMT_CHANNEL_FLAGS_INVERT_SIG;
     
    DSHOT_ERROR_CHECK(rmt_config(&config[h]));               // Check for data validity
    out[h] = rmt_driver_install(rmtChannel[h], 0, 0);
  }

  return *out;
}

esp_err_t MotorControl::DShotDeinit(){
  esp_err_t out[4];
  for (int h = 0; h<4; h++){
    out[h] = rmt_driver_uninstall(_rmtChannel[h]);
    _rmtChannel[h] = RMT_CHANNEL_MAX;
  }
  return *out;
}

esp_err_t MotorControl::DShotReset(){
  uint16_t data[4] = {0,0,0,0};
  // Set 50 emtpy data to reset DShot device 
  for (int i = 0; i < 50; i++)
	{
		DShotWriteData(data, true);
	}
	return ESP_OK;
}

void MotorControl::DShotSetData(uint16_t data){
	for (int i = 0; i < 16; i++, data <<= 1)
	{
		if (data & 0x8000) // Set for each data the command information: from left to right bit shifting
		{
      // set zero
			_dshotCmd[i].duration0 = dt_t0h;
			_dshotCmd[i].level0 = 1;
			_dshotCmd[i].duration1 = dt_t0l;
			_dshotCmd[i].level1 = 0;

		}
		else
		{
      // set one
			_dshotCmd[i].duration0 = dt_t1h;
			_dshotCmd[i].level0 = 1;
			_dshotCmd[i].duration1 = dt_t1l;
			_dshotCmd[i].level1 = 0;
		}
	}
}

esp_err_t MotorControl::DShotWriteData(uint16_t *data, bool wait){
  esp_err_t out[4];
  for (int h = 0; h<4; h++){
    DSHOT_ERROR_CHECK(rmt_wait_tx_done(_rmtChannel[h], 0));

    // Assemble data in RMT format
    DShotSetData(data[h]);

    // Send command _dshotCmd to channel with commadn size and waiting option
    out[h] = rmt_write_items(_rmtChannel[h], _dshotCmd, RMT_CMD_SIZE, wait);
  }
  return *out;
}

uint8_t MotorControl::DShotChecksum(uint16_t data){
	uint16_t csum = 0;
	for (int i = 0; i < 3; i++){
		csum ^= data;
		data >>= 4;
	}
	return csum & 0xf;
}

esp_err_t MotorControl::DShotWritePacket(dshot_packet_t *packet, bool wait){
	uint16_t data[4];
  for (int h = 0; h<4; h++){
    data[h] = packet[h].payload;
    data[h] <<= 1;
    data[h] |= packet[h].telemetry;
    data[h] = (data[h] << 4) | DShotChecksum(data[h]);
  }
  // Serial.println(data[0], BIN); //DEBUG for rmt library
	return DShotWriteData(data, wait);
}

esp_err_t MotorControl::DShotRepeatPacket(dshot_packet_t *packet, int n){
	for (int i = 0; i < n; i++)	{
		DSHOT_ERROR_CHECK(DShotWritePacket(packet, true));
		portYIELD();
	}
	return ESP_OK;
}

esp_err_t MotorControl::DShotSendThrottle(uint16_t *throttle)
{
  dshot_packet_t packet[4];
  for (int h = 0; h<4; h++){
    if (throttle[h] > DSHOT_THROTTLE_MAX)
    {
      throttle[h] = DSHOT_THROTTLE_MAX;
    }
    else if (throttle[h] < DSHOT_THROTTLE_MIN)
    {
      throttle[h] = DSHOT_THROTTLE_MIN;
    }
    packet[h] = {throttle[h], 0};
  }  
	return DShotWritePacket(packet, false);
}

esp_err_t MotorControl::DShotSetReversed(bool reversed)
{ 
  dshot_packet_t packet[4];
  for (int h = 0; h<4; h++){
    packet[h].payload = reversed ? (uint16_t)DSHOT_CMD::SPIN_DIRECTION_REVERSED : (uint16_t)DSHOT_CMD::SPIN_DIRECTION_NORMAL;
    packet[h].telemetry = 1;
  }
	DSHOT_ERROR_CHECK(DShotRepeatPacket(packet,10));
	return ESP_OK;
}


// void MotorControl::resetRemoteData(){
//   memcpy(&remoteData, &(globalStructPtr->gpioData), sizeof(remoteData));
// }