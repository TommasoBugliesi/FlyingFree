/*
  MotorControl.h - Library for Description
  Created by TommasoBugliesi, 11/03/2024.
  Brief description: Library to control motors depending on user inputs.
*/

#ifndef MotorControl_h
#define MotorControl_h

#include <Arduino.h>
#include "freertos/task.h"
#include "driver/rmt.h"
#include "Global.h"

#define DSHOT_ERROR_CHECK(x) ({       \
	esp_err_t __ret = x;                \
	if (__ret != ESP_OK)                \
		return __ret;                     \
	__ret;                              \
})

#define RMT_CMD_SIZE (sizeof(_dshotCmd) / sizeof(_dshotCmd[0])) // Prevent variable definition rmt_item32_t _dshotCmd[17]; 
#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047
#define MAX_UINT16 4095
#define MIN_UINT16 0

class MotorControl {
  GPIOStruct remoteData;

  public:
    MotorControl(); // Constructor
    ~MotorControl(); // Destructor

    void begin();
    void motorControl();
    // Add public methods here

  private:
  	struct dshot_packet_t{
      uint16_t payload;
      bool telemetry;
    };
    rmt_item32_t _dshotCmd[17];
	  rmt_channel_t _rmtChannel[4];
    uint16_t dt_t0h, dt_t0l;               //  Ticks duration to stay low and high for a 0
    uint16_t dt_t1h, dt_t1l;               //  Ticks duration to stay low and high for a 1
    uint16_t dt_tpb;                       //  Total duration of a bit (ticks per bit)
    uint16_t dt_pause;
    uint8_t divider;
    uint16_t throttle[4];

    // Add private variables and methods here
    void updateData();

    /*DShot functions and protocol definition*/
    esp_err_t DShotInit(gpio_num_t *gpio, rmt_channel_t *rmtChannel, unsigned long frequency = 600000UL, uint8_t rmtdivider = 3);
    esp_err_t DShotReset();
    esp_err_t DShotSendThrottle(uint16_t *throttle);
    esp_err_t DShotDeinit();
    void DShotSetData(uint16_t data);
    esp_err_t DShotWriteData(uint16_t *data, bool wait);
    uint8_t DShotChecksum(uint16_t data);
    esp_err_t DShotWritePacket(dshot_packet_t *packet, bool wait);
    esp_err_t DShotRepeatPacket(dshot_packet_t *packet, int n);
    // void resetRemoteData();
};

#endif // MotorControl_h
