/*
  MotorControl.h - Library for Description
  Created by TommasoBugliesi, 11/03/2024.
  Brief description: Library to control motors depending on user inputs.
  
  Reference system body frame drone 
                     ^ X
                     |
                     |
                     |
                 11° | °12
                    \|/
        <----------- o Z
        Y           / \
                 21°   °22
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
#define HALF_MAX_UINT16 2048
#define MIN_UINT16 0

enum class DSHOT_CMD : uint16_t
{
	MOTOR_STOP,						        // Currently not implemented
	BEEP1,							          // Wait at least length of beep (260ms) before next command
	BEEP2,							          // Wait at least length of beep (260ms) before next command
	BEEP3,							          // Wait at least length of beep (280ms) before next command
	BEEP4,							          // Wait at least length of beep (280ms) before next command
	BEEP5,							          // Wait at least length of beep (1020ms) before next command
	ESC_INFO,						          // Wait at least 12ms before next command
	SPIN_DIRECTION_1,				      // Need 6x, no wait required
	SPIN_DIRECTION_2,				      // Need 6x, no wait required
	MODE_3D_OFF,					        // Need 6x, no wait required
	MODE_3D_ON,						        // Need 6x, no wait required
	SETTINGS_REQUEST,				      // Currently not implemented
	SAVE_SETTINGS,					      // Need 6x, wait at least 35ms before next command
	SPIN_DIRECTION_NORMAL = 20,		// Need 6x, no wait required
	SPIN_DIRECTION_REVERSED,		  // Need 6x, no wait required
	LED0_ON,						          // No wait required
	LED1_ON,						          // No wait required
	LED2_ON,						          // No wait required
	LED3_ON,						          // No wait required
	LED0_OFF,						          // No wait required
	LED1_OFF,						          // No wait required
	LED2_OFF,						          // No wait required
	LED3_OFF,						          // No wait required
};

class MotorControl {
  GPIOStruct remoteData;

  float _inAngles[3];   
  float _inAnglesPrv[3];   

  // PID 
  bool _pidFirst = true;
  float _KpPitch = 0.0;
  float _KiPitch = 0.0;
  float _KdPitch = 0.0;
  float _KpRoll  = 0.0;
  float _KiRoll  = 0.0;
  float _KdRoll  = 0.0;

  // Roll PID controller class internal variables
  float _inRoll = 0.0;
  float _outRoll = 0.0;
  float _refRoll = 0.0;
  float _inRollPrv = 0.0;
  float _integRoll = 0.0;

  // Pitch PID controller class internal variables
  float _inPitch = 0.0;
  float _outPitch = 0.0;
  float _refPitch = 0.0;
  float _inPitchPrv = 0.0;
  float _integPitch = 0.0;

  // Motor control filter
  float _lowPasConst = 0.3;

  // Time variables
  unsigned long _currentTime; 
  unsigned long _lastTime; 
  float _elapsedTime; 

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
    uint16_t throttlePrv[4];
    float throttleNoSat[4];

    // Throttle floating numbers [0,1]
    float h = 0.0;
    float r = 0.0;
    float p = 0.0;
    float y = 0.0;

    // Flags section 
    bool B_resetMotors = 0;                 // 0 motor will not enter power up, 1 normal operation
    bool B_resetThrottle = 0;               // Always reset throttle at start up to DSHOT_THROTTLE_MIN
    bool B_lowThrottle = 0;                 // Power up only if all ESC throttle is DSHOT_THROTTLE_MIN
    uint16_t U_armCounter = 0;              // Counter for ESC power up

    // Add private variables and methods here
    void updateData();
    void motorSaturation();
    void motorFiltering();

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
    esp_err_t DShotSetReversed(bool reversed);
    // void resetRemoteData();
};

#endif // MotorControl_h
