/*
  ESPNOWSender.h - Library for Description
  Created by Tommaso Bugliesi, 10/03/2024.
  Brief description: A library to activate the ESPNOW protocol for duplex communication with another ESP32 board.
*/

#ifndef ESPNOWSender_h
#define ESPNOWSender_h

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#include "Global.h"

#define GPIO_LED 33

enum msgHead {
  ESPNOW_DATA_00,
  ESPNOW_DATA_01,
  ESPNOW_DATA_02
};

class ESPNOWSender {
  private:
    // Add private variables and methods here
    uint8_t broadcastAddress[6] = {0xD8, 0xBC, 0x38, 0x78, 0x24, 0x78}; // ESP32 Flight Controller 
    
    static bool ledState;

    // Static member function to toggle the LED state
    static void toggleLEDState() {ledState = !ledState;}

  public:
    ESPNOWSender(); // Constructor
    ~ESPNOWSender(); // Destructor

    // Add public methods here
    void sendData(uint8_t header, const uint8_t *data, int dataLen);
    static void receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataLen);
    void begin();
};

#endif // ESPNOW_Sender_h
