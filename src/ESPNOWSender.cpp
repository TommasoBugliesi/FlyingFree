/*
  ESPNOWSender.cpp - Library for Description
  Created by Tommaso Bugliesi, 10/03/2024.
  Brief description: A library to activate the ESPNOW protocol for duplex communication with another ESP32 board.
*/

#include "ESPNOWSender.h"
#include "WiFi.h"


// Init static member outside the class
bool ESPNOWSender::ledState = false;

ESPNOWSender::ESPNOWSender() {
  pinMode(GPIO_LED, OUTPUT);
}

ESPNOWSender::~ESPNOWSender() {
  // Destructor implementation
}

// Add private method implementations here

// Add public method implementations here
void ESPNOWSender::sendData(uint8_t dataType, const uint8_t *data, int dataLen) {
  // Create data packet with header
  uint8_t packet[dataLen + 1];
  packet[0] = dataType; // Header indicating data type
  memcpy(packet + 1, data, dataLen); // Copy data after header
  
  // Send data
  esp_err_t result = esp_now_send(broadcastAddress, packet, dataLen + 1);
  if (result == ESP_OK) {
    Serial.println("Data sent successfully");
  } else {
    Serial.println("Error sending data");
  }
}

void ESPNOWSender::receiveCallback(const uint8_t *macAddr, const uint8_t *data, int dataSize) {
  // Extract data type from header
  uint8_t dataType = data[0]; 
  
  // Handle different types of data
  switch(dataType) {
    case ESPNOW_DATA_00:
      // Data type 1 handling logic
      Serial.println("Received data type 1:");

      break;
    case ESPNOW_DATA_01:
      // Data type 2 handling logic
      if (xSemaphoreTake(globalMutex, portMAX_DELAY) == pdTRUE) {
        memcpy(&globalStructPtr->gpioData, data + 1, sizeof(GPIOStruct));
        xSemaphoreGive(globalMutex);
      }
      // printf("Y: %d, ",globalStructPtr->gpioData.By_In);
      // printf("W: %d, ",globalStructPtr->gpioData.Bw_In);
      // printf("Z: %d, ",globalStructPtr->gpioData.Bz_In);
      // printf("X: %d, ",globalStructPtr->gpioData.Bx_In);
      // printf("B1: %d, ",globalStructPtr->gpioData.B1_In);
      // printf("B2: %d, ",globalStructPtr->gpioData.B2_In);
      // printf("T1: %d, ",globalStructPtr->gpioData.T1_In);
      // printf("T2: %d, ",globalStructPtr->gpioData.T2_In);
      // printf("T3: %d, ",globalStructPtr->gpioData.T3_In);
      // printf("T4: %d, ",globalStructPtr->gpioData.T4_In);
      // printf("P1: %d, ",globalStructPtr->gpioData.P1_In);
      // printf("P2: %d, ",globalStructPtr->gpioData.P2_In);
      // printf("J2X: %d, ",globalStructPtr->gpioData.J2x_In);
      // printf("J2Y: %d, ",globalStructPtr->gpioData.J2y_In);
      // printf("J1X: %d, ",globalStructPtr->gpioData.J1x_In);
      // printf("J1Y: %d\n",globalStructPtr->gpioData.J1y_In);
      
      break;
    default:
      // Unknown data type
      Serial.println("Received unknown data type");
  }

  toggleLEDState();
  digitalWrite(GPIO_LED, ESPNOWSender::ledState);
}

void ESPNOWSender::begin(){
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Disconnect before reconnecting 
  // WiFi.disconnect(true);

  // Print MAC address
  uint8_t macAddr[6];
  WiFi.macAddress(macAddr);
  
  Serial.print("MAC Address: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(macAddr[i], HEX);
    if (i < 5) {
      Serial.print(":");
    }
  }
  Serial.println();
  
  // Initialize ESPNOW
  if (esp_now_init() != ESP_OK) {
    Serial.printf("Error initializing ESPNOW \n");
    return;
  }
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, sizeof(broadcastAddress));
  peerInfo.channel = 0;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  while (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.printf("Failed to add peer \n");
    delay(100);
    //TODO: Add led blinking when no peer is found
  }

  // Register receive callback function
  esp_now_register_recv_cb(receiveCallback);
}
