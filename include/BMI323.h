/*
  BMI323.h - Library for Description
  Created by Tommaso, 25/02/2024.
  Brief description: -
*/

#ifndef BMI323_h
#define BMI323_h

#include <Arduino.h>
#include <SPI.h>

#define BMI323_REG_CHIP_ID 0x00  ///< Register that contains the part ID

class BMI323 {
  private:
    SPIClass *spi; // Reference to SPI object
    const SPISettings& spiSettings = SPISettings(1000*1000, MSBFIRST, SPI_MODE0);
    const int BMI323_CS_PIN = 15;

    // Add private methods here

  public:
    BMI323(SPIClass* _spi); // Constructor
    ~BMI323(); // Destructor

    // Add public methods here
    
    /**
     * @brief Reads the WHO_AM_I register of the BMI323.
     * 
     * This method initiates communication with the BMI323 sensor,
     * reads the value of the WHO_AM_I register (0x0F), and deselects
     * the device afterwards.
     * 
     * @param spiSettings The SPI settings to use for the transaction.
     * @return The value read from the WHO_AM_I register.
     */
    void read_who_am_i();
}; 

#endif // BMI323_h
