#ifndef BMP180_H
#define BMP180_H

#include <Wire.h>
#include <Arduino.h>

#define BMP180_DEBUG 0 //!< Debug mode

#define BMP180_I2CADDR 0x77 //!< BMP180 I2C address

#define BMP180_ULTRALOWPOWER 0 //!< Ultra low power mode
#define BMP180_STANDARD 1      //!< Standard mode
#define BMP180_HIGHRES 2       //!< High-res mode
#define BMP180_ULTRAHIGHRES 3  //!< Ultra high-res mode
#define BMP180_CAL_AC1 0xAA    //!< R   Calibration data (16 bits)
#define BMP180_CAL_AC2 0xAC    //!< R   Calibration data (16 bits)
#define BMP180_CAL_AC3 0xAE    //!< R   Calibration data (16 bits)
#define BMP180_CAL_AC4 0xB0    //!< R   Calibration data (16 bits)
#define BMP180_CAL_AC5 0xB2    //!< R   Calibration data (16 bits)
#define BMP180_CAL_AC6 0xB4    //!< R   Calibration data (16 bits)
#define BMP180_CAL_B1 0xB6     //!< R   Calibration data (16 bits)
#define BMP180_CAL_B2 0xB8     //!< R   Calibration data (16 bits)
#define BMP180_CAL_MB 0xBA     //!< R   Calibration data (16 bits)
#define BMP180_CAL_MC 0xBC     //!< R   Calibration data (16 bits)
#define BMP180_CAL_MD 0xBE     //!< R   Calibration data (16 bits)

#define BMP180_CONTROL 0xF4         //!< Control register
#define BMP180_TEMPDATA 0xF6        //!< Temperature data register
#define BMP180_PRESSUREDATA 0xF6    //!< Pressure data register
#define BMP180_READTEMPCMD 0x2E     //!< Read temperature control register value
#define BMP180_READPRESSURECMD 0x34 //!< Read pressure control register value

/*!
 * @brief Main BMP180 class
 */
class BMP180 {
public:
  BMP180(void){
  };
  /*!
   * @brief Starts I2C connection
   * @param mode Mode to set, ultra high-res by default
   * @param wire The I2C interface to use, defaults to Wire
   * @return Returns true if successful
   */
  bool begin(uint8_t mode = BMP180_ULTRAHIGHRES);
  /*!
   * @brief Gets the temperature over I2C from the BMP180
   * @return Returns the temperature
   */
  float readTemperature(void);
  /*!
   * @brief Gets the pressure over I2C from the BMP180
   * @return Returns the pressure
   */
  int32_t readPressure(void);
  /*!
   * @brief Calculates the pressure at sea level
   * @param altitude_meters Current altitude (in meters)
   * @return Returns the calculated pressure at sea level
   */
  int32_t readSealevelPressure(float altitude_meters = 0);
  /*!
   * @brief Reads the altitude
   * @param sealevelPressure Pressure at sea level, measured in pascals
   * @return Returns the altitude
   */
  float readAltitude(float sealevelPressure = 101325); // std atmosphere
  /*!
   * @brief Reads the raw temperature
   * @return Returns the raw temperature
   */
  uint16_t readRawTemperature(void);
  /*!
   * @brief Reads the raw pressure
   * @return Returns the raw pressure
   */
  uint32_t readRawPressure(void);

private:
  int32_t computeB5(int32_t UT);
  uint8_t read8(uint8_t registerAddress);
  uint16_t read16(uint8_t registerAddress);
  void write8(uint8_t registerAddress, uint8_t data);
  
  uint8_t oversampling;

  int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
  uint16_t ac4, ac5, ac6;
};

#endif //  BMP180_H
