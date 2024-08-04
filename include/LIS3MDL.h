/*
  LIS3MDL.h - Library for Description
  Created by Tommaso, 25/02/2024.
  Brief description: -
*/

#ifndef LIS3MDL_h
#define LIS3MDL_h

#include <Arduino.h>
#include <SPI.h>

#define LIS3MDL_REG_WHO_AM_I 0x0F  ///< Register that contains the part ID
#define LIS3MDL_REG_CTRL_REG1 0x20 ///< Register address for control 1
#define LIS3MDL_REG_CTRL_REG2 0x21 ///< Register address for control 2
#define LIS3MDL_REG_CTRL_REG3 0x22 ///< Register address for control 3
#define LIS3MDL_REG_CTRL_REG4 0x23 ///< Register address for control 3
#define LIS3MDL_REG_STATUS 0x27    ///< Register address for status
#define LIS3MDL_REG_OUT_X_L 0x28   ///< Register address for X axis lower byte
#define LIS3MDL_REG_INT_CFG 0x30   ///< Interrupt configuration register
#define LIS3MDL_REG_INT_THS_L 0x32 ///< Low byte of the irq threshold

/** Reboot */
typedef enum {
  LIS3MDL_REBOOT_DISABLE = 0b00,  ///< Reboot true
  LIS3MDL_REBOOT_ENABLE = 0b01,   ///< Reboot false
} lis3mdl_reboot_t;

/** Soft Reset */
typedef enum {
  LIS3MDL_SOFT_RESET_DISABLE = 0b00,  ///< Soft Reset true
  LIS3MDL_SOFT_RESET_ENABLE = 0b01,   ///< Soft Reset false
} lis3mdl_reset_t;

/** The magnetometer ranges */
typedef enum {
  LIS3MDL_RANGE_4_GAUSS = 0b00,  ///< +/- 4g (default value)
  LIS3MDL_RANGE_8_GAUSS = 0b01,  ///< +/- 8g
  LIS3MDL_RANGE_12_GAUSS = 0b10, ///< +/- 12g
  LIS3MDL_RANGE_16_GAUSS = 0b11, ///< +/- 16g
} lis3mdl_range_t;

/** The magnetometer data rate, includes FAST_ODR bit */
typedef enum {
  LIS3MDL_DATARATE_0_625_HZ = 0b0000, ///<  0.625 Hz
  LIS3MDL_DATARATE_1_25_HZ = 0b0010,  ///<  1.25 Hz
  LIS3MDL_DATARATE_2_5_HZ = 0b0100,   ///<  2.5 Hz
  LIS3MDL_DATARATE_5_HZ = 0b0110,     ///<  5 Hz
  LIS3MDL_DATARATE_10_HZ = 0b1000,    ///<  10 Hz
  LIS3MDL_DATARATE_20_HZ = 0b1010,    ///<  20 Hz
  LIS3MDL_DATARATE_40_HZ = 0b1100,    ///<  40 Hz
  LIS3MDL_DATARATE_80_HZ = 0b1110,    ///<  80 Hz
  LIS3MDL_DATARATE_155_HZ = 0b0001,   ///<  155 Hz (FAST_ODR + UHP)
  LIS3MDL_DATARATE_300_HZ = 0b0011,   ///<  300 Hz (FAST_ODR + HP)
  LIS3MDL_DATARATE_560_HZ = 0b0101,   ///<  560 Hz (FAST_ODR + MP)
  LIS3MDL_DATARATE_1000_HZ = 0b0111,  ///<  1000 Hz (FAST_ODR + LP)
} lis3mdl_dataRate_t;

/** The magnetometer performance mode */
typedef enum {
  LIS3MDL_LOWPOWERMODE = 0b00,  ///< Low power mode
  LIS3MDL_MEDIUMMODE = 0b01,    ///< Medium performance mode
  LIS3MDL_HIGHMODE = 0b10,      ///< High performance mode
  LIS3MDL_ULTRAHIGHMODE = 0b11, ///< Ultra-high performance mode
} lis3mdl_performancemode_t;

/** The magnetometer operation mode */
typedef enum {
  LIS3MDL_CONTINUOUSMODE = 0b00, ///< Continuous conversion
  LIS3MDL_SINGLEMODE = 0b01,     ///< Single-shot conversion
  LIS3MDL_POWERDOWNMODE = 0b11,  ///< Powered-down mode
} lis3mdl_operationmode_t;

/** Temperature enable */
typedef enum {
  LIS3MDL_TEMPERATURE_DISABLE = 0b00, ///< Temperature disable
  LIS3MDL_TEMPERATURE_ENABLE = 0b01,  ///< Temperature enable
} lis3mdl_temperature_t;

/** Fast Output Data Rate */
typedef enum {
  LIS3MDL_FAST_ODR_DISABLE = 0b00, ///< Fast ODR disable
  LIS3MDL_FAST_ODR_ENABLE = 0b01,  ///< Fast ODR enable
} lis3mdl_odr_t;

/** Self test */
typedef enum {
  LIS3MDL_SELFTEST_DISABLE = 0b00, ///< Fast ODR disable
  LIS3MDL_SELFTEST_ENABLE = 0b01,  ///< Fast ODR enable
} lis3mdl_selftest_t;

class LIS3MDL {
  private:
    SPIClass *spi; // Reference to SPI object
    const int LIS3MDL_CS_PIN      = 5;
    const int LIS3MDL_SPI_SPEED   = 1000*1000;
    uint8_t register_value        = 0x00;
    uint8_t status_register[1]    = {0x00};
    uint8_t data_raw[6];
    int16_t data[3];

    // Add private methods here
    /**
     * @brief Read registers. 
     * 
     * @param reg: Register to be read.
     * @param buffer: pointer to array.
     * @param length: size of array, default value is 1.
     */
    void readRegisters(uint8_t reg, uint8_t *buffer, size_t length=1);

    /**
     * @brief Write register.
     * 
     * @param reg: Register to be written.
     * @param reg_value: Register value.
     */
    void writeRegister(uint8_t reg, uint8_t reg_value);

    /**
     * @brief Reset hardware.
     */
    void reset();

    /**
     * @brief Write custom settings to the CTRL_REG1 register of LIS3MDL magnetometer.
     */
    void writeCtrlReg1();

    /**
     * @brief Write custom settings to the CTRL_REG2 register of LIS3MDL magnetometer.
     */
    void writeCtrlReg2();

    /**
     * @brief Write custom settings to the CTRL_REG3 register of LIS3MDL magnetometer.
     */
    void writeCtrlReg3();

    /**
     * @brief Write custom settings to the CTRL_REG4 register of LIS3MDL magnetometer.
     */
    void writeCtrlReg4();

    /**
     * @brief Read status register for data stream.
     */
    void readStatusReg();

    /**
     * @brief Read status register for data stream.
     */
    void readDataRaw();

  public:
    LIS3MDL(SPIClass* _spi); // Constructor
    ~LIS3MDL(); // Destructor

    // Add public methods here

    /**
     * @brief Begin with custom setup .
     */
    void begin();
    
    /**
     * @brief Reads the WHO_AM_I register of the LIS3MDL.
     * 
     * This method initiates communication with the LIS3MDL sensor,
     * reads the value of the WHO_AM_I register (0x0F), and deselects
     * the device afterwards.
     */
    void readWhoAmI();

    /**
     * @brief Read LIS3MDL data as int16_t.
     * 
     * This method read the register of the LIS3MDL and apply a bitshifting
     * to represent the data as combination of MSB and LSB.
     */
    void readData();
}; 

#endif // LIS3MDL_h
