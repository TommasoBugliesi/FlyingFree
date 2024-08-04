/*
  BMI323.h - Library for Description
  Created by Tommaso, 25/02/2024.
  Brief description: -
*/

#ifndef BMI323_h
#define BMI323_h

#include <Arduino.h>
#include <SPI.h>

#define BMI323_REG_CHIP_ID 0x00             ///< Register that contains the chip ID
#define BMI323_REG_ERR_REG 0x01             ///< Register address for error register
#define BMI323_REG_STATUS 0x02              ///< Register address for status
#define BMI323_REG_ACC_DATA_X 0x03          ///< Register address for accelerometer data X-axis
#define BMI323_REG_ACC_DATA_Y 0x04          ///< Register address for accelerometer data Y-axis
#define BMI323_REG_ACC_DATA_Z 0x05          ///< Register address for accelerometer data Z-axis
#define BMI323_REG_GYR_DATA_X 0x06          ///< Register address for gyroscope data X-axis
#define BMI323_REG_GYR_DATA_Y 0x07          ///< Register address for gyroscope data Y-axis
#define BMI323_REG_GYR_DATA_Z 0x08          ///< Register address for gyroscope data Z-axis
#define BMI323_REG_TEMP_DATA 0x09           ///< Register address for temperature data
#define BMI323_REG_SENSOR_TIME_0 0x0A       ///< Register address for sensor time, LSB
#define BMI323_REG_SENSOR_TIME_1 0x0B       ///< Register address for sensor time, MSB
#define BMI323_REG_SAT_FLAGS 0x0C           ///< Register address for saturation flags
#define BMI323_REG_INT_STATUS_INT1 0x0D     ///< Register address for interrupt status 1
#define BMI323_REG_INT_STATUS_INT2 0x0E     ///< Register address for interrupt status 2
#define BMI323_REG_INT_STATUS_IBI 0x0F      ///< Register address for interrupt status IBI
#define BMI323_REG_FEATURE_IO0 0x10         ///< Register address for feature IO0
#define BMI323_REG_FEATURE_IO1 0x11         ///< Register address for feature IO1
#define BMI323_REG_FEATURE_IO2 0x12         ///< Register address for feature IO2
#define BMI323_REG_FEATURE_IO3 0x13         ///< Register address for feature IO3
#define BMI323_REG_FEATURE_IO_STATUS 0x14   ///< Register address for feature IO status
#define BMI323_REG_FIFO_FILL_LEVEL 0x15     ///< Register address for FIFO fill level
#define BMI323_REG_FIFO_DATA 0x16           ///< Register address for FIFO data
#define BMI323_REG_ACC_CONF 0x20            ///< Register address for accelerometer configuration
#define BMI323_REG_GYR_CONF 0x21            ///< Register address for gyroscope configuration
#define BMI323_REG_ALT_ACC_CONF 0x28        ///< Register address for alternate accelerometer configuration
#define BMI323_REG_ALT_GYR_CONF 0x29        ///< Register address for alternate gyroscope configuration
#define BMI323_REG_ALT_CONF 0x2A            ///< Register address for alternate configuration
#define BMI323_REG_ALT_STATUS 0x2B          ///< Register address for alternate status
#define BMI323_REG_FIFO_WATERMARK 0x35      ///< Register address for FIFO watermark
#define BMI323_REG_FIFO_CONF 0x36           ///< Register address for FIFO configuration
#define BMI323_REG_FIFO_CTRL 0x37           ///< Register address for FIFO control
#define BMI323_REG_IO_INT_CTRL 0x38         ///< Register address for IO interrupt control
#define BMI323_REG_INT_CONF 0x39            ///< Register address for interrupt configuration
#define BMI323_REG_INT_MAP1 0x3A            ///< Register address for interrupt map 1
#define BMI323_REG_INT_MAP2 0x3B            ///< Register address for interrupt map 2
#define BMI323_REG_FEATURE_CTRL 0x40        ///< Register address for feature control
#define BMI323_REG_FEATURE_DATA_ADDR 0x41   ///< Register address for feature data address
#define BMI323_REG_FEATURE_DATA_TX 0x42     ///< Register address for feature data transmission
#define BMI323_REG_FEATURE_DATA_STATUS 0x43 ///< Register address for feature data status
#define BMI323_REG_FEATURE_ENGINE_STATUS 0x45 ///< Register address for feature engine status
#define BMI323_REG_FEATURE_EVENT_EXT 0x47   ///< Register address for feature external event
#define BMI323_REG_IO_PDN_CTRL 0x4F         ///< Register address for IO power down control
#define BMI323_REG_IO_SPI_IF 0x50           ///< Register address for IO SPI interface
#define BMI323_REG_IO_PAD_STRENGTH 0x51     ///< Register address for IO pad strength
#define BMI323_REG_IO_I2C_IF 0x52           ///< Register address for IO I2C interface
#define BMI323_REG_IO_ODR_DEVIATION 0x53    ///< Register address for IO ODR deviation
#define BMI323_REG_ACC_DP_OFF_X 0x60        ///< Register address for accelerometer digital offset X-axis
#define BMI323_REG_ACC_DP_DGAIN_X 0x61      ///< Register address for accelerometer digital gain X-axis
#define BMI323_REG_ACC_DP_OFF_Y 0x62        ///< Register address for accelerometer digital offset Y-axis
#define BMI323_REG_ACC_DP_DGAIN_Y 0x63      ///< Register address for accelerometer digital gain Y-axis
#define BMI323_REG_ACC_DP_OFF_Z 0x64        ///< Register address for accelerometer digital offset Z-axis
#define BMI323_REG_ACC_DP_DGAIN_Z 0x65      ///< Register address for accelerometer digital gain Z-axis
#define BMI323_REG_GYR_DP_OFF_X 0x66        ///< Register address for gyroscope digital offset X-axis
#define BMI323_REG_GYR_DP_DGAIN_X 0x67      ///< Register address for gyroscope digital gain X-axis
#define BMI323_REG_GYR_DP_OFF_Y 0x68        ///< Register address for gyroscope digital offset Y-axis
#define BMI323_REG_GYR_DP_DGAIN_Y 0x69      ///< Register address for gyroscope digital gain Y-axis
#define BMI323_REG_GYR_DP_OFF_Z 0x6A        ///< Register address for gyroscope digital offset Z-axis
#define BMI323_REG_GYR_DP_DGAIN_Z 0x6B      ///< Register address for gyroscope digital gain Z-axis
#define BMI323_REG_I3C_TC_SYNC_TPH 0x70     ///< Register address for I3C timing control synchronous, part A
#define BMI323_REG_I3C_TC_SYNC_TU 0x71      ///< Register address for I3C timing control synchronous, part B
#define BMI323_REG_I3C_TC_SYNC_ODR 0x72     ///< Register address for I3C timing control synchronous, part C
#define BMI323_REG_CMD 0x7E                 ///< Register address for command
#define BMI323_REG_CFG_RES 0x7F             ///< Register address for configuration and reset

typedef enum {
    BMI323_CMD_TRIGGER_SELF_TEST = 0x100, ///< Trigger the self-test of the device
    BMI323_CMD_TRIGGER_SELF_CALIBRATION = 0x101, ///< Trigger the self-calibration of the gyroscope
    BMI323_CMD_ABORT_SELF_CALIBRATION = 0x200, ///< Abort a running self-calibration of the gyroscope
    BMI323_CMD_UPDATE_I3C_CONFIG = 0x201, ///< Update the configuration of the I3C timing control synchronous feature
    BMI323_CMD_UPDATE_AXIS_MAPPING = 0x300, ///< Axis mapping gets updated
    BMI323_CMD_SOFT_RESET = 0xDEAF ///< Triggers a soft reset
} bmi323_commands_t;

typedef enum {
    BMI323_ODR_0_78125HZ = 0b0001, ///< Output Data Rate (ODR) = 0.78125Hz
    BMI323_ODR_1_5625HZ = 0b0010,  ///< ODR = 1.5625Hz
    BMI323_ODR_3_125HZ = 0b0011,   ///< ODR = 3.125Hz
    BMI323_ODR_6_25HZ = 0b0100,    ///< ODR = 6.25Hz
    BMI323_ODR_12_5HZ = 0b0101,    ///< ODR = 12.5Hz
    BMI323_ODR_25HZ = 0b0110,      ///< ODR = 25Hz
    BMI323_ODR_50HZ = 0b0111,      ///< ODR = 50Hz
    BMI323_ODR_100HZ = 0b1000,     ///< ODR = 100Hz
    BMI323_ODR_200HZ = 0b1001,     ///< ODR = 200Hz
    BMI323_ODR_400HZ = 0b1010,     ///< ODR = 400Hz
    BMI323_ODR_800HZ = 0b1011,     ///< ODR = 800Hz
    BMI323_ODR_1_6KHZ = 0b1100,    ///< ODR = 1.6kHz
    BMI323_ODR_3_2KHZ = 0b1101,    ///< ODR = 3.2kHz
    BMI323_ODR_6_4KHZ = 0b1110     ///< ODR = 6.4kHz
} bmi323_odr_t;

typedef enum {
    BMI323_ACC_RANGE_2G = 0b000,  ///< Accelerometer range +/-2g, 16.38 LSB/mg
    BMI323_ACC_RANGE_4G = 0b001,  ///< +/-4g, 8.19 LSB/mg
    BMI323_ACC_RANGE_8G = 0b010,  ///< +/-8g, 4.10 LSB/mg
    BMI323_ACC_RANGE_16G = 0b011 ///< +/-16g, 2.05 LSB/mg
} bmi323_acc_range_t;

typedef enum {
    BMI323_ACC_BANDWIDTH_ODR_2 = 0b0, ///< Bandwidth = ODR/2
    BMI323_ACC_BANDWIDTH_ODR_4 = 0b1  ///< Bandwidth = ODR/4
} bmi323_acc_bandwidth_t;

typedef enum {
    BMI323_ACC_AVERAGING_NONE = 0b000, ///< No averaging; pass sample without filtering
    BMI323_ACC_AVERAGING_2 = 0b001,    ///< Averaging of 2 samples
    BMI323_ACC_AVERAGING_4 = 0b010,    ///< Averaging of 4 samples
    BMI323_ACC_AVERAGING_8 = 0b011,    ///< Averaging of 8 samples
    BMI323_ACC_AVERAGING_16 = 0b100,   ///< Averaging of 16 samples
    BMI323_ACC_AVERAGING_32 = 0b101,   ///< Averaging of 32 samples
    BMI323_ACC_AVERAGING_64 = 0b110    ///< Averaging of 64 samples
} bmi323_acc_averaging_t;

typedef enum {
    BMI323_ACC_OPERATION_DISABLED = 0b000,        ///< Disables the accelerometer
    BMI323_ACC_OPERATION_DUTY_CYCLING = 0b011,    ///< Enables the accelerometer with sensing operated in duty-cycling
    BMI323_ACC_OPERATION_CONTINUOUS_REDUCED_CURRENT = 0b100, ///< Enables the accelerometer in a continuous operation mode with reduced current
    BMI323_ACC_OPERATION_HIGH_PERFORMANCE = 0b111  ///< Enables the accelerometer in high performance mode
} bmi323_acc_operation_t;

typedef enum {
    BMI323_GYRO_ODR_0_78125HZ = 0b0001, ///< ODR = 0.78125Hz
    BMI323_GYRO_ODR_1_5625HZ = 0b0010,  ///< ODR = 1.5625Hz
    BMI323_GYRO_ODR_3_125HZ = 0b0011,   ///< ODR = 3.125Hz
    BMI323_GYRO_ODR_6_25HZ = 0b0100,    ///< ODR = 6.25Hz
    BMI323_GYRO_ODR_12_5HZ = 0b0101,    ///< ODR = 12.5Hz
    BMI323_GYRO_ODR_25HZ = 0b0110,      ///< ODR = 25Hz
    BMI323_GYRO_ODR_50HZ = 0b0111,      ///< ODR = 50Hz
    BMI323_GYRO_ODR_100HZ = 0b1000,     ///< ODR = 100Hz
    BMI323_GYRO_ODR_200HZ = 0b1001,     ///< ODR = 200Hz
    BMI323_GYRO_ODR_400HZ = 0b1010,     ///< ODR = 400Hz
    BMI323_GYRO_ODR_800HZ = 0b1011,     ///< ODR = 800Hz
    BMI323_GYRO_ODR_1_6KHZ = 0b1100,    ///< ODR = 1.6kHz
    BMI323_GYRO_ODR_3_2KHZ = 0b1101,    ///< ODR = 3.2kHz
    BMI323_GYRO_ODR_6_4KHZ = 0b1110     ///< ODR = 6.4kHz
} bmi323_gyro_odr_t;

typedef enum {
    BMI323_GYRO_RANGE_125DPS = 0b000,    ///< Gyroscope range +/-125 degrees per second
    BMI323_GYRO_RANGE_250DPS = 0b001,    ///< +/-250 dps
    BMI323_GYRO_RANGE_500DPS = 0b010,    ///< +/-500 dps
    BMI323_GYRO_RANGE_1000DPS = 0b011,   ///< +/-1000 dps
    BMI323_GYRO_RANGE_2000DPS = 0b100    ///< +/-2000 dps
} bmi323_gyro_range_t;

typedef enum {
    BMI323_GYRO_BANDWIDTH_ODR_2 = 0b0, ///< Bandwidth = ODR/2
    BMI323_GYRO_BANDWIDTH_ODR_4 = 0b1  ///< Bandwidth = ODR/4
} bmi323_gyro_bandwidth_t;

typedef enum {
    BMI323_GYRO_AVERAGING_NONE = 0b000, ///< No averaging; pass sample without averaging
    BMI323_GYRO_AVERAGING_2 = 0b001,    ///< Averaging of 2 samples
    BMI323_GYRO_AVERAGING_4 = 0b010,    ///< Averaging of 4 samples
    BMI323_GYRO_AVERAGING_8 = 0b011,    ///< Averaging of 8 samples
    BMI323_GYRO_AVERAGING_16 = 0b100,   ///< Averaging of 16 samples
    BMI323_GYRO_AVERAGING_32 = 0b101,   ///< Averaging of 32 samples
    BMI323_GYRO_AVERAGING_64 = 0b110    ///< Averaging of 64 samples
} bmi323_gyro_averaging_t;

typedef enum {
    BMI323_GYRO_OPERATION_DISABLED = 0b000,        ///< Disables the gyroscope
    BMI323_GYRO_OPERATION_DRIVE_ENABLED = 0b001,   ///< Disables the gyroscope but keep the gyroscope drive enabled
    BMI323_GYRO_OPERATION_DUTY_CYCLING = 0b011,    ///< Enables the gyroscope with sensing operated in duty-cycling
    BMI323_GYRO_OPERATION_CONTINUOUS_REDUCED_CURRENT = 0b100, ///< Enables the gyroscope in a continuous operation mode with reduced current
    BMI323_GYRO_OPERATION_HIGH_PERFORMANCE = 0b111  ///< Enables the gyroscope in high performance mode
} bmi323_gyro_operation_t;

class BMI323 {
  private:
    SPIClass *spi; // Reference to SPI object
    const int BMI323_CS_PIN = 15;
    const int BMI323_SPI_SPEED   = 1000*1000;
    uint16_t register_value        = 0x00;
    uint16_t status_register[1]    = {0x00};
    uint8_t buffer_MSB;
    uint8_t buffer_LSB;
    int16_t data_raw[6];
    float acc_gain = 0;
    float acc_offset[3] = {0,0,0};
    float gyro_gain = 0;
    float gyro_offset[3] = {0,0,0};
    float data[6];

    // Add private methods here
    /**
     * @brief Read registers. 
     * 
     * @param reg: Register to be read.
     * @param buffer: pointer to array.
     * @param length: size of array, default value is 1.
     */
    void readRegisters(uint8_t reg, uint16_t *buffer, size_t length=1);

    /**
     * @brief Read registers. 
     * 
     * @param reg: Register to be read.
     * @param buffer: pointer to array.
     * @param length: size of array, default value is 1.
     */
    void readRegisters(uint8_t reg, int16_t *buffer, size_t length=1);

    /**
     * @brief Write register.
     * 
     * @param reg: Register to be written.
     * @param reg_value: Register value.
     */
    void writeRegister(uint8_t reg, uint16_t reg_value);

    /**
     * @brief Reset hardware.
     */
    void reset();

    /**
     * @brief Write custom settings to the BMI323_REG_ACC_CONF register of BMI323 magnetometer.
     */
    void writeAccConfig();

    /**
     * @brief Read accelerometer config to set internal class gain.
     */
    void readAccConfig();

    /**
     * @brief Write custom settings to the BMI323_REG_GYR_CONF register of BMI323 magnetometer.
     */
    void writeGyrConfig();

    /**
     * @brief Read gyro config to set internal class gain.
     */
    void readGyrConfig();

    /**
     * @brief Read status register for data stream.
     */
    void readStatusReg();

    /**
     * @brief Read error register for data stream.
     */
    void readErrorReg();

    /**
     * @brief Read status register for data stream.
     */
    void readDataRaw();


  public:
    BMI323(SPIClass* _spi); // Constructor
    ~BMI323(); // Destructor

    // Add public methods here
    /**
     * @brief Begin with custom setup .
     */
    void begin();
    
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
    void readWhoAmI();

    /**
     * @brief Calibration of accelerometer with custom function.
     * 
     * This method read the register of the BMI323.
     */
    void calibrateAcc();

    /**
     * @brief Calibration of gyroscope with custom function.
     * 
     * This method read the register of the BMI323.
     */
    void calibrateGyr();
    
    /**
     * @brief Read BMI323 data as int16_t.
     * 
     * This method read the register of the BMI323.
     */
    void readData();

}; 

#endif // BMI323_h
