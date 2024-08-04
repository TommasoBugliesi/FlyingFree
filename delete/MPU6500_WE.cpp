/********************************************************************
* 
* This is a library for the 6-axis gyroscope and accelerometer MPU6500.
*
* You'll find an example which should enable you to use the library.
*
* You are free to use it, change it or build on it. In case you like
* it, it would be cool if you give it a star.
*
* If you find bugs, please inform me!
*
* Written by Wolfgang (Wolle) Ewald
*
* For further information visit my blog:
*
* https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1  (German)
* https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1  (English)
*
*********************************************************************/

#include "MPU6500_WE.h"

/* Registers MPU6500 */
uint8_t constexpr MPU6500_WE::REGISTER_SELF_TEST_X_GYRO     ;
uint8_t constexpr MPU6500_WE::REGISTER_SELF_TEST_Y_GYRO     ;
uint8_t constexpr MPU6500_WE::REGISTER_SELF_TEST_Z_GYRO     ;
uint8_t constexpr MPU6500_WE::REGISTER_SELF_TEST_X_ACCEL    ;
uint8_t constexpr MPU6500_WE::REGISTER_SELF_TEST_Y_ACCEL    ;
uint8_t constexpr MPU6500_WE::REGISTER_SELF_TEST_Z_ACCEL    ;
uint8_t constexpr MPU6500_WE::REGISTER_XG_OFFSET_H          ;
uint8_t constexpr MPU6500_WE::REGISTER_XG_OFFSET_L          ;
uint8_t constexpr MPU6500_WE::REGISTER_YG_OFFSET_H          ;
uint8_t constexpr MPU6500_WE::REGISTER_YG_OFFSET_L          ;
uint8_t constexpr MPU6500_WE::REGISTER_ZG_OFFSET_H          ;
uint8_t constexpr MPU6500_WE::REGISTER_ZG_OFFSET_L          ;
uint8_t constexpr MPU6500_WE::REGISTER_SMPLRT_DIV           ;
uint8_t constexpr MPU6500_WE::REGISTER_CONFIG               ;
uint8_t constexpr MPU6500_WE::REGISTER_GYRO_CONFIG          ;
uint8_t constexpr MPU6500_WE::REGISTER_ACCEL_CONFIG         ;
uint8_t constexpr MPU6500_WE::REGISTER_ACCEL_CONFIG_2       ;
uint8_t constexpr MPU6500_WE::REGISTER_LP_ACCEL_ODR         ;
uint8_t constexpr MPU6500_WE::REGISTER_WOM_THR              ;
uint8_t constexpr MPU6500_WE::REGISTER_FIFO_EN              ;
uint8_t constexpr MPU6500_WE::REGISTER_I2C_MST_CTRL         ;
uint8_t constexpr MPU6500_WE::REGISTER_I2C_SLV0_ADDR        ;
uint8_t constexpr MPU6500_WE::REGISTER_I2C_SLV0_REG         ;
uint8_t constexpr MPU6500_WE::REGISTER_I2C_SLV0_CTRL        ;
uint8_t constexpr MPU6500_WE::REGISTER_I2C_MST_STATUS       ;
uint8_t constexpr MPU6500_WE::REGISTER_INT_PIN_CFG          ;
uint8_t constexpr MPU6500_WE::REGISTER_INT_ENABLE           ;
uint8_t constexpr MPU6500_WE::REGISTER_INT_STATUS           ;
uint8_t constexpr MPU6500_WE::REGISTER_ACCEL_OUT            ;
uint8_t constexpr MPU6500_WE::REGISTER_TEMP_OUT             ;
uint8_t constexpr MPU6500_WE::REGISTER_GYRO_OUT             ;
uint8_t constexpr MPU6500_WE::REGISTER_EXT_SLV_SENS_DATA_00 ;
uint8_t constexpr MPU6500_WE::REGISTER_I2C_SLV0_DO          ;
uint8_t constexpr MPU6500_WE::REGISTER_I2C_MST_DELAY_CTRL   ;
uint8_t constexpr MPU6500_WE::REGISTER_SIGNAL_PATH_RESET    ;
uint8_t constexpr MPU6500_WE::REGISTER_MOT_DET_CTRL         ;
uint8_t constexpr MPU6500_WE::REGISTER_USER_CTRL            ;
uint8_t constexpr MPU6500_WE::REGISTER_PWR_MGMT_1           ;
uint8_t constexpr MPU6500_WE::REGISTER_PWR_MGMT_2           ;
uint8_t constexpr MPU6500_WE::REGISTER_FIFO_COUNT           ;
uint8_t constexpr MPU6500_WE::REGISTER_FIFO_R_W             ;
uint8_t constexpr MPU6500_WE::REGISTER_WHO_AM_I             ;
uint8_t constexpr MPU6500_WE::REGISTER_XA_OFFSET_H          ;
uint8_t constexpr MPU6500_WE::REGISTER_XA_OFFSET_L          ;
uint8_t constexpr MPU6500_WE::REGISTER_YA_OFFSET_H          ;
uint8_t constexpr MPU6500_WE::REGISTER_YA_OFFSET_L          ;
uint8_t constexpr MPU6500_WE::REGISTER_ZA_OFFSET_H          ;
uint8_t constexpr MPU6500_WE::REGISTER_ZA_OFFSET_L          ;

/* Register Values */
uint8_t constexpr MPU6500_WE::REGISTER_VALUE_RESET          ;
uint8_t constexpr MPU6500_WE::REGISTER_VALUE_BYPASS_EN      ;
uint8_t constexpr MPU6500_WE::REGISTER_VALUE_I2C_MST_EN     ;
uint8_t constexpr MPU6500_WE::REGISTER_VALUE_CLK_SEL_PLL    ;

/* Others */
float constexpr MPU6500_WE::ROOM_TEMPERATURE_OFFSET         ;
float constexpr MPU6500_WE::TEMPERATURE_SENSITIVITY         ;
float constexpr MPU6500_WE::WHO_AM_I_CODE                   ;

/************  Constructors ************/

MPU6500_WE::MPU6500_WE(uint8_t addr)
    : MPU6500_WE(&Wire, addr)
{
    // intentionally empty
}

MPU6500_WE::MPU6500_WE(TwoWire *w, uint8_t addr)
    : _wire(w)
    , i2cAddress(addr)
{
    // intentionally empty
}

MPU6500_WE::MPU6500_WE(SPIClass *s, int cs, bool spi)
    : _spi(s)
    , csPin(cs)
    , useSPI(spi)
{
    // intentionally empty
}


/************ Basic Settings ************/

bool MPU6500_WE::init(uint8_t const expectedValue, int SPI_mode){
    if(useSPI){
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);
        if (SPI_mode==0){
            _spi->begin();
            mySPISettings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
        }
        else if  (SPI_mode==1){          
            _spi->begin(14,12,13,15); //(SCK, MISO, MOSI, CS)
            mySPISettings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
        }
        else if  (SPI_mode==2){
            mySPISettings = SPISettings(8000000, MSBFIRST, SPI_MODE2);
        }
        else {
            mySPISettings = SPISettings(8000000, MSBFIRST, SPI_MODE3);
        }
                      
    }   
    reset_MPU9250();
    delay(10);
    writeMPU9250Register(REGISTER_INT_PIN_CFG, REGISTER_VALUE_BYPASS_EN);  // Bypass Enable
    delay(10);
    if(whoAmI() != expectedValue){
        Serial.print(whoAmI());
        Serial.print("\n");
        return false;
    }
    return true;
}


// bool MPU6500_WE::init(){
//     return init(WHO_AM_I_CODE);
// }

uint8_t MPU6500_WE::whoAmI(){
    return readMPU9250Register8(REGISTER_WHO_AM_I);
}

void MPU6500_WE::reset_MPU9250(){
    writeMPU9250Register(REGISTER_PWR_MGMT_1, REGISTER_VALUE_RESET);
    delay(10);  // wait for registers to reset
}

void MPU6500_WE::enableI2CMaster(){
    uint8_t regVal = readMPU9250Register8(REGISTER_USER_CTRL);
    regVal |= REGISTER_VALUE_I2C_MST_EN;
    writeMPU9250Register(REGISTER_USER_CTRL, regVal); //enable I2C master
    writeMPU9250Register(REGISTER_I2C_MST_CTRL, 0x00); // set I2C clock to 400 kHz
    delay(10);
}

void MPU6500_WE::writeMPU9250Register(uint8_t reg, uint8_t val){
    if(!useSPI){
        _wire->beginTransmission(i2cAddress);
        _wire->write(reg);
        _wire->write(val);
        _wire->endTransmission();
    }
    else{
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg); 
        _spi->transfer(val);
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
    }
}

uint8_t MPU6500_WE::readMPU9250Register8(uint8_t reg){
    uint8_t regValue = 0;
    
    if(!useSPI){
        _wire->beginTransmission(i2cAddress);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(i2cAddress,(uint8_t)1);
        if(_wire->available()){
            regValue = _wire->read();
        }
    }
    else{
        reg |= 0x80;
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg); 
        regValue = _spi->transfer(0x00);
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
    }
    return regValue;
}

int16_t MPU6500_WE::readMPU9250Register16(uint8_t reg){
    uint8_t MSByte = 0, LSByte = 0;
    int16_t regValue = 0;
    
    if(!useSPI){
        _wire->beginTransmission(i2cAddress);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(i2cAddress,(uint8_t)2);
        if(_wire->available()){
            MSByte = _wire->read();
            LSByte = _wire->read();
        }
    }
    else{
        reg |= 0x80;
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg); 
        MSByte = _spi->transfer(0x00);
        LSByte = _spi->transfer(0x00);
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
    }
    regValue = (MSByte<<8) + LSByte;
    return regValue;
}

void MPU6500_WE::readMPU9250Register3x16(uint8_t reg, uint8_t *buf){
    if(!useSPI){
        _wire->beginTransmission(i2cAddress);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(i2cAddress,(uint8_t)6);
        if(_wire->available()){
            for(int i=0; i<6; i++){
                buf[i] = _wire->read();
            }
        }
    }
    else{
        reg |= 0x80;
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg);
        for(int i=0; i<6; i++){
                buf[i] = _spi->transfer(0x00);
        }
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
    }
}

/************ end ************/
