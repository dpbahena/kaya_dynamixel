

#ifndef BMI160_I2C_H
#define BMI160_12C_H


#include "bmi160_defs.h"
#include "i2c_driver/i2c_driver.h"
#include <string>



class Bmi160_I2C{
public:
    Bmi160_I2C(uint8_t addr);
    void getMotion_6DOF(int16_t* raw);
    
    void setGyro_offsetEnable(bool enabled);
    void autoCalibrateGyroOffset();

private:
I2C_Driver *i2c_driver;
const char * i2c_device_name;
uint8_t i2c_addr;
int bus;

/* Functions */
void init_i2c();
void init_bmi160();
void reg_Write(uint8_t reg_addr, uint8_t *data, uint16_t len);
void reg_Read(uint8_t reg_addr, uint8_t *data, uint16_t len);
uint8_t read_bits(uint8_t reg_addr, int pos, int len);
void write_bits(uint8_t reg_addr, uint8_t data, int pos, int len);

void setFullScaleGyroRange(int range);
void setFullScaleAccelRange(int range);

uint8_t getFullScaleGyroRange();
uint8_t getFullScaleAccelRange();

int get_deviceID();


int16_t getRotation_X();
int16_t getRotation_Z();
int16_t getAcceleration_Z();

uint8_t getGyro_OffsetEnabled();

int16_t get_X_gyro_Offset();
void set_X_gyro_Offset(int16_t offset);

int16_t twosComplementToDecimal(int16_t raw);
};





#endif // BMI160_12C_H