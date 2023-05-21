#include "bmi160-i2c.h"
#include <assert.h>
#include <thread>
#include <cmath>


#define BMI160_SIGN_EXTEND(val, from) \
    (((val) & (1 << ((from) - 1))) ? (val | (((1 << (1 + (sizeof(val) << 3) - (from))) - 1) << (from))) : val)

   

Bmi160_I2C::Bmi160_I2C(uint8_t addr){
    
    /**
     * Power on and prepare for general usage.
     * This will activate the device and take it out of sleep mode (which must be done
     * after start-up). This function also sets both the accelerometer and the gyroscope
     * to default range settings, namely +/- 2g and +/- 250 degrees/sec.
     * */    
    
    i2c_addr = addr;
    init_i2c();
    init_bmi160();
    printf("ID: %d\n", get_deviceID());
    setGyro_offsetEnable(true);
    printf("gyro enable? : %d\n", getGyro_OffsetEnabled());
    int16_t offsetValue = get_X_gyro_Offset();
    printf("Get gyro X offset value: %d\n", offsetValue);
    // set_X_gyro_Offset(offsetValue);
    // int16_t offsetValueNew = get_X_gyro_Offset();
    // printf("Get gyro X offset NEW value: %d\n", offsetValueNew);

   

}

/**
 * Open the I2C device name
*/
void Bmi160_I2C::init_i2c(){

    i2c_device_name = "/dev/i2c-1";  
    i2c_driver = new I2C_Driver(i2c_device_name);
    bool openSuccess = i2c_driver->open_i2c_device();
    if(!openSuccess){
    printf("FAILED to open I2C driver");
    assert(openSuccess == true);
  }
}


void Bmi160_I2C::init_bmi160(){

  /* Issue a soft-reset to bring the device into a clean state */
  uint8_t data = SOFT_RESET;
  reg_Write(CMD, &data, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  /* Issue a dummy-read to force the device into I2C comms mode */
  data = 0;
  reg_Read(0x7f, &data, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  /* Power the accelerometer */
  data =  ACC_MODE_NORMAL;
  reg_Write(CMD, &data, 1);
  
  
  while(1 != read_bits(PMU_STATUS, ACC_PMU_STATUS_BIT, ACC_PMU_STATUS_LEN)){
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  /* Power the gyroscope */
  data = GYR_MODE_NORMAL;
  reg_Write(CMD, &data, 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  
  /* Wait power-up to complete */
  while(1 != read_bits(PMU_STATUS, GYR_PMU_STATUS_BIT, GYR_PMU_STATUS_LEN)){
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  setFullScaleGyroRange(GYRO_RANGE_250);
  setFullScaleAccelRange(ACCEL_RANGE_2G);

  /* Only PIN1 interrupts currently supported - map all interrupts to PIN1 */
  data = 0xff;
  reg_Write(INT_MAP_0,&data, 1);
  data = 0xF0;
  reg_Write(INT_MAP_1, &data, 1);
  data = 0x00;
  reg_Write(INT_MAP_2, &data, 1);
  
}

void Bmi160_I2C::setFullScaleGyroRange(int range){
  write_bits(GYRO_RANGE, range, GYRO_RANGE_SEL_BIT, GYRO_RANGE_SEL_LEN);

}

uint8_t Bmi160_I2C::getFullScaleGyroRange(){
  return read_bits(GYRO_RANGE, GYRO_RANGE_SEL_BIT, GYRO_RANGE_SEL_LEN);
}

/**
 * Set full scale gyroscope range.
 * @param range    New full-scale gyroscope range value
 * 
*/
void Bmi160_I2C::setFullScaleAccelRange(int range){
  write_bits(ACCEL_RANGE, range, ACCEL_RANGE_SEL_BIT, ACCEL_RANGE_SEL_LEN);
}

uint8_t Bmi160_I2C::getFullScaleAccelRange(){
  return read_bits(ACCEL_RANGE, ACCEL_RANGE_SEL_BIT, ACCEL_RANGE_SEL_LEN);
}

void Bmi160_I2C::write_bits(uint8_t reg_addr, uint8_t data, int pos, int len){
  uint8_t read_data = -1;
  reg_Read(reg_addr, &read_data, 1);
  int mask = ((1 << len) - 1) << pos;
  data <<= pos;  // shift data into correct position
  data &= mask; // zero all non-important bits in data
  read_data &= ~(mask);
  read_data |= data;

  reg_Write(reg_addr, &read_data, 1);
  
}


void Bmi160_I2C::reg_Read(uint8_t reg_addr, uint8_t *data, uint16_t len){
  
  bool success = i2c_driver->write_data_then_read_data(i2c_addr, len, &reg_addr, len, data);
  if (!success){
    printf("Failed to read data\n");
    assert(success == true);
  }
}

uint8_t Bmi160_I2C::read_bits(uint8_t reg_addr, int pos, int len){
  uint8_t data=-1;
  reg_Read(reg_addr, &data, 1);

  //printf("data: %d\n", data);
  int mask = (1 << len) - 1;
  data >>= pos;
  data &= mask;
  return data;
}

void Bmi160_I2C::reg_Write(uint8_t reg_addr, uint8_t *data, uint16_t len){
  uint8_t *buffer = new uint8_t[len+1];
  buffer[0]= reg_addr;
  for(int i = 0; i < len; i++){
    buffer[i+1] = data[i];
    //std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  bool success = i2c_driver->write_data(i2c_addr, len + 1 , buffer);
  
  if (!success){
    printf("Failed to write data\n");
    assert(success == true);
  }
  delete[] buffer;

}



/**
 * Get Device ID.
 * @return ID (should be 0xD1 or 209)
*/
int Bmi160_I2C::get_deviceID(){
  uint8_t data = -1;
  reg_Read(CHIP_ID, &data, 1);
  return data;
}

/**
 * Get X-axis gyroscope reading
 * @return X-axis rotation measurement in 16-bits 2's complement format
*/
int16_t Bmi160_I2C::getRotation_X(){
  uint8_t data[]={0,0};
  int16_t raw;
  reg_Read(GYRO_X_L, data, 2);
  return raw = (data[1]<< 8) | data[0];  
}
/**
 * Get Z-axis gyroscope reading
 * @return Z-axis rotation measurement in 16-bits 2's complement format
*/
int16_t Bmi160_I2C::getRotation_Z(){
  uint8_t data[]={0,0};
  int16_t raw;
  reg_Read(GYRO_Z_L, data, 2);
  return raw = (data[1]<< 8) | data[0];
}

int16_t Bmi160_I2C::getAcceleration_Z(){
  uint8_t data[]={0,0};
  int16_t raw;
  reg_Read(ACCEL_Z_L, data, 2);
  return raw = (data[1]<< 8) | data[0];
}


/**
 * Get raw 6-axis motion sensor readings (accel/gyro).
  # Retrieves all currently available motion sensor values.
  # @return gx 16-bit signed integer container for gyroscope X-axis value
  # @return gy 16-bit signed integer container for gyroscope Y-axis value
  # @return gz 16-bit signed integer container for gyroscope Z-axis value
  # @return ax 16-bit signed integer container for accelerometer X-axis value
  # @return ay 16-bit signed integer container for accelerometer Y-axis value
  # @return az 16-bit signed integer container for accelerometer Z-axis value
  # @see getAcceleration()
  # @see getRotation()
  # @see registers.GYRO_X_L
*/

void Bmi160_I2C::getMotion_6DOF(int16_t * raw){
  uint8_t data[12];
  reg_Read(GYRO_X_L, data, 12);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  for(int i = 0; i < 6; i++){
    // raw[i] = twosComplementToDecimal((data[i*2+1]<< 8) | data[i * 2]);
    raw[i] = (data[i*2+1]<< 8) | data[i * 2];
    //printf("%d\n", raw[i]);
    
  }
  //printf("_____________________________________\n");
}


/**
 *  It is a function that takes a 16-bit integer (raw acceleration data in two's complement format) 
 *  as input and returns its decimal equivalent. It checks the sign bit (the 16th bit from the right). 
 *  If the sign bit is 1, it means the number is negative and it negates all the bits and 
 *  adds 1, finally returning the negative of the result. If the sign bit is 0, it returns the number as it is.
 *  @param int16_t raw
 *  @return a 16-bit positive or negative decimal number
*/
int16_t Bmi160_I2C::twosComplementToDecimal(int16_t raw) {
    // If the number is negative (based on the 16th bit)
    if(raw & 0x8000) {
        return -((~raw + 1) & 0xFFFF);  // Negate all bits and add 1, then return negative
    } else {
        return raw; // If not negative, return as is
    }
}

/** Get gyroscope offset compensation enabled value.
 * @see getXGyroOffset()
 * @see BMI160_RA_OFFSET_6
 */
uint8_t Bmi160_I2C::getGyro_OffsetEnabled(){
  return 0 != read_bits(OFFSET_6, GYR_OFFSET_EN, 1);
}


/** Set gyroscope offset compensation enabled value.
 * @see getXGyroOffset()
 * @see BMI160_RA_OFFSET_6
 */
void Bmi160_I2C::setGyro_offsetEnable(bool enabled){
   
  write_bits(OFFSET_6, enabled ? 0x1 : 0x0, GYR_OFFSET_EN, 1 );
}




/** Execute internal calibration to generate Gyro offset values.
 * This populates the Gyro offset compensation values for all 3 axes.
 * These can be retrieved using the get[X/Y/Z]GyroOffset() methods.
 * Note that this procedure may take up to 250ms to complete.
 *
 * IMPORTANT: The user MUST ensure that NO rotation of the BMI160 device
 * occurs while this auto-calibration process is active.
 *
 * To enable offset compensation, @see setGyroOffsetEnabled()
 * @see setGyroOffsetEnabled()
 * @see getXGyroOffset()
 * @see getYGyroOffset()
 * @see getZGyroOffset()
 * @see BMI160_RA_FOC_CONF
 * @see BMI160_RA_CMD
 */
void Bmi160_I2C::autoCalibrateGyroOffset(){
  uint8_t foc_conf = (1 << FOC_GYR_EN);
  reg_Write(FOC_CONF, &foc_conf, 1);
  uint8_t data = START_FOC;
  reg_Write(CMD, &data, 1);
  while(!read_bits(STATUS, STATUS_FOC_RDY, 1)){
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  } 
}

/** Get offset compensation value for gyroscope X-axis data.
 * The value is represented as an 10-bit two-complement number in
 * units of 0.061 degrees/s per LSB (sign-extended for int16_t type).
 * @see BMI160_RA_OFFSET_3
 * @see BMI160_RA_OFFSET_6
 */
int16_t Bmi160_I2C::get_X_gyro_Offset(){
  uint8_t data[2];
  int16_t offset;
  reg_Read(OFFSET_3, data, 2);
  offset = (data[1]<< 8) | data[0];
  offset |= (read_bits(OFFSET_6, GYR_OFFSET_X_MSB_BIT, GYR_OFFSET_X_MSB_LEN)) << 8;
  return BMI160_SIGN_EXTEND(offset, 10);

}

void Bmi160_I2C::set_X_gyro_Offset(int16_t offset){
  uint8_t data[2];
  /* Separate 16-bit offset into 2 bytes */
  // 1. Shift right by 8 bits and mask with 0xFF to get the low byte
  data[1]= (offset >> 8) & 0xff;
  // 2. Mask with 0xff to get to high byte
  data[0]= offset & 0xff;
  reg_Write(OFFSET_3, data, 2);
  write_bits(OFFSET_6, offset >> 8, GYR_OFFSET_X_MSB_BIT, GYR_OFFSET_X_MSB_LEN);
  getRotation_X();  // read and discard the next data value


}


