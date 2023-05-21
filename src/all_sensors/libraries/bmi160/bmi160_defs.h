#ifndef BMI160_DEFS_H
#define BMI160_DEFS_H


/* bit field offsets and lengths */
#define ACC_OFFSET_EN            (6)
#define GYR_OFFSET_EN            (7)
#define GYR_OFFSET_X_MSB_BIT     (0)
#define GYR_OFFSET_X_MSB_LEN     (2)
#define GYR_OFFSET_Y_MSB_BIT     (2)
#define GYR_OFFSET_Y_MSB_LEN     (2)
#define GYR_OFFSET_Z_MSB_BIT     (4)
#define GYR_OFFSET_Z_MSB_LEN     (2)
#define ACC_PMU_STATUS_BIT       (4)
#define ACC_PMU_STATUS_LEN       (2)
#define GYR_PMU_STATUS_BIT       (2)
#define GYR_PMU_STATUS_LEN       (2)
#define GYRO_RANGE_SEL_BIT       (0)
#define GYRO_RANGE_SEL_LEN       (3)
#define GYRO_RATE_SEL_BIT        (0)
#define GYRO_RATE_SEL_LEN        (4)
#define GYRO_DLPF_SEL_BIT        (4)
#define GYRO_DLPF_SEL_LEN        (2)
#define ACCEL_DLPF_SEL_BIT       (4)
#define ACCEL_DLPF_SEL_LEN       (3)
#define ACCEL_RANGE_SEL_BIT      (0)
#define ACCEL_RANGE_SEL_LEN      (4)
#define STATUS_FOC_RDY           (3)

/**
* Gyroscope Sensitivity Range options
* see setFullScaleGyroRange()
*/
#define GYRO_RANGE_2000      (0)    // +/- 2000 degrees/second
#define GYRO_RANGE_1000      (1)    // +/- 1000 degrees/second
#define GYRO_RANGE_500       (2)    // +/-  500 degrees/second
#define GYRO_RANGE_250       (3)    // +/-  250 degrees/second
#define GYRO_RANGE_125       (4)    // +/-  125 degrees/second

/**
* Accelerometer Sensitivity Range options
* see setFullScaleAccelRange()
*/

#define ACCEL_RANGE_2G       (0X03) // +/-  2g range
#define ACCEL_RANGE_4G       (0X05) // +/-  4g range
#define ACCEL_RANGE_8G       (0X08) // +/-  8g range
#define ACCEL_RANGE_16G      (0X0C) // +/- 16g range

#define FOC_ACC_Z_BIT        (0)
#define FOC_ACC_Z_LEN        (2)
#define FOC_ACC_Y_BIT        (2)
#define FOC_ACC_Y_LEN        (2)
#define FOC_ACC_X_BIT        (4)
#define FOC_ACC_X_LEN        (2)
#define FOC_GYR_EN           (6)

/* FIFO config options */
#define FIFO_TIME_EN_BIT     (1)
#define FIFO_MAG_EN_BIT      (5)
#define FIFO_ACC_EN_BIT      (6)
#define FIFO_GYR_EN_BIT      (7)

/* Step counter definitions */
#define STEP_MODE_NORMAL     (0)
#define STEP_MODE_SENSITIVE  (1)
#define STEP_MODE_ROBUST     (2)
#define STEP_MODE_UNKNOWN    (0)
#define STEP_BUF_MIN_BIT     (0)
#define STEP_CNT_EN_BIT      (3)
#define STEP_EN_BIT          (3)
#define STEP_BUF_MIN_LEN     (4)

/* Interrupt definitions */
#define INT1_OUTPUT_EN       (3)
#define INT1_OD              (2)
#define INT1_LVL             (1)
#define LATCH_MODE_BIT       (0)
#define LATCH_MODE_LEN       (4)

/* command definitions */
#define START_FOC        (0x03)
#define ACC_MODE_NORMAL  (0x11)
#define GYR_MODE_NORMAL  (0x15)
#define FIFO_FLUSH       (0xB0)
#define INT_RESET        (0xB1)
#define STEP_CNT_CLR     (0xB2)
#define SOFT_RESET       (0xB6)


/* register definitions */
#define CHIP_ID            (0x00)
#define PMU_STATUS         (0x03)
#define GYRO_X_L           (0x0C)
#define GYRO_X_H           (0x0D)
#define GYRO_Y_L           (0x0E)
#define GYRO_Y_H           (0x0F)
#define GYRO_Z_L           (0x10)
#define GYRO_Z_H           (0x11)
#define ACCEL_X_L          (0x12)
#define ACCEL_X_H          (0x13)
#define ACCEL_Y_L          (0x14)
#define ACCEL_Y_H          (0x15)
#define ACCEL_Z_L          (0x16)
#define ACCEL_Z_H          (0x17)
#define STATUS             (0x1B)
#define INT_STATUS_0       (0x1C)
#define INT_STATUS_1       (0x1D)
#define INT_STATUS_2       (0x1E)
#define INT_STATUS_3       (0x1F)
#define TEMP_L             (0x20)
#define TEMP_H             (0x21)
#define FIFO_LENGTH_0      (0x22)
#define FIFO_LENGTH_1      (0x23)
#define FIFO_DATA          (0x24)
#define ACCEL_CONF         (0X40)
#define ACCEL_RANGE        (0X41)
#define GYRO_CONF          (0X42)
#define GYRO_RANGE         (0X43)
#define FIFO_CONFIG_0      (0x46)
#define FIFO_CONFIG_1      (0x47)
#define INT_EN_0           (0x50)
#define INT_EN_1           (0x51)
#define INT_EN_2           (0x52)
#define INT_OUT_CTRL       (0x53)
#define INT_LATCH          (0x54)
#define INT_MAP_0          (0x55)
#define INT_MAP_1          (0x56)
#define INT_MAP_2          (0x57)
#define INT_LOWHIGH_0      (0x5A)
#define INT_LOWHIGH_1      (0x5B)
#define INT_LOWHIGH_2      (0x5C)
#define INT_LOWHIGH_3      (0x5D)
#define INT_LOWHIGH_4      (0x5E)
#define INT_MOTION_0       (0x5F)
#define INT_MOTION_1       (0x60)
#define INT_MOTION_2       (0x61)
#define INT_MOTION_3       (0x62)
#define INT_TAP_0          (0x63)
#define INT_TAP_1          (0x64)
#define FOC_CONF           (0x69)
#define OFFSET_0           (0x71)
#define OFFSET_1           (0x72)
#define OFFSET_2           (0x73)
#define OFFSET_3           (0x74)
#define OFFSET_4           (0x75)
#define OFFSET_5           (0x76)
#define OFFSET_6           (0x77)
#define STEP_CNT_L         (0x78)
#define STEP_CNT_H         (0x79)
#define STEP_CONF_0        (0x7A)
#define STEP_CONF_1        (0x7B)
#define STEP_CONF_0_NOR    (0x15)
#define STEP_CONF_0_SEN    (0x2D)
#define STEP_CONF_0_ROB    (0x1D)
#define STEP_CONF_1_NOR    (0x03)
#define STEP_CONF_1_SEN    (0x00)
#define STEP_CONF_1_ROB    (0x07)
#define CMD                (0x7E)

#endif // BMI160_DEFS_H