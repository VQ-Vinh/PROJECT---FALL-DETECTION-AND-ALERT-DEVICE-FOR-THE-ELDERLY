#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"
#include "esp_err.h"

#define MPU6050_ADDR             0x68 // AD0 -> GND

#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43

esp_err_t mpu6050_init(i2c_port_t i2c_num);
esp_err_t mpu6050_read_raw_data(i2c_port_t i2c_num, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

void mpu6050_calibrate_get_bias(i2c_port_t i2c_num, float *acc_bias, float *gyro_bias);

#endif