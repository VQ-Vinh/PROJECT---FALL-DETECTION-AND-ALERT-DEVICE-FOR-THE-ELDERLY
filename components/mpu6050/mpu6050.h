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
void mpu6050_convert_accel(int16_t raw_x, int16_t raw_y, int16_t raw_z, float *accel_x, float *accel_y, float *accel_z);
void mpu6050_convert_gyro(int16_t raw_x, int16_t raw_y, int16_t raw_z, float *gyro_x, float *gyro_y, float *gyro_z);
/**
 * @brief  Perform a simple bias calibration by averaging a number of
 *         stationary samples.
 *
 * This routine computes accelerometer and gyroscope biases, subtracts
 * gravity from the Z‑axis, and stores the results in internal static
 * variables used by convert_*() functions.  The computed values are also
 * copied to the provided output arrays if they are non‑NULL.
 *
 * @param i2c_num       I2C port.
 * @param accel_bias    Pointer to 3‑element array to receive accel biases
 *                      (m/s^2).  May be NULL.
 * @param gyro_bias     Pointer to 3‑element array to receive gyro biases
 *                      (deg/s).  May be NULL.
 */
void mpu6050_calibrate(i2c_port_t i2c_num, float *accel_bias, float *gyro_bias);
/**
 * @brief Calculate total acceleration magnitude (vector sum)
 *
 * Computes: A = sqrt(ax² + ay² + az²)
 *
 * @param accel_x Accelerometer X (m/s^2)
 * @param accel_y Accelerometer Y (m/s^2)
 * @param accel_z Accelerometer Z (m/s^2)
 * @param accel_g Pointer to store result in g (optional, can be NULL)
 * @return Total acceleration magnitude (m/s^2)
 */
float mpu6050_get_total_accel(float accel_x, float accel_y, float accel_z, float *accel_g);
float mpu6050_get_total_gyro(float gyro_x, float gyro_y, float gyro_z);

#endif