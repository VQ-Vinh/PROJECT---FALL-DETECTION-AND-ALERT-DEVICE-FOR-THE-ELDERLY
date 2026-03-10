#include "mpu6050.h"
#include <stdio.h>

// Định nghĩa bổ sung các thanh ghi cấu hình
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C

// Các hệ số tỷ lệ tương ứng với cấu hình trong hàm init
#define ACC_SCALE 4096.0f   // Cho dải +- 8g
#define GYRO_SCALE 131.0f   // Cho dải +- 250 deg/s
#define GRAVITY 9.81f

esp_err_t mpu6050_init(i2c_port_t i2c_num) {
    esp_err_t ret;

    // 1. Wake up: Ghi 0x00 vào PWR_MGMT_1 (0x6B)
    uint8_t pwr_data[] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    ret = i2c_master_write_to_device(i2c_num, MPU6050_ADDR, pwr_data, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    // 2. Cấu hình Accel +-8g: Ghi 0x10 vào ACCEL_CONFIG (0x1C)
    uint8_t accel_cfg[] = {MPU6050_REG_ACCEL_CONFIG, 0x10};
    ret = i2c_master_write_to_device(i2c_num, MPU6050_ADDR, accel_cfg, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    // 3. Cấu hình Gyro +-250 deg/s: Ghi 0x00 vào GYRO_CONFIG (0x1B)
    uint8_t gyro_cfg[] = {MPU6050_REG_GYRO_CONFIG, 0x00};
    ret = i2c_master_write_to_device(i2c_num, MPU6050_ADDR, gyro_cfg, 2, 1000 / portTICK_PERIOD_MS);

    return ret;
}

esp_err_t mpu6050_read_raw_data(i2c_port_t i2c_num, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z, 
                               int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z) {
    uint8_t data[14];
    uint8_t reg_addr = MPU6050_REG_ACCEL_XOUT_H;

    // Write address then read 14 bytes (6 Accel, 2 Temp, 6 Gyro)
    esp_err_t ret = i2c_master_write_read_device(i2c_num, MPU6050_ADDR, &reg_addr, 1, data, 14, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    *accel_x = (int16_t)((data[0] << 8) | data[1]);
    *accel_y = (int16_t)((data[2] << 8) | data[3]);
    *accel_z = (int16_t)((data[4] << 8) | data[5]);
    // data[6], data[7] là nhiệt độ (bỏ qua)
    *gyro_x  = (int16_t)((data[8] << 8) | data[9]);
    *gyro_y  = (int16_t)((data[10] << 8) | data[11]);
    *gyro_z  = (int16_t)((data[12] << 8) | data[13]);

    return ESP_OK;
}



void mpu6050_calibrate_get_bias(i2c_port_t i2c_num, float *acc_bias, float *gyro_bias) {
    int16_t rx, ry, rz, rgx, rgy, rgz;
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    const int samples = 1000;

    printf("Calibrating... Keep the sensor horizontal and still.\n");

    for (int i = 0; i < samples; i++) {
        if (mpu6050_read_raw_data(i2c_num, &rx, &ry, &rz, &rgx, &rgy, &rgz) == ESP_OK) {
            ax_sum += rx;
            ay_sum += ry;
            az_sum += rz;
            gx_sum += rgx;
            gy_sum += rgy;
            gz_sum += rgz;
        }
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }

    // Chuyển đổi trung bình Raw sang đơn vị vật lý m/s^2 và deg/s
    acc_bias[0] = ((float)ax_sum / samples) / ACC_SCALE * GRAVITY;
    acc_bias[1] = ((float)ay_sum / samples) / ACC_SCALE * GRAVITY;
    // Trục Z chịu tác động của trọng trường khi nằm ngang (1g)
    acc_bias[2] = (((float)az_sum / samples) / ACC_SCALE * GRAVITY) - GRAVITY;

    gyro_bias[0] = ((float)gx_sum / samples) / GYRO_SCALE;
    gyro_bias[1] = ((float)gy_sum / samples) / GYRO_SCALE;
    gyro_bias[2] = ((float)gz_sum / samples) / GYRO_SCALE;

    printf("Calibration Done!\n");
    printf("Accel Bias (m/s^2): X:%.3f, Y:%.3f, Z:%.3f\n", acc_bias[0], acc_bias[1], acc_bias[2]);
    printf("Gyro Bias (deg/s):  X:%.3f, Y:%.3f, Z:%.3f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}