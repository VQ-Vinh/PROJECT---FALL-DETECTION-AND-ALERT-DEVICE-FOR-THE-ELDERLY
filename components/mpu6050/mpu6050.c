#include "mpu6050.h"
#include <stdio.h>

// Địa chỉ thanh ghi cấu hình cho Accel và Gyro
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C

// Hệ số tỷ lệ để chuyển đổi raw data sang đơn vị vật lý (g và deg/s)
#define ACC_SCALE   4096.0f     // Cho dải +- 8g
#define GYRO_SCALE  16.4f      // Cho dải +- 2000 deg/s
#define GRAVITY     9.81f

// Giá trị độ lệch ban đầu được đặt thành 0 và sẽ được cập nhật trong quá trình hiệu chuẩn (mpu6050_calibrate). 
// Các hàm convert_*() sẽ sử dụng các giá trị này để điều chỉnh kết quả đầu ra.
static float accel_bias[3] = {0.0f, 0.0f, 0.0f}; // (m/s^2)
static float gyro_bias[3]  = {0.0f, 0.0f, 0.0f}; // (deg/s)

esp_err_t mpu6050_init(i2c_port_t i2c_num) {
    esp_err_t ret;

    // 1. Wake up: Ghi 0x00 vào thanh ghi PWR_MGMT_1 (0x6B)
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

    // 14 bytes: 6 cho accel (X, Y, Z), 2 cho nhiệt độ (bỏ qua), 6 cho gyro (X, Y, Z)
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

void mpu6050_convert_accel(int16_t raw_x, int16_t raw_y, int16_t raw_z, float *accel_x, float *accel_y, float *accel_z) {
    *accel_x = (raw_x / ACC_SCALE) * GRAVITY - accel_bias[0];
    *accel_y = (raw_y / ACC_SCALE) * GRAVITY - accel_bias[1];
    *accel_z = (raw_z / ACC_SCALE) * GRAVITY - accel_bias[2];
}

void mpu6050_convert_gyro(int16_t raw_x, int16_t raw_y, int16_t raw_z, float *gyro_x, float *gyro_y, float *gyro_z) {
    *gyro_x = (raw_x / GYRO_SCALE) - gyro_bias[0];
    *gyro_y = (raw_y / GYRO_SCALE) - gyro_bias[1];
    *gyro_z = (raw_z / GYRO_SCALE) - gyro_bias[2];
}


void mpu6050_calibrate(i2c_port_t i2c_num, float *accel_bias_out, float *gyro_bias_out) {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    float accel_x_sum = 0.0f, accel_y_sum = 0.0f, accel_z_sum = 0.0f;
    float gyro_x_sum = 0.0f, gyro_y_sum = 0.0f, gyro_z_sum = 0.0f;
    const int samples = 100;

    for (int i = 0; i < samples; i++) {
        mpu6050_read_raw_data(i2c_num, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);

        // Chuyển đổi raw data sang đơn vị vật lý 
        accel_x_g = (accel_x / ACC_SCALE) * GRAVITY;
        accel_y_g = (accel_y / ACC_SCALE) * GRAVITY;
        accel_z_g = (accel_z / ACC_SCALE) * GRAVITY;

        gyro_x_dps = (gyro_x / GYRO_SCALE);
        gyro_y_dps = (gyro_y / GYRO_SCALE);
        gyro_z_dps = (gyro_z / GYRO_SCALE);

        accel_x_sum += accel_x_g;
        accel_y_sum += accel_y_g;
        accel_z_sum += accel_z_g;
        gyro_x_sum += gyro_x_dps;
        gyro_y_sum += gyro_y_dps;
        gyro_z_sum += gyro_z_dps;

        vTaskDelay(5 / portTICK_PERIOD_MS); // Delay giữa các lần đọc
    }

    // Tính giá trị trung bình và trừ đi trọng lực từ trục Z
    float a_bias[3];
    float g_bias[3];

    a_bias[0] = accel_x_sum / samples;
    a_bias[1] = accel_y_sum / samples;
    a_bias[2] = accel_z_sum / samples - GRAVITY;

    g_bias[0] = gyro_x_sum / samples;
    g_bias[1] = gyro_y_sum / samples;
    g_bias[2] = gyro_z_sum / samples;

    // Cập nhật giá trị bias vào output nếu con trỏ không NULL
    if (accel_bias_out) {
        accel_bias_out[0] = a_bias[0];
        accel_bias_out[1] = a_bias[1];
        accel_bias_out[2] = a_bias[2];
    }
    if (gyro_bias_out) {
        gyro_bias_out[0] = g_bias[0];
        gyro_bias_out[1] = g_bias[1];
        gyro_bias_out[2] = g_bias[2];
    }

    // Lưu trữ giá trị bias vào biến toàn cục để sử dụng trong các hàm convert_*()
    accel_bias[0] = a_bias[0];
    accel_bias[1] = a_bias[1];
    accel_bias[2] = a_bias[2];

    gyro_bias[0] = g_bias[0];
    gyro_bias[1] = g_bias[1];
    gyro_bias[2] = g_bias[2];
}
