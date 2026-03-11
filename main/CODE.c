#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"

#define I2C_MASTER_SCL_IO       22          
#define I2C_MASTER_SDA_IO       21          
#define I2C_MASTER_NUM          I2C_NUM_0   
#define I2C_MASTER_FREQ_HZ      400000     
#define I2C_MASTER_FLAG_DEFAULT 0

void app_main(void)
{
    esp_err_t ret;
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
    float acc_bias[3] = {0};
    float gyro_bias[3] = {0};

    // Config I2C master
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to configure I2C: %s", esp_err_to_name(ret));
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, I2C_MASTER_FLAG_DEFAULT);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return;
    }

    // Khởi tạo MPU6050
    ret = mpu6050_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Failed to initialize MPU6050: %s", esp_err_to_name(ret));
        return;
    }

    // Hiệu chỉnh MPU6050 để tính bias
    mpu6050_calibrate(I2C_MASTER_NUM, acc_bias, gyro_bias);
    ESP_LOGI("MPU6050", "Calibration completed: accel bias [%.2f, %.2f, %.2f] m/s^2, gyro bias [%.2f, %.2f, %.2f] deg/s",
             acc_bias[0], acc_bias[1], acc_bias[2],
             gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    while(1) 
    {
        // Đọc dữ liệu thô
        ret = mpu6050_read_raw_data(I2C_MASTER_NUM, &accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
        if (ret != ESP_OK) {
            ESP_LOGE("MPU6050", "Read failed");
            return;
        }
        // Chuyển đổi dữ liệu thô sang đơn vị vật lý và áp dụng hiệu chỉnh bias
        mpu6050_convert_accel(accel_x, accel_y, accel_z, &accel_x_g, &accel_y_g, &accel_z_g);
        mpu6050_convert_gyro(gyro_x, gyro_y, gyro_z, &gyro_x_dps, &gyro_y_dps, &gyro_z_dps);

        // In ra console
        printf("Accel: X=%0.2f m/s^2, Y=%0.2f m/s^2, Z=%0.2f m/s^2\n", accel_x_g, accel_y_g, accel_z_g);
        printf("Gyro: X=%0.2f deg/s, Y=%0.2f deg/s, Z=%0.2f deg/s\n", gyro_x_dps, gyro_y_dps, gyro_z_dps);

        vTaskDelay(200 / portTICK_PERIOD_MS); // Delay between readings
    }
}
