#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "mpu6050.h"

#define I2C_MASTER_SCL_IO       22    // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO       21    // GPIO number for I2C master
#define I2C_MASTER_NUM          I2C_NUM_0 // I2C port number for master device
#define I2C_MASTER_FREQ_HZ      400000 // I2C master clock frequency
#define I2C_MASTER_FLAG_DEFAULT 0

void app_main(void)
{
    esp_err_t ret;
    float acc_bias[3], gyro_bias[3];

    // Configure I2C master
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

    // Initialize MPU6050
    ret = mpu6050_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Failed to initialize MPU6050: %s", esp_err_to_name(ret));
        return;
    }

    // Calibrate MPU6050 and get bias values
    mpu6050_calibrate_get_bias(I2C_MASTER_NUM, acc_bias, gyro_bias);

    while(1) 
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
