#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
// #include "roll_pitch.h"

#define I2C_MASTER_SCL_IO       22          
#define I2C_MASTER_SDA_IO       21          
#define I2C_MASTER_NUM          I2C_NUM_0   
#define I2C_MASTER_FREQ_HZ      400000     
#define I2C_MASTER_FLAG_DEFAULT 0

#define LED_PIN 2
#define BTN_PIN 3
#define BUZ_PIN 4



void gpio_conf(void) {
    // LED & Buzzer
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_PIN) | (1ULL << BUZ_PIN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    gpio_config(&io_conf);

    // Button
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BTN_PIN);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // I2C
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to configure I2C: %s", esp_err_to_name(ret));
        return;
    }
    ret = i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, I2C_MASTER_FLAG_DEFAULT);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return;
    }
}

// FreeRTOS Task đọc dữ liệu MPU6050
void mpu6050_task(void *param) {
    esp_err_t ret;
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float acc_bias[3] = {0}, gyro_bias[3] = {0};

    // Khởi tạo MPU6050
    ret = mpu6050_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Failed to initialize MPU6050: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    // Hiệu chỉnh bias (calibration)
    mpu6050_calibrate(I2C_MASTER_NUM, acc_bias, gyro_bias);
    ESP_LOGI("MPU6050", "Calibration done: accel bias [%.2f, %.2f, %.2f] m/s^2, gyro bias [%.2f, %.2f, %.2f] deg/s",
             acc_bias[0], acc_bias[1], acc_bias[2],
             gyro_bias[0], gyro_bias[1], gyro_bias[2]);

    while (1) {
        // Đọc dữ liệu thô từ MPU6050
        ret = mpu6050_read_raw_data(I2C_MASTER_NUM, &raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
        if (ret != ESP_OK) {
            ESP_LOGE("MPU6050", "Read failed");
            vTaskDelete(NULL);
            return;
        }

        // Chuyển đổi dữ liệu thô sang đơn vị vật lý (m/s^2 và deg/s)
        mpu6050_convert_accel(raw_ax, raw_ay, raw_az, &accel_x, &accel_y, &accel_z);
        mpu6050_convert_gyro(raw_gx, raw_gy, raw_gz, &gyro_x, &gyro_y, &gyro_z);

        // Tính gia tốc tổng hợp (Total Acceleration)
        float total_accel_ms2, total_accel_g;
        total_accel_ms2 = mpu6050_get_total_accel(accel_x, accel_y, accel_z, &total_accel_g);

        // Tính tốc độ góc tổng hợp (Total Gyroscope)
        float total_gyro = mpu6050_get_total_gyro(gyro_x, gyro_y, gyro_z);

        // In ra console
        printf("---------------------------------------\n");
        printf("Total Accel: %0.2f m/s^2 (%.2f g)\n", total_accel_ms2, total_accel_g);
        printf("Total Gyro: %0.2f deg/s\n", total_gyro);

        // Delay 200ms
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    gpio_conf();

    // Tạo FreeRTOS task cho MPU6050
    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 5, NULL);
}
