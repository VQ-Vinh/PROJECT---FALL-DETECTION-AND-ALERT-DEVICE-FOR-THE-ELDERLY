#include "roll_pitch.h"
#include <math.h>

#define ACC_SCALE   4096.0f     // Cho dải +- 8g
#define GYRO_SCALE  131.0f      // Cho dải +- 250 deg/s
#define GRAVITY     9.81f
#define DT          0.001f      // Thời gian giữa các lần cập nhật (1000 Hz)
#define ALPHA       0.98f       // Hệ số lọc cho complementary filter

static float roll = 0.0f;
static float pitch = 0.0f;

void roll_pitch_init(void) {
    roll = 0.0f;
    pitch = 0.0f;
}

void roll_pitch_update(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y, float gyro_z) {
    // Tính góc roll và pitch từ dữ liệu accelerometer
    float accel_roll = atan2f(accel_y, accel_z) * 180.0f / M_PI;
    float accel_pitch = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) * 180.0f / M_PI;

    // Giá trị đọc gyro tính bằng deg/s
    float roll_rate = gyro_x / GYRO_SCALE;
    float pitch_rate = gyro_y / GYRO_SCALE; 

    // Cập nhật góc roll và pitch bằng dữ liệu gyroscope
    roll += roll_rate * DT;
    pitch += pitch_rate * DT;

    // Kết hợp dữ liệu accelerometer và gyroscope bằng complementary filter
    roll = ALPHA * roll + (1.0f - ALPHA) * accel_roll;
    pitch = ALPHA * pitch + (1.0f - ALPHA) * accel_pitch;
}

float get_roll(void) {
    return roll;
}

float get_pitch(void) {
    return pitch;
}