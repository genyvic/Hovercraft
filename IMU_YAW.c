/* =============================================================================
 * imu_yaw.c  –  MPU6050 driver: calibration + yaw integration from the
 * gyro's Z axis. main.c reads `yaw` and drives the steering servo itself
 * via servo_set_yaw(); this file only tracks heading.
 * =============================================================================*/

#include "TWI_UART_GPIO.h"

#define GZ_DEADZONE 0.3f   // ignore gyro-z noise below this (deg/s) so we don't drift while sitting still

#define MPU_ADDR    0x68

/* Calibration offsets */
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;

float yaw = 0.0f; // integrated heading, degrees, wrapped to [-180, 180]

/* Sensitivity tables */
uint8_t gyro_setting  = 0;
uint8_t accel_setting = 0;
uint8_t gyro_config[]  = {0x00, 0x08, 0x10, 0x18};
float   gyro_sens[]    = {131.0f, 65.5f, 32.8f, 16.4f};
uint8_t accel_config[] = {0x00, 0x08, 0x10, 0x18}; // full-scale-range bits written to MPU6050 reg 0x1C at init

// angle in degrees, +/-90 range; maps to a 1-2ms servo pulse centered at 1.5ms
void servo_set_yaw(float angle) {
    float pulse_ms = 1.5f + (angle / 90.0f);
    OCR1A = (uint16_t)(pulse_ms * 2000.0f);
}

// averages readings with the craft sitting still to find each axis's
// zero-offset; retries until every axis settles within its deadzone
// or we run out of tries
void imu_calibrate(void) {
    int32_t gx_sum, gy_sum, gz_sum;
    int32_t ax_sum, ay_sum, az_sum;
    uint8_t sensor_data[14];
    int gyro_deadzone  = 1;
    int accel_deadzone = 8;
    int ready_count    = 0;

    gx_offset = 0; gy_offset = 0; gz_offset = 0;
    ax_offset = 0; ay_offset = 0; az_offset = 0;

    int maximumTries = 10;
    while (maximumTries--) {
        ready_count = 0;
        gx_sum = 0; gy_sum = 0; gz_sum = 0;
        ax_sum = 0; ay_sum = 0; az_sum = 0;
        int bufferSize = 300;

        for (int i = 0; i < (bufferSize + 101); i++) {
            Read_Reg_N(MPU_ADDR, 0x3B, 14, (int16_t*)sensor_data);
            if (i > 100) {
                ax_sum += (int16_t)((sensor_data[0]<<8)|sensor_data[1])  - ax_offset;
                ay_sum += (int16_t)((sensor_data[2]<<8)|sensor_data[3])  - ay_offset;
                az_sum += ((int16_t)((sensor_data[4]<<8)|sensor_data[5]) - 16384) - az_offset;
                gx_sum += (int16_t)((sensor_data[8]<<8)|sensor_data[9])  - gx_offset;
                gy_sum += (int16_t)((sensor_data[10]<<8)|sensor_data[11])- gy_offset;
                gz_sum += (int16_t)((sensor_data[12]<<8)|sensor_data[13])- gz_offset;
            }
            delay_ms(2);
        }

        int16_t mean_ax = ax_sum / bufferSize;
        int16_t mean_ay = ay_sum / bufferSize;
        int16_t mean_az = az_sum / bufferSize;
        int16_t mean_gx = gx_sum / bufferSize;
        int16_t mean_gy = gy_sum / bufferSize;
        int16_t mean_gz = gz_sum / bufferSize;

        if (abs(mean_ax) <= accel_deadzone) ready_count++; else ax_offset += mean_ax;
        if (abs(mean_ay) <= accel_deadzone) ready_count++; else ay_offset += mean_ay;
        if (abs(mean_az) <= accel_deadzone) ready_count++; else az_offset += mean_az;
        if (abs(mean_gx) <= gyro_deadzone)  ready_count++; else gx_offset += mean_gx;
        if (abs(mean_gy) <= gyro_deadzone)  ready_count++; else gy_offset += mean_gy;
        if (abs(mean_gz) <= gyro_deadzone)  ready_count++; else gz_offset += mean_gz;

        if (ready_count == 6) break;
    }
}

// reads gyro-z over I2C and integrates it into `yaw`.
// doesn't touch the servo -- main.c decides what to do with the heading.
void runServo(float dt) {
    uint8_t gyro_data[6];
    Read_Reg_N(MPU_ADDR, 0x43, 6, (int16_t*)gyro_data);
    int16_t gz_raw = (int16_t)((gyro_data[4] << 8) | gyro_data[5]);

    float gz = (gz_raw - gz_offset) / gyro_sens[gyro_setting];
    if (fabs(gz) < GZ_DEADZONE) gz = 0.0f; // ignore noise so we don't drift while sitting still

    yaw += gz * dt;
    if (yaw >  180.0f) yaw -= 360.0f;
    if (yaw < -180.0f) yaw += 360.0f;
}