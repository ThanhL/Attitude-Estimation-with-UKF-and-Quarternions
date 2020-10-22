/*  
LSM6DS33 LIS3MDL 9DOF IMU Wrapper
*/
#ifndef LSM6DS33_LIS3MDL_9DOF_IMU_h
#define LSM6DS33_LIS3MDL_9DOF_IMU_h

#include "Arduino.h"
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LIS3MDL.h>

#define LSM6DS33_I2C_ADDR 0x6A
#define LIS3MDL_I2C_ADDR 0x1E



// --- Imu Data ---
typedef struct 
{
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
    float mag_x, mag_y, mag_z;
} imu_data_t;


class IMU_9DOF
{
public:
    // Constructor
    IMU_9DOF();
    ~IMU_9DOF();

    // Initialize sensors over I2C
    void init_sensors_i2c(void);

    // Get sensor configuration
    void get_accelerometer_settings(void);
    void get_gyroscope_settings(void);
    void get_magnetometer_settings(void);

    // LSM6DS33 (accelerometer + gyro)
    Adafruit_LSM6DS33 lsm6ds;

    // LIS3MDL (magnetometer)
    Adafruit_LIS3MDL lis3mdl;
};

#endif