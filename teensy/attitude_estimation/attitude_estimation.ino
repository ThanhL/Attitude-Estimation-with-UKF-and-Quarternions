/***
Attitude Estimation with Unscented Kalman Filters

***/
#include "IMU_9DOF.h"


// --- IMU Global ---
IMU_9DOF imu_9dof;

// --- EMBEDDED CORE! ---
void setup() 
{
    // --- Start the Serial ---
    Serial.begin(115200);

    // Pause Serial until console opens
    while (!Serial) 
        delay(10);

    // --- Intiialize hw modules over I2C, otherwise keep polling. ---
    imu_9dof.init_sensors_i2c();

    // --- Accelerometer setup ---
    imu_9dof.lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    imu_9dof.lsm6ds.setAccelDataRate(LSM6DS_RATE_12_5_HZ);

    // --- Gyroscope Setup ---
    imu_9dof.lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu_9dof.lsm6ds.setGyroDataRate(LSM6DS_RATE_12_5_HZ);

    // ---  Magnetometer Setup ---
    imu_9dof.lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
    imu_9dof.lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
    imu_9dof.lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    imu_9dof.lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    imu_9dof.lis3mdl.setIntThreshold(500);
    imu_9dof.lis3mdl.configInterrupt(false, false, true, // enable z axis
                                    true, // polarity
                                    false, // don't latch
                                    true); // enabled!

    // --- Output the current settings by 9DOF IMU ---
    imu_9dof.get_accelerometer_settings();
    imu_9dof.get_gyroscope_settings();
    imu_9dof.get_magnetometer_settings();
}

void loop() 
{
    // --- Update Readings ---
    // Create sensors_event_t in memory to hold accel, gyro, mag and temp readings
    sensors_event_t accel, gyro, mag, temp;

    // Get new readings from 9DOF IMU
    imu_9dof.lsm6ds.getEvent(&accel, &gyro, &temp);
    imu_9dof.lis3mdl.getEvent(&mag);

    // With the new readings "packetize" it and send over serial!
    send_imu_data_serial(&accel, &gyro, &mag);

}


void send_imu_data_serial(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *mag)
{
    // Store the imu data into struct
    imu_data_t imu_data;

    imu_data.acc_x = accel->acceleration.x;
    imu_data.acc_y = accel->acceleration.y;
    imu_data.acc_z = accel->acceleration.z;

    imu_data.gyro_x = gyro->gyro.x;
    imu_data.gyro_y = gyro->gyro.y;
    imu_data.gyro_z = gyro->gyro.z;

    imu_data.mag_x = mag->magnetic.x;
    imu_data.mag_y = mag->magnetic.y;
    imu_data.mag_z = mag->magnetic.z;

    // Send the data pkt over serial
    Serial.write((const byte*)&imu_data, sizeof(imu_data));
}
