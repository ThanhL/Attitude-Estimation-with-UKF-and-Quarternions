/***
Roll Pitch Estimation using Accelerometer
https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
***/
#include "IMU_9DOF.h"
#include "math_helpers.h"


// --- IMU Global ---
IMU_9DOF imu_9dof;
imu_data_t imu_data;

// --- Roll, pitch ----
float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;
float mag_x, mag_y, mag_z;

float initial_roll, initial_pitch, initial_yaw;
float roll, pitch, yaw = 0;


// --- Magnetometer Values ---
// Constants dervied from location: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
#define INCLINATION -68.5006 * (PI/180.0)      // Inclination Angle (rads) 
#define DECLINATION 11.4017 * (PI/180.0)       // Declination Angle (rads)
#define B_INTENSITY 21951.5e-9                 // Magnetic Field Intensity (Tesla)

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

    // --- Initial roll, pitch, yaw ---
    // Create sensors_event_t in memory to hold accel, gyro, mag and temp readings
    sensors_event_t accel, gyro, mag, temp;

    // Get new readings from 9DOF IMU
    imu_9dof.lsm6ds.getEvent(&accel, &gyro, &temp);
    imu_9dof.lis3mdl.getEvent(&mag);    

    // --- Store data ---
    // Accelerometer (m/s^2)
    acc_x = accel.acceleration.x;
    acc_y = accel.acceleration.y;
    acc_z = accel.acceleration.z;

    // Gyro (rad/s)
    gyro_x = gyro.gyro.x;
    gyro_y = gyro.gyro.y;
    gyro_z = gyro.gyro.z;

    // Magnetometer (Tesla)
    mag_x = mag.magnetic.x * 1e-6;
    mag_y = mag.magnetic.y * 1e-6;
    mag_z = mag.magnetic.z * 1e-6;    


    // --- Calcuate initial yaw ---
    // Calculate Roll and Pitch using eqn 28 and eqn 29
    initial_roll = atan(acc_y / sqrt(pow(acc_x, 2) + pow(acc_z, 2)));
    initial_pitch = atan(-1 * acc_x / sqrt(pow(acc_y, 2) + pow(acc_z, 2)));

    // Calculate yaw angle (Robotics, Vision and Control p87 yaw angle eqn)
    initial_yaw = atan2(cos(initial_pitch) * (mag_z * sin(initial_roll) - mag_y * cos(initial_roll)), 
            mag_x + B_INTENSITY * sin(INCLINATION) * sin(initial_pitch));    


    // Constrain angles to -pi to pi
    initial_roll = constrain_angle(initial_roll);
    initial_pitch = constrain_angle(initial_pitch);
    initial_yaw = constrain_angle(initial_yaw);
}

void loop() 
{
    // --- Update Readings ---
    // Create sensors_event_t in memory to hold accel, gyro, mag and temp readings
    sensors_event_t accel, gyro, mag, temp;

    // Get new readings from 9DOF IMU
    imu_9dof.lsm6ds.getEvent(&accel, &gyro, &temp);
    imu_9dof.lis3mdl.getEvent(&mag);

    // --- Store data ---
    // Accelerometer (m/s^2)
    acc_x = accel.acceleration.x;
    acc_y = accel.acceleration.y;
    acc_z = accel.acceleration.z;

    // Gyro (rad/s)
    gyro_x = gyro.gyro.x;
    gyro_y = gyro.gyro.y;
    gyro_z = gyro.gyro.z;

    // Magnetometer (Tesla)
    mag_x = mag.magnetic.x * 1e-6;
    mag_y = mag.magnetic.y * 1e-6;
    mag_z = mag.magnetic.z * 1e-6;


    // --- Calculate Roll, pitch, yaw
    // Calculate Roll and Pitch using eqn 28 and eqn 29
    roll = atan(acc_y / sqrt(pow(acc_x, 2) + pow(acc_z, 2)));
    pitch = atan(-1 * acc_x / sqrt(pow(acc_y, 2) + pow(acc_z, 2)));


    // // Calculate Roll and pitch (Robotics, Vision and Control p84, eqn 3.20 & eqn 3.21)
    // pitch = asin(-acc_x / 9.81);
    // roll = atan2(acc_y, acc_z);

    // Calculate yaw angle (Robotics, Vision and Control p87 yaw angle eqn)
    yaw = atan2(cos(pitch) * (mag_z * sin(roll) - mag_y * cos(roll)), 
            mag_x + B_INTENSITY * sin(INCLINATION) * sin(pitch));

    
    // Calculate yaw w.r.t to initial yaw
    yaw = yaw - initial_yaw;

    // // Calculate heading w.r.t to true magnetic north
    // yaw =  yaw - DECLINATION;

    // --- Constrain angles and conversion ---
    roll = constrain_angle(roll);
    pitch = constrain_angle(pitch);
    yaw = constrain_angle(yaw);

    // Convert to angles
    roll = roll * 180.0 / PI;
    pitch = pitch * 180.0 / PI;
    yaw = yaw * 180.0 / PI;

    // --- Put to serial ---
    Serial.print(roll);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(yaw);
    Serial.println();
}



// --- Serial and data packet stuff ---
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
 