/***
Roll Pitch Estimation using Accelerometer
https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
***/
#include "IMU_9DOF.h"
#include "math_helpers.h"
#include "debugging_helpers.h"
#include "Quaternion.h"
#include "UKF.h"

#include "Eigen.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

// --- Namespaces ---
using namespace Eigen;

// --- IMU Global ---
// IMU sensor initialized
IMU_9DOF imu_9dof;

// --- Sensor Measurements ----
// Sensor measurements @ time t (i.e current sensor measurements)
Eigen::Vector3d acc_measurement;
Eigen::Vector3d gyro_measurement;
Eigen::Vector3d mag_measurement;

// Hard Iron offets (uT)
// These values were calculated using magnetometer calibration steps from jupyter notebook
// on adafruit's website: 
// https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration/magnetic-calibration-with-motioncal
const float mag_hardiron_offset_x = -31.71;
const float mag_hardiron_offset_y = 28.61;
const float mag_hardiron_offset_z = 33.985;

// Gryroscope offesets (rad/s)
// These values were calculated using gyroscope calibration steps from jupyter notebook
// on adafruit's website: 
// https://learn.adafruit.com/adafruit-sensorlab-gyroscope-calibration
const float gyro_offset_x = 0.06285;
const float gyro_offset_y = -0.08785;;
const float gyro_offset_z = -0.06815;;

/*** --- Unscented Kalman Filter --- ***/
/*** sigma points ***/
const int n_state_dim = 7;  // x_state dimension
const float alpha = 0.3;
const float beta = 2.0;
const float kappa = 0.1;

MerwedSigmaPoints sigma_points(n_state_dim, alpha, beta, kappa);

/*** Unscented Kalman Filter itself ***/
UKF quat_ukf(sigma_points);

// --- Delta time calculations ---
// Previous time to calculate delta time
unsigned long prev_time = 0;

float calculate_delta_time()
{
    // Calculate the time difference in seconds
    unsigned long current_time = millis();
    unsigned long delta_time = current_time - prev_time;
    prev_time = current_time;
    return delta_time * 1e-3;
}

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

    // --- Debug UKF before the main execution ---
    quat_ukf.debug();

}   

void loop() 
{
    // --- Update Readings ---
    // Create sensors_event_t in memory to hold accel, gyro, mag and temp readings
    sensors_event_t accel, gyro, mag, temp;

    // Get new readings from 9DOF IMU
    imu_9dof.lsm6ds.getEvent(&accel, &gyro, &temp);
    imu_9dof.lis3mdl.getEvent(&mag);

    // --- Store data into matrices ---
    // Accelerometer (m/s^2)
    acc_measurement << accel.acceleration.x, 
                    accel.acceleration.y, 
                    accel.acceleration.z;

    // Gyro (rad/s)
    gyro_measurement << gyro.gyro.x - gyro_offset_x,
                    gyro.gyro.y - gyro_offset_y,
                    gyro.gyro.z - gyro_offset_z;

    // Magnetometer (Tesla)
    mag_measurement << (mag.magnetic.x - mag_hardiron_offset_x) * 1e-6,
                    (mag.magnetic.y - mag_hardiron_offset_y) * 1e-6,
                    (mag.magnetic.z - mag_hardiron_offset_z) * 1e-6;

    // Calculate time difference
    float dt = calculate_delta_time();

    // --- Calculate Roll, pitch, yaw  
    // Predict step with UKF's quaternion with ang vec model
    quat_ukf.predict_with_quaternion_ang_vec_model(dt, gyro_measurement);

    // Cocantenate gyroscope, accelerometer and magnetometer for measurement
    // z_measurement = [z_gyro, z_acc, z_mag].T
    Eigen::VectorXd z_measurement(gyro_measurement.size() + acc_measurement.size() + mag_measurement.size());
    z_measurement << gyro_measurement, acc_measurement, mag_measurement;

    // Update step UKF with measurements 
    quat_ukf.update_with_quaternion_ang_vec_model(z_measurement);

    // --- Output to Serial ---
    // Quaternion states
    Serial.print(quat_ukf.x_hat(0));
    Serial.print("\t");  
    Serial.print(quat_ukf.x_hat(1));
    Serial.print("\t");  
    Serial.print(quat_ukf.x_hat(2));
    Serial.print("\t");  
    Serial.print(quat_ukf.x_hat(3));
    Serial.print("\t"); 

    // Angular Velocity states
    // Serial.print(quat_ukf.x_hat(4));
    // Serial.print("\t"); 
    // Serial.print(quat_ukf.x_hat(5));
    // Serial.print("\t"); 
    // Serial.print(quat_ukf.x_hat(6));
    // Serial.print("\t"); 


    Serial.println();
}





