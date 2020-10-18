/***
Roll Pitch Estimation using Accelerometer
https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
***/
#include "IMU_9DOF.h"
#include "math_helpers.h"
#include "debugging_helpers.h"
#include "Quaternion.h"
#include "UKF.h"
#include <BasicLinearAlgebra.h>

#include "Eigen.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

// --- Namespaces ---
using namespace BLA;
using namespace Eigen;

// --- Magnetometer Values ---
// Constants dervied from location: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
#define INCLINATION -68.5006 * (PI/180.0)      // Inclination Angle (rads) 
#define DECLINATION 11.4017 * (PI/180.0)       // Declination Angle (rads)
#define B_INTENSITY 21951.5e-9                 // Magnetic Field Intensity (Tesla)

// --- IMU Global ---
IMU_9DOF imu_9dof;
imu_data_t imu_data;

// --- Roll, pitch ----
BLA::Matrix<3> acc_measurement;
BLA::Matrix<3> gyro_measurement;
BLA::Matrix<3> mag_measurement;

// Initial measurements
BLA::Matrix<3> initial_acc_measurement = {0.0, 0.0, 0.0};
BLA::Matrix<3> initial_gyro_measurement = {0.0, 0.0, 0.0};
BLA::Matrix<3> initial_mag_measurement = {0.0, 0.0, 0.0};

// --- Unscented Kalman Filter 
Eigen::Vector3d g0(0,0,1);
Eigen::Vector3d m0(B_INTENSITY * cos(INCLINATION), 0.0, B_INTENSITY * sin(INCLINATION));


/*** sigma points ***/
const float alpha = 0.3;
const float beta = 2.0;
const float kappa = 0.1;

MerwedSigmaPoints sigma_points(alpha, beta, kappa);


// --- Estimated Attitude Quaternion ---
UnitQuaternion attitude;

// --- Delta time calculations 
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

    // --- Grab initial Readings ---
    // Create sensors_event_t in memory to hold accel, gyro, mag and temp readings
    sensors_event_t accel, gyro, mag, temp;

    // Get new readings from 9DOF IMU
    imu_9dof.lsm6ds.getEvent(&accel, &gyro, &temp);
    imu_9dof.lis3mdl.getEvent(&mag);

    // Accelerometer (m/s^2)
    initial_acc_measurement = {accel.acceleration.x, 
                            accel.acceleration.y, 
                            accel.acceleration.z};


    // Gyro (rad/s)
    initial_gyro_measurement = {gyro.gyro.x,
                            gyro.gyro.y,
                            gyro.gyro.z};

    // Magnetometer (Tesla)
    initial_mag_measurement = {mag.magnetic.x * 1e-6,
                            mag.magnetic.y * 1e-6,
                            mag.magnetic.z * 1e-6};

    // --- Testing SigmaPoints ---

    // --- Testing Cholesky Decomp ---


    // --- Testing Quaternion Rotation ---
    UnitQuaternion attitude_ecf(-0.76165, 0.53608, -0.32176, -0.1702);
    UnitQuaternion invq_attitude_ecf = attitude_ecf.inverse();
    
    Eigen::Vector4d invq_vec = invq_attitude_ecf.to_quaternion_vector();
    print_mtxd(invq_vec);

    // Serial << "inv(attitude ecf): " << invq_attitude_ecf.to_vector() << "\n";
    // Serial << "invq * g0: " << invq_attitude_ecf.vector_rotation_by_quaternion(g0) << "\n";

    // --- Testing Skew matrix ---
    Eigen::Vector3d x_test(1.0, 2.0, 3.0);
    Eigen::Vector4d x_test_4d(1.0, 2.0, 3.0, 4.0);
    print_mtxd(skew_matrix(x_test));
    print_mtxd(skew_matrix(x_test_4d));
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
    acc_measurement = {accel.acceleration.x, 
                    accel.acceleration.y, 
                    accel.acceleration.z};

    // Gyro (rad/s)
    gyro_measurement = {gyro.gyro.x,
                    gyro.gyro.y,
                    gyro.gyro.z};

    // Magnetometer (Tesla)
    mag_measurement = {mag.magnetic.x * 1e-6,
                    mag.magnetic.y * 1e-6,
                    mag.magnetic.z * 1e-6};

    // Calculate time difference
    float dt = calculate_delta_time();

    // --- Calculate Roll, pitch, yaw  


}





