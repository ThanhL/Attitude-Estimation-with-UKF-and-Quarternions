/***
Roll Pitch Estimation using Accelerometer
https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
***/
#include "IMU_9DOF.h"
#include "math_helpers.h"
#include "Quaternion.h"
#include <BasicLinearAlgebra.h>

// --- Namespaces ---
using namespace BLA;

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
BLA::Matrix<3> initial_acc_measurement;
BLA::Matrix<3> initial_gyro_measurement;
BLA::Matrix<3> initial_mag_measurement;

// --- Explicit Comp Filter
const float dt = 0.05;
const float kI = 0.2;
const float kP = 1;
BLA::Matrix<3> b = {0.0, 0.0, 0.0};

BLA::Matrix<3> g0 = {0, 0, 1};
BLA::Matrix<3> m0 = {B_INTENSITY * cos(INCLINATION), 0.0, B_INTENSITY * sin(INCLINATION)};
//BLA::Matrix<3> m0 = {0.5281, -0.1023, 0.8430};

// --- Estimated Attitude Quaternion ---
UnitQuaternion attitude;

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

    // --- Calculate Roll, pitch, yaw  
    // Inverse estimated pose from {B} to {0} (a.k.a pose from {0} to {B})
    UnitQuaternion invq = attitude.inverse();
    
    // sigmaR is calculated in body frame {B} this is so that EQN 3.26
    // could work when summing with wm that is in frame {B}
    BLA::Matrix<3> sigmaR = 
            cross_product(acc_measurement, invq.vector_rotation_by_quaternion(g0)) + 
            cross_product(mag_measurement, invq.vector_rotation_by_quaternion(m0));

    // Exponential term
    BLA::Matrix<3> wp = gyro_measurement - b + (sigmaR * kP);
    
    // Estimated attitude update with incremental rotation update
    // EQN 3.26 & EQN 3.17 (Exponential with skew matrix and delta_t)
    UnitQuaternion uq_omega = UnitQuaternion::omega(wp(0)*dt, 
                                                wp(1)*dt, 
                                                wp(2)*dt);
    attitude = attitude * uq_omega;

    // Bias update term
    b =  b - (sigmaR * dt * kI);


    // Calculate yaw angle (Robotics, Vision and Control p87 yaw angle eqn)
    float yaw = atan2(cos(attitude.get_pitch()) * 
        (mag_measurement(2) * sin(attitude.get_roll()) - mag_measurement(1) * cos(attitude.get_roll())), 
        mag_measurement(0) + B_INTENSITY * sin(INCLINATION) * sin(attitude.get_pitch()));


    // --- Put attitude to serial ---
    Serial << "uq_omega quat: " << uq_omega.to_vector() << "\n";
    Serial << "attitude quat: " << attitude.to_vector() << "\n";
    // BLA::Matrix<3> rpy = attitude.to_rpy();
    // Serial.print(constrain_angle(rpy(0)) * 180.0 / PI);
    // Serial.print("\t");
    // Serial.print(constrain_angle(rpy(1)) * 180.0 / PI);
    // Serial.print("\t");
    // Serial.print(constrain_angle(rpy(2)) * 180.0 / PI);
    // Serial.println();

    Serial.print(constrain_angle(attitude.get_roll()) * 180.0 / PI);
    Serial.print("\t");
    Serial.print(constrain_angle(attitude.get_pitch()) * 180.0 / PI);
    Serial.print("\t");
    //Serial.print(yaw * 180.0 / PI);
    Serial.print(constrain_angle(attitude.get_yaw()) * 180.0 / PI);
    Serial.println();
}





