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

// --- IMU Global ---
IMU_9DOF imu_9dof;
imu_data_t imu_data;

// --- Roll, pitch ----
float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;
float mag_x, mag_y, mag_z;

float initial_acc_x, initial_acc_y, initial_acc_z;
float initial_gyro_x, initial_gyro_y, initial_gyro_z;
float initial_mag_x, initial_mag_y, initial_mag_z;

float initial_roll, initial_pitch, initial_yaw;
float roll, pitch, yaw = 0;


// --- Explicit Comp Filter
const float dt = 0.05;
const float kI = 0.2;
const float kP = 1;
BLA::Matrix<3> b = {0.0, 0.0, 0.0};

BLA::Matrix<3> g0 = {0, 0, 1};
BLA::Matrix<3> m0 = {0.5281, -0.1023, 0.8430};


// --- Quaternion ---
UnitQuaternion attitude;

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


    // --- Testing skew matrix ---
    BLA::Matrix<4> v_test = {1.0, 2.0, 3.0, 4.0};
    Serial << "v_test: " << v_test << "\n";

    BLA::Matrix<4,4> v_skew_matrix;
    v_skew_matrix = skew_matrix_4d(v_test);
    Serial << "skew(v): " << v_skew_matrix << "\n";


    BLA::Matrix<3> x_test = {1.0, 2.0, 3.0};
    Serial << "x_test: " << x_test << "\n";

    BLA::Matrix<3,3> x_skew_matrix;
    x_skew_matrix = skew_matrix_3d(x_test);
    Serial << "skew(x): " << x_skew_matrix << "\n";


    // --- Testing cross product ---
    BLA::Matrix<3> a = {1.0, 2.0, 3.0};
    BLA::Matrix<3> b = {45.0, 6.0, 7.0};

    Serial << "a: " << a << "\n";
    Serial << "b: " << b << "\n";
    Serial << "a x b: " << cross_product(a,b) << "\n";

    // --- Grab initial Readings ---
    // Create sensors_event_t in memory to hold accel, gyro, mag and temp readings
    sensors_event_t accel, gyro, mag, temp;

    // Get new readings from 9DOF IMU
    imu_9dof.lsm6ds.getEvent(&accel, &gyro, &temp);
    imu_9dof.lis3mdl.getEvent(&mag);

    // Accelerometer (m/s^2)
    initial_acc_x = accel.acceleration.x;
    initial_acc_y = accel.acceleration.y;
    initial_acc_z = accel.acceleration.z;

    // Gyro (rad/s)
    initial_gyro_x = gyro.gyro.x;
    initial_gyro_y = gyro.gyro.y;
    initial_gyro_z = gyro.gyro.z;

    // Magnetometer (Tesla)
    initial_mag_x = mag.magnetic.x * 1e-6;
    initial_mag_y = mag.magnetic.y * 1e-6;
    initial_mag_z = mag.magnetic.z * 1e-6;


    UnitQuaternion uq_omega = UnitQuaternion::omega(0.0097, 0.0202, 0.0193);
    attitude = attitude * uq_omega;

    Serial << "attitude to vec" << "\n";
    Serial << attitude.to_vector() << "\n";
    Serial << "attitude inverse" << "\n";
    Serial << attitude.inverse().to_vector() << "\n";


    // --- Testing cross product for ecf ---
    UnitQuaternion attitude_ecf(-0.76165, 0.53608, -0.32176, -0.1702);
    UnitQuaternion invq_attitude_ecf = attitude_ecf.inverse();
    Serial << "attitude ecf: " << attitude_ecf.to_vector() << "\n";
    Serial << "inv(attitude ecf): " << invq_attitude_ecf.to_vector() << "\n";
    Serial << "invq * g0: " << invq_attitude_ecf.vector_rotation_by_quaternion(g0) << "\n";

    // Dummy bias
    BLA::Matrix<3> b_dummy = {-0.0953, 0.2111, -0.0953};

    // Dummy measurement
    BLA::Matrix<3> gm = {-0.6539, -0.7269, 0.2561};
    BLA::Matrix<3> mm = {-0.2216, -0.9748, 0.2584};
    BLA::Matrix<3> wm = {-0.6472, 0.1645, -0.0353};

    BLA::Matrix<3> sigma_dummy;
    sigma_dummy = cross_product(gm, invq_attitude_ecf.vector_rotation_by_quaternion(g0)) + 
                cross_product(mm, invq_attitude_ecf.vector_rotation_by_quaternion(m0));
    Serial << "sigmaR dummy: " << sigma_dummy << "\n";
    Serial.println(sigma_dummy(0),4);
    Serial.println(sigma_dummy(1),4);
    Serial.println(sigma_dummy(2),4);

    BLA::Matrix<3> wp_dummy = wm - b_dummy + (sigma_dummy * kP);
    Serial << "wp dummy: " << wp_dummy << "\n";

    // Attitude ecf update
    UnitQuaternion uq_omega_test = UnitQuaternion::omega(wm(0)*dt, wm(1)*dt, wm(2)*dt);
    Serial << "attitude_ecf next timestep: " << (attitude_ecf * uq_omega_test).to_vector() << "\n";

    // Bias update
    BLA::Matrix<3> bias_dummy_next = b_dummy - sigma_dummy * dt * kI;
    Serial << "bias next timestep: " << bias_dummy_next << "\n";
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

    // Store data into matrices
    BLA::Matrix<3> acc_measurement = {accel.acceleration.x, 
                                    accel.acceleration.y, 
                                    accel.acceleration.z};

    BLA::Matrix<3> gyro_measurement = {gyro.gyro.x,
                                    gyro.gyro.y,
                                    gyro.gyro.z};

    BLA::Matrix<3> mag_measurement = {mag.magnetic.x * 1e-6,
                                    mag.magnetic.y * 1e-6,
                                    mag.magnetic.z * 1e-6};

    // --- Calculate Roll, pitch, yaw  
    // float dt = 0.01;
    // UnitQuaternion uq_omega = UnitQuaternion::omega(gyro_x*dt, gyro_y*dt, gyro_z*dt);
    // attitude = attitude * uq_omega;

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
    b =  b - sigmaR * dt * kI;



    // --- Put attitude to serial ---
    // Serial << "attitude quat: " << attitude.to_vector() << "\n";


    Serial.print(constrain_angle(attitude.get_roll()) * 180.0 / PI);
    Serial.print("\t");
    Serial.print(constrain_angle(attitude.get_pitch()) * 180.0 / PI);
    Serial.print("\t");
    Serial.print(constrain_angle(attitude.get_yaw()) * 180.0 / PI);
    Serial.println();
}



