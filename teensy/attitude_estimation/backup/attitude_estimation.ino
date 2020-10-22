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

// --- Sensor Measurements ----
Eigen::Vector3d acc_measurement;
Eigen::Vector3d gyro_measurement;
Eigen::Vector3d mag_measurement;

Eigen::Vector3d initial_acc_measurement(0.0, 0.0, 0.0);
Eigen::Vector3d initial_gyro_measurement(0.0, 0.0, 0.0);
Eigen::Vector3d initial_mag_measurement(0.0, 0.0, 0.0);

// --- Unscented Kalman Filter 
const int n_state_dim = 2;      // State space dimensions x_state = [q0 q1 q2 q3 wx wy wz].T

Eigen::Vector3d g0(0,0,1);
Eigen::Vector3d m0(B_INTENSITY * cos(INCLINATION), 0.0, B_INTENSITY * sin(INCLINATION));


/*** sigma points ***/
const float alpha = 0.3;
const float beta = 2.0;
const float kappa = 0.1;

MerwedSigmaPoints sigma_points(n_state_dim, alpha, beta, kappa);
UKF ukf(sigma_points);


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

    // --- Testing SigmaPoints ---
    // Sigma weights
    Eigen::VectorXd Wm = sigma_points.compute_Wm();
    Eigen::VectorXd Wc = sigma_points.compute_Wc();

    Serial.println("Wc:");
    print_mtxd(Wc.transpose());

    Serial.println("Wm:");
    print_mtxd(Wm.transpose());

    // Sigma points
    Serial.println("Sigma Points");
    
    Eigen::VectorXd mean(2);
    mean << 2.0, 0.0;

    Eigen::MatrixXd cov(2,2);
    cov << 32.0, 15.0,
        15.0, 40.0;

    print_mtxd(sigma_points.calculate_sigma_points(mean, cov));

    // --- Testing Cholesky Decomp ---


    // --- Testing Quaternion Rotation ---
    UnitQuaternion attitude_ecf(-0.76165, 0.53608, -0.32176, -0.1702);
    UnitQuaternion invq_attitude_ecf = attitude_ecf.inverse();
    
    Eigen::Vector4d invq_vec = invq_attitude_ecf.to_quaternion_vector();
    print_mtxd(invq_vec.transpose());

    // Serial << "inv(attitude ecf): " << invq_attitude_ecf.to_vector() << "\n";
    // Serial << "invq * g0: " << invq_attitude_ecf.vector_rotation_by_quaternion(g0) << "\n";

    // --- Testing Skew matrix ---
    Eigen::Vector3d x_test(1.0, 2.0, 3.0);
    Eigen::Vector4d x_test_4d(1.0, 2.0, 3.0, 4.0);

    Serial.println("Skew matrix 3D:");
    print_mtxd(skew_matrix(x_test));

    Serial.println("Skew matrix 4D:");
    print_mtxd(skew_matrix(x_test_4d));

    // --- Testing UKF ---
    ukf.debug();

    // --- Testing Unscented Transform ---
    const int n = 2;
    Eigen::VectorXd wm(2*n + 1);
    wm << 0, 1, 2, 3, 4;

    Eigen::VectorXd wc(2*n + 1);
    wc << 5, 6, 7, 8, 9;

    Eigen::MatrixXd sigmas(2*n + 1, n);
    sigmas << 0, 1,
            2, 1,
            0, 0,
            3, 4,
            1, 1;


    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n,n);


    Eigen::VectorXd mu;
    Eigen::MatrixXd P_cov;

    std::tie(mu, P_cov) = ukf.unscented_transform(sigmas, wm, wc, Q);


    Serial.println("---- Testing Unscented Transform -----");
    
    Serial.println("wm: ");
    print_mtxd(wm);

    Serial.println("wc: ");
    print_mtxd(wc);

    Serial.println("sigmas: ");
    print_mtxd(sigmas);

    Serial.println("mu: ");
    print_mtxd(mu);

    Serial.println("P_cov: ");
    print_mtxd(P_cov);


    // --- Testing UKF after one prediction ----
    delay(10);
    // Sigma Points
    MerwedSigmaPoints custom_sigma_points(4, 0.1, 2.0, -1.0);
    custom_sigma_points.debug();

    // UKF
    UKF custom_ukf(4, 2, custom_sigma_points);
    custom_ukf.x_hat << 0, 90, 1100, 0;
    custom_ukf.z << 0, 0;

    custom_ukf.P << 300*300, 0, 0, 0,
                    0, 3*3, 0, 0,
                    0, 0, 150*150, 0,
                    0, 0, 0, 3*3;

    custom_ukf.Q << 2.025, 1.35, 0.0, 0.0,
                    1.35, 0.9, 0.0, 0.0,
                    0.0, 0.0, 2.025, 1.35,
                    0.0, 0.0, 1.35, 0.9; 

    custom_ukf.R << 25, 0,
                    0, 0.000076;

    custom_ukf.debug();

    // Running After one predict
    custom_ukf.predict(3.0);

    Serial.println("Custom UKF x_prior after one predict: ");
    print_mtxd(custom_ukf.x_prior.transpose());

    Serial.println("Custom UKF P_prior after one predict: ");
    print_mtxd(custom_ukf.P_prior);

    // Running after one update
    Eigen::VectorXd z_dummy_time_0(2);     // Dummy measurement @ time t=0
    z_dummy_time_0 << 36694.415192643624, 0.07122009951682325;

    custom_ukf.update(z_dummy_time_0);

    Serial.println("Custom UKF x_post after one update: ");
    print_mtxd(custom_ukf.x_post.transpose());

    Serial.println("Custom UKF P_post after one update: ");
    print_mtxd(custom_ukf.P_post);
 
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
    gyro_measurement << gyro.gyro.x,
                    gyro.gyro.y,
                    gyro.gyro.z;

    // Magnetometer (Tesla)
    mag_measurement << mag.magnetic.x * 1e-6,
                    mag.magnetic.y * 1e-6,
                    mag.magnetic.z * 1e-6;

    // Calculate time difference
    float dt = calculate_delta_time();

    // --- Calculate Roll, pitch, yaw  
    // print_mtxd(acc_measurement.transpose());
    // print_mtxd(gyro_measurement.transpose());
    // print_mtxd(mag_measurement.transpose());
}





