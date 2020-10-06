/***
Roll Pitch Estimation using Accelerometer
https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
***/
#include "IMU_9DOF.h"
#include "math_helpers.h"
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
}

void loop() 
{

}



