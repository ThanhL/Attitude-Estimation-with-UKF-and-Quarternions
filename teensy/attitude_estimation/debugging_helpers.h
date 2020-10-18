/*  
LSM6DS33 LIS3MDL 9DOF IMU Wrapper
*/
#ifndef DEBUGGING_HELPERS_h
#define DEBUGGING_HELPERS_h

#include "Arduino.h"
#include "Eigen.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

// Matrix Printers
void print_mtxd(const Eigen::MatrixXd &X);		// Matrix of double type
void print_mtxf(const Eigen::MatrixXf &X);		// Matrix of float type


#endif