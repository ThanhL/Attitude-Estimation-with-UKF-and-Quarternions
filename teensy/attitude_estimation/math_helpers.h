/*  
Math helpers header file
*/
#include "Arduino.h"
#include "Eigen.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

float constrain_angle(float x);
Eigen::MatrixXd skew_matrix(Eigen::VectorXd x);