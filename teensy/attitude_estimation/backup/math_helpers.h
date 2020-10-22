/*  
Math helpers header file
*/
#include "Arduino.h"
#include "Eigen.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include "BasicLinearAlgebra.h"


float constrain_angle(float x);
BLA::Matrix<4,4> skew_matrix_4d(BLA::Matrix<4> x); 
BLA::Matrix<3,3> skew_matrix_3d(BLA::Matrix<3> x);
BLA::Matrix<3> cross_product(BLA::Matrix<3> a, BLA::Matrix<3> b);

Eigen::MatrixXd skew_matrix(Eigen::VectorXd x);