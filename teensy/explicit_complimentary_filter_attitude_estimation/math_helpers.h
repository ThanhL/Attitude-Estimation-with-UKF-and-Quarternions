/*  
Math helpers header file
*/
#include "Arduino.h"
#include "BasicLinearAlgebra.h"


float constrain_angle(float x);
BLA::Matrix<4,4> skew_matrix_4d(BLA::Matrix<4> x); 
BLA::Matrix<3,3> skew_matrix_3d(BLA::Matrix<3> x);



