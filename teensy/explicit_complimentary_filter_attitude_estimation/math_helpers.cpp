/*  
Math helpers implementation 
*/
#include "Arduino.h"
#include "math_helpers.h"
#include <BasicLinearAlgebra.h>


float constrain_angle(float x)
{
    x = fmod(x + PI, 2*PI);
    if (x < 0)
        x += 2*PI;
    return x - PI;
}


BLA::Matrix<4,4> skew_matrix_4d(BLA::Matrix<4> x)
{
	BLA::Matrix<4,4> mat;
	mat << 0, -x(0), x(1), -x(2),
		x(0), 0, x(2), x(3),
		-x(1), -x(2), 0, -x(0),
		x(2), -x(3), x(0), 0;
	return mat;
}


BLA::Matrix<3,3> skew_matrix_3d(BLA::Matrix<3> x)
{
	BLA::Matrix<3,3> mat;
	mat << 0.0, -x(2), x(1),
		x(2), 0.0, -x(0),
		-x(1), x(0), 0.0;
	return mat;
}


