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


BLA::Matrix<3> cross_product(BLA::Matrix<3> a, BLA::Matrix<3> b)
{
	// Implementation of cross product: a x b
	// Cross product implementation with skew matrices
	BLA::Matrix<3,3> skew_matrix_a = skew_matrix_3d(a);
	return skew_matrix_a * b;
}


Eigen::MatrixXd skew_matrix(Eigen::VectorXd x)
{
	// Check matrix dimension
	int nrows = x.rows();

	// 4D matrix
	if (nrows == 4)
	{
		Eigen::MatrixXd skew(4,4);
		skew << 0, -x(0), x(1), -x(2),
			x(0), 0, x(2), x(3),
			-x(1), -x(2), 0, -x(0),
			x(2), -x(3), x(0), 0;
		return skew;
	} 
	else if (nrows == 3)
	{
		Eigen::MatrixXd skew(3,3);
		skew << 0.0, -x(2), x(1),
			x(2), 0.0, -x(0),
			-x(1), x(0), 0.0;;
		return skew;
	}
	else
	{
		// Just return the matrix for now
		return x;
	}
	
}