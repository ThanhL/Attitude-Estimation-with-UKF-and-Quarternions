/*  
Math helpers implementation 
*/
#include "Arduino.h"
#include "math_helpers.h"

float constrain_angle(float x)
{
	/***
	Constrains angle to range [-pi,pi]
	***/
    x = fmod(x + PI, 2*PI);
    if (x < 0)
        x += 2*PI;
    return x - PI;
}


Eigen::MatrixXd skew_matrix(Eigen::VectorXd x)
{
	/***
	Returns the skew symmetric matrix of vector x
	***/
	// TODO: I don't think theres a 4d skew symmetric matrix. Need to double check...
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