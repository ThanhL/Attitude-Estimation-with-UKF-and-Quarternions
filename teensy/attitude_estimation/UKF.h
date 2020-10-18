#ifndef UKF_H
#define UKF_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"
#include <vector> 
#include <math.h>

// Hacky Soln to template matrix from BasicLinearAlgebra library. Not best practice
// TODO: find a better of creating this class that doesn't rely on
// global variables
const int n = 7;		// State space dimensions x_state = [q0 q1 q2 q3 wx wy wz].T
const int num_sigma_points = 2*n + 1;


// Merwed Sigma points for UKF
class MerwedSigmaPoints
{
public:
	// Sigma point parameters
	float alpha, beta, kappa;

	// Matrix 
	BLA::Matrix<2*n + 1> Wm;
	BLA::Matrix<2*n + 1> Wc;


	/*** Constructors ***/
	MerwedSigmaPoints();
	MerwedSigmaPoints(float alpha, float beta, float kappa);

	/*** Weight Computations ***/
	BLA::Matrix<2*n + 1> compute_Wm();
	BLA::Matrix<2*n + 1> compute_Wc();

	/*** Sigma Points ***/
	BLA::Matrix<2*n+1, n> calculate_sigma_points(BLA::Matrix<n>, BLA::Matrix<n,n>);
	// BLA::Matrix<2*n+1, n> calculate_sigma_points(BLA::Matrix<7>, BLA::Matrix<7,7>);

};


// Unscented kalman filter 
class UKF
{
public:
	UKF();
};


#endif