#ifndef UKF_H
#define UKF_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"
#include "Eigen.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <math.h>

// Hacky Soln to template matrix from BasicLinearAlgebra library. Not best practice
// TODO: find a better of creating this class that doesn't rely on
// global variables
// const int n = 7;		// State space dimensions x_state = [q0 q1 q2 q3 wx wy wz].T
// const int num_sigma_points = 2*n + 1;


// Merwed Sigma points for UKF
class MerwedSigmaPoints
{
public:
	// Sigma points
	int n;
	int num_sigma_points;

	// Sigma point parameters
	float alpha, beta, kappa;

	// Sigma point Weights 
	Eigen::VectorXd Wm;
	Eigen::VectorXd Wc;

	/*** Constructors ***/
	MerwedSigmaPoints(int n);
	MerwedSigmaPoints(int n, float alpha, float beta, float kappa);

	/*** Weight Computations ***/
	Eigen::VectorXd compute_Wm();
	Eigen::VectorXd compute_Wc();

	/*** Sigma Points ***/
	Eigen::MatrixXd calculate_sigma_points(Eigen::VectorXd mean, Eigen::MatrixXd cov);

};


// Unscented kalman filter 
class UKF
{
public:
	UKF();
};


#endif