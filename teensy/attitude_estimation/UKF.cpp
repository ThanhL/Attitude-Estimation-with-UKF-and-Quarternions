#include "UKF.h"


/*** ------ Sigma points --------- ***/
MerwedSigmaPoints::MerwedSigmaPoints()
{
	// Default sigma points params
	this->alpha = 3.0;
	this->beta = 2.0;
	this->kappa = 0.1;
}

MerwedSigmaPoints::MerwedSigmaPoints(float alpha, float beta, float kappa)
{
	// Default sigma points params
	this->alpha = alpha;
	this->beta = beta;
	this->kappa = kappa;
}


/*** Weight computation ***/
BLA::Matrix<2*n+1> MerwedSigmaPoints::compute_Wm()
{
	// Compute lambda
	float lambda_ = alpha*alpha * (n + kappa) - n;

	// Initialize Wm weight array 
	BLA::Matrix<2*n + 1> Wm;
	
	// Compute initial weight
	Wm(0) = lambda_ / (n + lambda_);

	// Compute the rest of the weight
	for (int i = 1; i < 2*n+1; i++)
	{
		Wm(i) = 1.0 / (2*(n + lambda_));
	}

	return Wm;
}

BLA::Matrix<2*n+1> MerwedSigmaPoints::compute_Wc()
{
	// Compute lambda
	float lambda_ = alpha*alpha * (n + kappa) - n;

	// Initialize Wm weight array 
	BLA::Matrix<2*n + 1> Wm;
	
	// Compute initial weight
	Wm(0) = (lambda_ / (n + lambda_)) + 1 - alpha*alpha + beta;

	// Compute the rest of the weight
	for (int i = 1; i < 2*n+1; i++)
	{
		Wm(i) = 1.0 / (2*(n + lambda_));
	}

	return Wm;
}

/*** Sigma point calculation ***/
BLA::Matrix<2*n+1, n> MerwedSigmaPoints::calculate_sigma_points(BLA::Matrix<n>, BLA::Matrix<n,n>)
{
	// Init sigma point array
	BLA::Matrix<2*n+1, n> sigma_points;

	// Square root of (n + lambda) * cov
	BLA::Matrix<n,n> U;


}