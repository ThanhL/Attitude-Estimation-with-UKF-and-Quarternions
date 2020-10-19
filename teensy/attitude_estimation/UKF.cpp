#include "UKF.h"
#include "debugging_helpers.h"

/*** ------ Sigma points --------- ***/
MerwedSigmaPoints::MerwedSigmaPoints(int n)
{
	// Num sigma points
	this->n = n;
	this->num_sigma_points = int(2*n + 1);

	// Default sigma points params
	this->alpha = 3.0;
	this->beta = 2.0;
	this->kappa = 0.1;
}

MerwedSigmaPoints::MerwedSigmaPoints(int n, float alpha, float beta, float kappa)
{
	// Num sigma points
	this->n = n;
	this->num_sigma_points = int(2*n + 1);

	// Default sigma points params
	this->alpha = alpha;
	this->beta = beta;
	this->kappa = kappa;
}


/*** Weight computation ***/
Eigen::VectorXd MerwedSigmaPoints::compute_Wm()
{
	// Compute lambda
	float lambda_ = alpha*alpha * (n + kappa) - n;

	// Initialize Wm weight array 
	// BLA::Matrix<2*n + 1> Wm;
	Wm = Eigen::VectorXd(num_sigma_points);

	// Compute initial weight
	Wm(0) = lambda_ / (n + lambda_);

	// Compute the rest of the weight
	for (int i = 1; i < 2*n+1; i++)
	{
		Wm(i) = 1.0 / (2*(n + lambda_));
	}

	return Wm;

}

Eigen::VectorXd MerwedSigmaPoints::compute_Wc()
{
	// Compute lambda
	float lambda_ = alpha*alpha * (n + kappa) - n;

	// Initialize Wm weight array 
	// BLA::Matrix<2*n + 1> Wc;
	Wc = Eigen::VectorXd(num_sigma_points);

	// Compute initial weight
	Wc(0) = (lambda_ / (n + lambda_)) + 1 - alpha*alpha + beta;

	// Compute the rest of the weight
	for (int i = 1; i < 2*n+1; i++)
	{
		Wc(i) = 1.0 / (2*(n + lambda_));
	}

	return Wc;

}

/*** Sigma point calculation ***/
Eigen::MatrixXd MerwedSigmaPoints::calculate_sigma_points(Eigen::VectorXd mean, Eigen::MatrixXd cov)
{
	/***
	Calculates Merwed Sigma Points
	Inputs:
	mean: nx1 matrix of state mean
	cov: nxn covariance matrix
	***/
	// Init sigma point array
	Eigen::MatrixXd sigma_points = Eigen::MatrixXd::Zero(num_sigma_points,n);

	// Square root of (n + lambda) * cov
	float lambda_ = alpha*alpha * (n + kappa) - n;
	Eigen::MatrixXd n_plus_lambda_times_cov = (n + lambda_) * cov;

	Eigen::LLT<Eigen::MatrixXd> lltOfA(n_plus_lambda_times_cov);	// compute the Cholesky decomposition of A
	Eigen::MatrixXd U = lltOfA.matrixU();							// retrieve factor U  in the decomposition (upper)

	// Calculate sigma points
	sigma_points.row(0) = mean;
	for (int i = 1; i < num_sigma_points; i++)
	{
		if (i <= n)
		{
			sigma_points.row(i) = mean.transpose() + U.row(i-1);
		}
		else
		{
			sigma_points.row(i) = mean.transpose() - U.row(i-n-1);
		}
	}

	return sigma_points;
}