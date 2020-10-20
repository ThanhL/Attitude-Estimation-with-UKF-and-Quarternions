#include "UKF.h"
#include "debugging_helpers.h"

/*** ------ Sigma points --------- ***/
MerwedSigmaPoints::MerwedSigmaPoints()
{
	// Default Constructor with default parameters
	this->n = n;
	this->num_sigma_points = int(2*n + 1);

	// Default sigma points params
	this->alpha = 3.0;
	this->beta = 2.0;
	this->kappa = 0.1;
}


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


MerwedSigmaPoints::~MerwedSigmaPoints()
{
	// Destructor
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


/*** ------ Unscented Kalman Filter for Orientation Estimation ------ ***/
/*** Constructors ***/
UKF::UKF()
{
	// Initialize x state vector
	x_dim = 2;
	x_hat = Eigen::VectorXd::Zero(x_dim);
	// x << 1, 0, 0, 0, 0, 0, 0;	// Initial Quaterion: 1+0i+0j+0k and initial ang vel: [0 0 0].T
 
	// Initialize z state vector
	z_dim = 3;
	z = Eigen::VectorXd::Zero(z_dim);
	// z << 0, 0, 0;

	// Intialize Posteriori Estimate Covariance Matrix
	P = Eigen::MatrixXd::Zero(x_dim, x_dim);

	// Initialize Sigma Points
	sigma_points = MerwedSigmaPoints(x_dim);

	sigmas_f = Eigen::MatrixXd::Zero(sigma_points.num_sigma_points, x_dim);
	sigmas_h = Eigen::MatrixXd::Zero(sigma_points.num_sigma_points, z_dim);

	// Initialize noise matrices
	Q = Eigen::MatrixXd::Identity(x_dim, x_dim);
	
	R = Eigen::MatrixXd::Identity(z_dim, z_dim);
}


UKF::UKF(MerwedSigmaPoints merwed_sigma_points)
{
	// Initialize x state vector
	x_dim = 2;
	x_hat = Eigen::VectorXd::Zero(x_dim);
	// x << 1, 0, 0, 0, 0, 0, 0;	// Initial Quaterion: 1+0i+0j+0k and initial ang vel: [0 0 0].T
 
	// Initialize z state vector
	z_dim = 3;
	z = Eigen::VectorXd::Zero(z_dim);
	// z << 0, 0, 0;

	// Intialize Posteriori Estimate Covariance Matrix
	P = Eigen::MatrixXd::Zero(x_dim, x_dim);

	// Initialize Sigma Points
	sigma_points = merwed_sigma_points;

	sigmas_f = Eigen::MatrixXd::Zero(sigma_points.num_sigma_points, x_dim);
	sigmas_h = Eigen::MatrixXd::Zero(sigma_points.num_sigma_points, z_dim);

	// Initialize noise matrices
	Q = Eigen::MatrixXd::Identity(x_dim, x_dim);

	R = Eigen::MatrixXd::Identity(z_dim, z_dim);
}

/*** Destructor ***/
UKF::~UKF()
{
	// Destructor
}

/*** Prediction + Update ***/
void UKF::predict(double dt)
{



}

void UKF::update(Eigen::MatrixXd z_measurement)
{

}


/*** Arduino Debugging ***/
void UKF::debug()
{
	/*** State Space ***/
	Serial.println("x_hat:");
	print_mtxd(x_hat.transpose());

	Serial.println("z: ");
	print_mtxd(z.transpose());

	/*** Posterior Covariance ***/
	Serial.println("P: ");
	print_mtxd(P);

	/*** Sigmas ***/
	Serial.println("sigmas_f: ");
	print_mtxd(sigmas_f);

	Serial.println("sigmas_h: ");
	print_mtxd(sigmas_h);

	/*** Noise Matrices ***/
	Serial.println("Q: ");
	print_mtxd(Q);

	Serial.println("R: ");
	print_mtxd(R);
}