#ifndef UKF_H
#define UKF_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"
#include "Eigen.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <math.h>
#include <tuple>

#include "Quaternion.h"

// Hacky Soln to template matrix from BasicLinearAlgebra library. Not best practice
// TODO: find a better of creating this class that doesn't rely on
// global variables
// const int n = 7;		// State space dimensions x_state = [q0 q1 q2 q3 wx wy wz].T
// const int num_sigma_points = 2*n + 1;

// --- Global Frame Values ---
// Magnetometer
// Constants dervied from location: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
#define INCLINATION -68.5006 * (PI/180.0)      // Inclination Angle (rads) 
#define DECLINATION 11.4017 * (PI/180.0)       // Declination Angle (rads)
#define B_INTENSITY 21951.5e-9                 // Magnetic Field Intensity (Tesla)

// Inertial reference frame of gravity and magnetometer (will be used in prediction for UKF)
// Eigen::Vector3d g0(0,0,1);
// Eigen::Vector3d m0(B_INTENSITY * cos(INCLINATION), 0.0, B_INTENSITY * sin(INCLINATION));


// Merwe Scaled Sigma points for UKF
class MerwedSigmaPoints
{
public:
	// Sigma points
	int n;
	int num_sigma_points;

	// Sigma point parameters
	double alpha, beta, kappa;

	// Sigma point Weights 
	Eigen::VectorXd Wm;
	Eigen::VectorXd Wc;

	/*** Constructors ***/
	MerwedSigmaPoints();		// Default Constructor
	MerwedSigmaPoints(int n);
	MerwedSigmaPoints(int n, double alpha, double beta, double kappa);

	/*** Destructors ***/
	virtual ~MerwedSigmaPoints();

	/*** Weight Computations ***/
	Eigen::VectorXd compute_Wm();
	Eigen::VectorXd compute_Wc();

	/*** Sigma Points ***/
	Eigen::MatrixXd calculate_sigma_points(Eigen::VectorXd mean, Eigen::MatrixXd cov);

	void debug();
};


// Unscented kalman filter for orientation estimation 
class UKF
{
public:
	/*** UKF State Variables ***/
	// State vector: [q0 q1 q2 q3 wx wy wz].T
	int x_dim;					// State dimension
	Eigen::VectorXd x_hat;		// Estimated State (mean)
	Eigen::VectorXd x_prior;	// x_state prediction (or x_bar)
	Eigen::VectorXd x_post;		// x_state poseterior

	// Measurement vector: []
	int z_dim;
	Eigen::VectorXd z;			// z_state
	Eigen::VectorXd z_prior;	// z_state prediction (or z_bar)

	// Posteriori Estimate Covariance Matrix 
	Eigen::MatrixXd P;			// Posteriori estimate covariance matrix
	Eigen::MatrixXd P_prior;	// Priori prediction cov matrix
	Eigen::MatrixXd P_post;		// Posteriori cov matrix cache

	/*** UKF Sigma Points ***/
	// Sigma points
	MerwedSigmaPoints sigma_points;
	// Eigen::MatrixXd sigma_points;

	// Transformed sigma points (after being passed through f(x) and h(x))
	Eigen::MatrixXd sigmas_f;	// Predicted sigma points gamma = f(x,t) (prob book: g(u_t,sigma_t-1))
	Eigen::MatrixXd sigmas_h;	// Measurement sigma points Zbar = h(X_bar)

	/*** Noise Matrices ***/
	// Process noise covariance matrix
	Eigen::MatrixXd Q;

	// Observation noise covariance matrix
	Eigen::MatrixXd R;

	// Standard deviations 
	// // TODO: Probably remove, not needed just input into the Q,R matrices. 
	// double process_std_q0;		// process noise of quaternion
	// double process_std_q1; 		
	// double process_std_q2;
	// double process_std_q3;

	// double process_std_wx;
	// double process_std_wy;
	// double process_std_wz;

	// double meas_std_acc;	// measurement noise stddev of accelerometer in m/s^2
	// double meas_std_gyro;	// measurement noise stddev of gyroscope in rad/s^2
	// double meas_std_mag;	// measurement noise stddev of magnetometer in tesla

	Eigen::Vector3d g0;
	Eigen::Vector3d m0;

	/*** Constructors ***/
	UKF();
	UKF(MerwedSigmaPoints merwed_sigma_points);
	UKF(int x_dim_, int z_dim_, MerwedSigmaPoints merwed_sigma_points);

	/*** Destructors ***/
	virtual ~UKF();

	/*** Prediction + Update Steps ***/
	void predict(double dt);
	void predict_with_quaternion_model(double dt, Eigen::VectorXd u_t);

	void update(Eigen::MatrixXd z_measurement);
	void update_with_quaternion_model(Eigen::MatrixXd z_measurement);

	std::tuple<Eigen::VectorXd, Eigen::MatrixXd> unscented_transform(Eigen::MatrixXd sigmas,
																	Eigen::MatrixXd Wm,
																	Eigen::MatrixXd Wc,
																	Eigen::MatrixXd noise_cov);

	/*** Nonlinear functions **/
	Eigen::VectorXd f(Eigen::VectorXd x, double dt);
	Eigen::VectorXd h(Eigen::VectorXd x);

	Eigen::VectorXd f_quaternion(Eigen::VectorXd x, Eigen::VectorXd u_t, double dt);
	Eigen::VectorXd h_quaternion(Eigen::VectorXd x);

	// Dummy nonlinear functions for testing
	Eigen::VectorXd f_cv_radar(Eigen::VectorXd x, double dt);
	Eigen::VectorXd h_radar(Eigen::VectorXd x); 

	/*** Arduino Debugging ***/
	void debug();
};


#endif