#include "UKF.h"
#include "debugging_helpers.h"

/*** ------ Sigma points --------- ***/
MerwedSigmaPoints::MerwedSigmaPoints()
{
    /***
    Merwe Sigma Point Default Constructor
    ***/    
    // Default Constructor with default parameters
    this->n = n;
    this->num_sigma_points = int(2*n + 1);

    // Default sigma points params
    this->alpha = 3.0;
    this->beta = 2.0;
    this->kappa = 0.1;

    // Compute the weight matrices
    this->Wm = compute_Wm();
    this->Wc = compute_Wc();
}


MerwedSigmaPoints::MerwedSigmaPoints(int n)
{
    /***
    Merwe Sigma Point Default Constructor

    Inputs:
    n: state dimension of state estimate
    ***/    
    // Num sigma points
    this->n = n;
    this->num_sigma_points = int(2*n + 1);

    // Default sigma points params
    this->alpha = 3.0;
    this->beta = 2.0;
    this->kappa = 0.1;

    // Compute the weight matrices
    this->Wm = compute_Wm();
    this->Wc = compute_Wc();
}


MerwedSigmaPoints::MerwedSigmaPoints(int n, double alpha, double beta, double kappa)
{
    /***
    Merwe Sigma Point Generalized Constructor

    Inputs:
    n: state dimension of state estimate
    alpha: alpha param for Merwe Sigma point
    beta: beta param of Merwe Sigma Point
    kappa: kappa param of Merwe Sigma Point
    ***/
    // Num sigma points
    this->n = n;
    this->num_sigma_points = int(2*n + 1);

    // Default sigma points params
    this->alpha = alpha;
    this->beta = beta;
    this->kappa = kappa;

    // Compute the weight matrices
    this->Wm = compute_Wm();
    this->Wc = compute_Wc();
}


MerwedSigmaPoints::~MerwedSigmaPoints()
{
    // Destructor
}

/*** Weight computation ***/
Eigen::VectorXd MerwedSigmaPoints::compute_Wm()
{
    /***
    Calculates The Weighted Mean for Merwe Sigma Points

    Outputs:
    Wm: (2n+1)x(n) Weight Mean
    ***/

    // Compute lambda
    double lambda_ = alpha*alpha * (n + kappa) - n;

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
    /***
    Calculates The Weighted Covariance for Merwe Sigma Points

    Outputs:
    Wc: (2n+1)x(n) Weight Covariance
    ***/

    // Compute lambda
    double lambda_ = alpha*alpha * (n + kappa) - n;

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
    Calculates Merwe Sigma Points
    Inputs:
    mean: nx1 matrix of state mean
    cov: nxn covariance matrix

    Outputs:
    sigma_points: (2n+1) Merwe Sigma Points
    ***/
    // Init sigma point array
    Eigen::MatrixXd sigma_points = Eigen::MatrixXd::Zero(num_sigma_points,n);

    // Square root of (n + lambda) * cov
    double lambda_ = alpha*alpha * (n + kappa) - n;
    Eigen::MatrixXd n_plus_lambda_times_cov = (n + lambda_) * cov;

    Eigen::LLT<Eigen::MatrixXd> lltOfA(n_plus_lambda_times_cov);    // compute the Cholesky decomposition of A
    Eigen::MatrixXd U = lltOfA.matrixU();                           // retrieve factor U  in the decomposition (upper)

    // Calculate sigma points
    sigma_points.row(0) = mean.transpose();
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

/*** Arduino Debugging ***/ 
void MerwedSigmaPoints::debug()
{
    /***
    Debug Merwe Sigma Points to Serial
    ***/    
    Serial.println("------- Debugging Sigma Points -------");
    Serial.print("n: ");
    Serial.println(n);

    Serial.print("num sigma points: ");
    Serial.println(num_sigma_points);

    Serial.print("alpha: ");
    Serial.println(alpha);

    Serial.print("beta: ");
    Serial.println(beta);

    Serial.print("kappa: ");
    Serial.println(kappa);

    Serial.println("Wm: ");
    print_mtxd(Wm.transpose());

    Serial.println("Wc: ");
    print_mtxd(Wc.transpose());
}

/*** ------ Unscented Kalman Filter for Orientation Estimation ------ ***/
/*** Constructors ***/
UKF::UKF()
{
    /***
    Unscented Kalman Filter Constructor
    
    This is a default constructor for the Unscented Kalman Filter for Orientation Estimation
    using Quaternions. The state space is:

    x = [q0 q1 q2 q3 omega_x, omega_y, omega_z].T

    where (q0,q1,q2,q3) are the quaternion elements of the following quaternion 
    q0 + q1*i + q2*j + q3*k. (omega_x, omega_y, omega_z) is the angular velocity.

    The z measurement state space is:

    z = [z_gyro, z_acc, z_mag].T

    where z_gyro is the measurement from the gyro, z_acc is the measurement from the accelerometer
    z_mag is the measurement from the magnetometer. Note that these measurements are in the 
    body frame.
    ***/
    // Initialize x state vector
    x_dim = 7;
    x_hat = Eigen::VectorXd::Zero(x_dim);
    x_hat << 1, 0, 0, 0, 0, 0, 0;   // Initial Quaterion: 1+0i+0j+0k and initial ang vel: [0 0 0].T
 
    // Initialize z state vector
    z_dim = 9;
    z = Eigen::VectorXd::Zero(z_dim); // Initial measurement in frame {B}, [z_gyro, z_acc, z_mag].T

    // Intialize Posteriori Estimate Covariance Matrix
    P = Eigen::MatrixXd::Zero(x_dim, x_dim);

    // Initialize Prior Estimates
    x_prior = x_hat;
    z_prior = z;
    P_prior = P;

    // Initial Posterior Estimates
    x_post = x_hat;
    P_post = P;

    // Initialize Sigma Points
    sigma_points = MerwedSigmaPoints(x_dim);

    sigmas_f = Eigen::MatrixXd::Zero(sigma_points.num_sigma_points, x_dim); // Predicted sigma points
    sigmas_h = Eigen::MatrixXd::Zero(sigma_points.num_sigma_points, z_dim); // Measurement sigma points

    // Initialize noise matrices
    Q = Eigen::MatrixXd::Identity(x_dim, x_dim) * 0.001;    // Process Noise Matrix
    
    R = Eigen::MatrixXd::Identity(z_dim, z_dim) * 0.1;      // Measurement Noise Matrix

    // Intialize inertial frame quantities
    g0 << 0, 0, 1;                              // Gravitational Acceleration Vector
    m0 << B_INTENSITY * cos(INCLINATION),       // Magnetic Field Intensity Vector 
        0.0, 
        B_INTENSITY * sin(INCLINATION); 
}


UKF::UKF(MerwedSigmaPoints merwed_sigma_points)
{
    /***
    Unscented Kalman Filter Constructor with Merwe Sigma Points
    
    This is a default constructor for the Unscented Kalman Filter for Orientation Estimation
    using Quaternions. The constructor takes in Merwe sigma points and assigns it to the UKF.

    The state space is:

    x = [q0 q1 q2 q3 omega_x, omega_y, omega_z].T

    where (q0,q1,q2,q3) are the quaternion elements of the following quaternion 
    q0 + q1*i + q2*j + q3*k. (omega_x, omega_y, omega_z) is the angular velocity.

    The z measurement state space is:

    z = [z_gyro, z_acc, z_mag].T

    where z_gyro is the measurement from the gyro, z_acc is the measurement from the accelerometer
    z_mag is the measurement from the magnetometer. Note that these measurements are in the 
    body frame.
    ***/
    // Initialize x state vector
    x_dim = 7;
    x_hat = Eigen::VectorXd::Zero(x_dim);
    x_hat << 1, 0, 0, 0, 0, 0, 0;   // Initial Quaterion: 1+0i+0j+0k and initial ang vel: [0 0 0].T
 
    // Initialize z state vector
    z_dim = 9;
    z = Eigen::VectorXd::Zero(z_dim); // Initial measurement in frame {B}, [z_gyro, z_acc, z_mag].T

    // Intialize Posteriori Estimate Covariance Matrix
    P = Eigen::MatrixXd::Zero(x_dim, x_dim);

    // Initialize Prior Estimates
    x_prior = x_hat;
    z_prior = z;
    P_prior = P;

    // Initial Posterior Estimates
    x_post = x_hat;
    P_post = P;

    // Assign the sigma points into UKF class
    sigma_points = merwed_sigma_points;

    sigmas_f = Eigen::MatrixXd::Zero(sigma_points.num_sigma_points, x_dim); // Predicted sigma points
    sigmas_h = Eigen::MatrixXd::Zero(sigma_points.num_sigma_points, z_dim); // Measurement sigma points

    // Initialize noise matrices
    Q = Eigen::MatrixXd::Identity(x_dim, x_dim) * 0.001;    // Process Noise Matrix

    //R = Eigen::MatrixXd::Identity(z_dim, z_dim) * 0.1;
    R = Eigen::MatrixXd::Identity(z_dim, z_dim) * 0.5;      // Measurement Noise Matrix

    // Intialize inertial frame quantities
    g0 << 0, 0, 1;                          // Gravitational Acceleration Vector
    m0 << B_INTENSITY * cos(INCLINATION),   // Magnetic Field Intensity Vector
        0.0, 
        B_INTENSITY * sin(INCLINATION);
}

UKF::UKF(int x_dim_, int z_dim_, MerwedSigmaPoints merwed_sigma_points)
{
    /***
    General Unscented Kalman Filter Constructor
    
    This is a general constructor for the Unscented Kalman Filter. This is intended for any 
    state space for any dimension for the state/measurement variables. It also takes in a 
    merwe sigma points to be assigned to UKF.

    Inputs:
    x_dim_: dimension of the estimated state variables
    z_dim_: dimension of the measurement state variables

    ***/
    // Initialize x state vector
    x_dim = x_dim_;
    x_hat = Eigen::VectorXd::Zero(x_dim);

    // Initialize z state vector
    z_dim = z_dim_;
    z = Eigen::VectorXd::Zero(z_dim);

    // Intialize Posteriori Estimate Covariance Matrix
    P = Eigen::MatrixXd::Zero(x_dim, x_dim);

    // Initialize Prior Estimates
    x_prior = x_hat;
    z_prior = z;
    P_prior = P;

    // Initial Posterior Estimates
    x_post = x_hat;
    P_post = P;

    // Initialize Sigma Points
    sigma_points = merwed_sigma_points;

    sigmas_f = Eigen::MatrixXd::Zero(sigma_points.num_sigma_points, x_dim); // Predicted sigma points
    sigmas_h = Eigen::MatrixXd::Zero(sigma_points.num_sigma_points, z_dim); // Measurement sigma points

    // Initialize noise matrices
    Q = Eigen::MatrixXd::Identity(x_dim, x_dim);    // Process Noise Matrix

    R = Eigen::MatrixXd::Identity(z_dim, z_dim);    // Measurement Noise Matrix

}

/*** Destructor ***/
UKF::~UKF()
{
    // Destructor
}

/*** Prediction + Update ***/
// --- Radar example (This is used for testing UKF) ---
void UKF::predict_with_radar_model(double dt)
{
    /***
    Dummy Radar Process Model example used for testing UKF
    ***/
    // Compute the sigma points for given mean and posteriori covariance
    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(x_hat, P);

    // Pass sigmas into f(x)
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        // TODO: Remove Just testing for now
        sigmas_f.row(i) = f_cv_radar(sigmas.row(i), dt);
    }

    // Compute unscented mean and covariance
    std::tie(x_hat, P) = unscented_transform(sigmas_f,
                                        sigma_points.Wm,
                                        sigma_points.Wc,
                                        Q);

    // Save prior
    x_prior = x_hat.replicate(1,1);
    P_prior = P.replicate(1,1);
}


void UKF::update_with_radar_model(Eigen::MatrixXd z_measurement)
{
    /***
    Dummy Radar Measurement Function example used for testing UKF
    ***/    
    // Pass the transformed sigmas into measurement function
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        // TODO: Remove Just testing for now
        sigmas_h.row(i) = h_radar(sigmas_f.row(i));
    }

    // Compute mean and covariance using unscented transform
    Eigen::VectorXd zp;
    Eigen::MatrixXd Pz;

    std::tie(zp, Pz) = unscented_transform(sigmas_h,
                                        sigma_points.Wm,
                                        sigma_points.Wc,
                                        R);

    // Compute cross variance of state and measurements
    Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(x_dim, z_dim);

    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        Eigen::VectorXd x_diff = sigmas_f.row(i) - x_prior.transpose();
        Eigen::VectorXd z_diff = sigmas_h.row(i) - zp.transpose();

        Pxz = Pxz + sigma_points.Wc(i) * x_diff * z_diff.transpose();
    }

    // Compute Kalman Gain
    Eigen::VectorXd y = z_measurement - zp;

    Eigen::MatrixXd Pz_inv = Pz.inverse();
    Eigen::MatrixXd K = Pxz * Pz_inv;

    x_hat = x_prior + K * y;
    P = P_prior - K * Pz * K.transpose();

    // Save posterior
    x_post = x_hat.replicate(1,1);
    P_post = P.replicate(1,1);
}

// --- Quaternion Model with angular velocity biases ---
void UKF::predict_with_quaternion_model(double dt, Eigen::VectorXd u_t)
{
    /***
    Predict with quaternion process model
    u_t: Measured angular velocity as input
    ***/
    // Compute the sigma points for given mean and posteriori covariance
    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(x_hat, P);
    
    // Pass sigmas into f(x)
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        // Predict with quaternion process model for sigma points
        sigmas_f.row(i) = f_quaternion(sigmas.row(i), u_t, dt);

    }


    // Compute unscented mean and covariance
    std::tie(x_hat, P) = unscented_transform(sigmas_f,
                                        sigma_points.Wm,
                                        sigma_points.Wc,
                                        Q);

    // Save prior
    x_prior = x_hat.replicate(1,1);
    P_prior = P.replicate(1,1); 

}

void UKF::update_with_quaternion_model(Eigen::MatrixXd z_measurement)
{
    // Pass the transformed sigmas into measurement function
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        // Update sigmas with measurement model
        sigmas_h.row(i) = h_quaternion(sigmas_f.row(i));
    }

    // Compute mean and covariance using unscented transform
    Eigen::VectorXd zp;
    Eigen::MatrixXd Pz;

    std::tie(zp, Pz) = unscented_transform(sigmas_h,
                                        sigma_points.Wm,
                                        sigma_points.Wc,
                                        R);

    // Compute cross variance of state and measurements
    Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(x_dim, z_dim);

    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        Eigen::VectorXd x_diff = sigmas_f.row(i) - x_prior.transpose();
        Eigen::VectorXd z_diff = sigmas_h.row(i) - zp.transpose();

        Pxz = Pxz + sigma_points.Wc(i) * x_diff * z_diff.transpose();
    }


    // Compute Kalman Gain
    Eigen::VectorXd y = z_measurement - zp;

    Eigen::MatrixXd Pz_inv = Pz.inverse();
    Eigen::MatrixXd K = Pxz * Pz_inv;

    // Update with Kalman Gains
    x_hat = x_prior + K * y;
    P = P_prior - K * Pz * K.transpose();

    // Save posterior
    x_post = x_hat.replicate(1,1);
    P_post = P.replicate(1,1);
       
}

// --- Quaternion Ang Vec Model ---
void UKF::predict_with_quaternion_ang_vec_model(double dt, Eigen::VectorXd u_t)
{
    /***
    Prediction step of UKF with Quaternion + Angular Velocity model i.e state space is:

    x = [q0 q1 q2 q3 omega_x omega_y omega_z].T

    Inputs:
    u_t: Commanded input to prediction step
    dt: Delta time between updates
    ***/
    // Compute the sigma points for given mean and posteriori covariance
    Eigen::MatrixXd sigmas = sigma_points.calculate_sigma_points(x_hat, P);
    
    // Pass sigmas into f(x)
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        // Predict with quaternion process model for sigma points
        sigmas_f.row(i) = f_quaternion_ang_vec_model(sigmas.row(i), dt);

    }


    // Compute unscented mean and covariance
    std::tie(x_hat, P) = unscented_transform(sigmas_f,
                                        sigma_points.Wm,
                                        sigma_points.Wc,
                                        Q);

    // Save prior
    x_prior = x_hat.replicate(1,1);
    P_prior = P.replicate(1,1); 

}

void UKF::update_with_quaternion_ang_vec_model(Eigen::MatrixXd z_measurement)
{
    /***
    Update step of UKF with Quaternion + Angular Velocity model i.e state space is:

    x = [q0 q1 q2 q3 omega_x omega_y omega_z].T
    z = [z_gyro z_acc z_mag].T

    Inputs:
    z_measurement: Sensor measurements from gyroscope, accelerometer and magnetometer
    ***/

    // Pass the transformed sigmas into measurement function
    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        // Update sigmas with measurement model
        sigmas_h.row(i) = h_quaternion_ang_vec_model(sigmas_f.row(i));
    }

    // Compute mean and covariance using unscented transform
    Eigen::VectorXd zp;
    Eigen::MatrixXd Pz;

    std::tie(zp, Pz) = unscented_transform(sigmas_h,
                                        sigma_points.Wm,
                                        sigma_points.Wc,
                                        R);

    // Compute cross variance of state and measurements
    Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(x_dim, z_dim);

    for (int i = 0; i < sigma_points.num_sigma_points; i++)
    {
        Eigen::VectorXd x_diff = sigmas_f.row(i) - x_prior.transpose();
        Eigen::VectorXd z_diff = sigmas_h.row(i) - zp.transpose();

        Pxz = Pxz + sigma_points.Wc(i) * x_diff * z_diff.transpose();
    }

    // Compute Kalman Gain
    Eigen::VectorXd y = z_measurement - zp;

    Eigen::MatrixXd Pz_inv = Pz.inverse();
    Eigen::MatrixXd K = Pxz * Pz_inv;

    // Update with Kalman Gains
    x_hat = x_prior + K * y;
    P = P_prior - K * Pz * K.transpose();

    // Save posterior
    x_post = x_hat.replicate(1,1);
    P_post = P.replicate(1,1);
       
}


// --- Unscented Transform ---
std::tuple<Eigen::VectorXd, Eigen::MatrixXd> UKF::unscented_transform(Eigen::MatrixXd sigmas,
                                                                Eigen::MatrixXd Wm,
                                                                Eigen::MatrixXd Wc,
                                                                Eigen::MatrixXd noise_cov)
{
    /***
    Computes the unscented transform from the sigma points, weighted means and weighted noise covariance.

    Inputs:
    sigmas: Sigma points of UKF
    Wm: Weighted Mean 
    Wc: Weighted Covariance
    noise_cov: Noise covariance matrix

    Outputs:
    mu: Mean from unscented transform computation
    P_cov: Covariance from unscented transform computation
    ***/
    // Compute new mean
    Eigen::VectorXd mu = sigmas.transpose() * Wm;   // Vectorization of sum(wm_i* sigma_i)

    // Compute new covariance matrix
    int kmax = sigmas.rows();
    int n = sigmas.cols();
    Eigen::MatrixXd P_cov = Eigen::MatrixXd::Zero(n,n);

    for (int k = 0; k < kmax; k++)
    {
        Eigen::VectorXd y = sigmas.row(k) - mu.transpose();
        P_cov = P_cov + Wc(k) * y * y.transpose() ;
    }

    // Add noise
    P_cov = P_cov + noise_cov;

    return std::make_tuple(mu, P_cov);
}

/*** Nonlinear Functions ***/
// --- Process & Measurement Model with Quaternion and gyroscope biases ---
// Process Model
Eigen::VectorXd UKF::f_quaternion(Eigen::VectorXd x, Eigen::VectorXd u_t, double dt)
{
    /***
    Nonlinear process model for Orientation estimation with Quaternions 

    Inputs:
    x: current sigma point of state estimate x = [q0 q1 q2 q3 bias_x bias_y bias_z].T
    u_t: Current input to nonlinear process model
    dt: delta time

    Outputs:
    predicted_sigmas: sigma points after being propagated through nonlinear process model
    ***/ 
    // Extract quaternion from current state estimates (q0,q1,q2,q3)
    UnitQuaternion attitude(x(0), 
                            x(1),
                            x(2),
                            x(3));


    // Estimated attitude update with incremental rotation update
    // EQN 3.26 & EQN 3.17 (Exponential with skew matrix and delta_t)
    UnitQuaternion uq_omega = UnitQuaternion::omega(u_t(0)*dt, 
                                                u_t(1)*dt, 
                                                u_t(2)*dt);
    attitude = attitude * uq_omega;

    // Update the state space on sigmas
    Eigen::VectorXd predicted_sigma(7);


    // Quaternions
    predicted_sigma(0) = attitude.s;
    predicted_sigma(1) = attitude.v_1;
    predicted_sigma(2) = attitude.v_2;
    predicted_sigma(3) = attitude.v_3;
    
    // Biases
    predicted_sigma(4) = x(4);
    predicted_sigma(5) = x(5);
    predicted_sigma(6) = x(6);

    return predicted_sigma;
}

// Measurement Model
Eigen::VectorXd UKF::h_quaternion(Eigen::VectorXd x)
{
    /***
    Nonlinear measurement model for Orientation estimation with Quaternions

    Inputs:
    x: current sigma point of state estimate x = [q0 q1 q2 q3 bias_x bias_y bias_z].T

    Outputs:
    z_pred_sigma: sigma point after being propagated through nonlinear measurement model
    ***/ 
    // --- Measurement model ---
    // Extract quaternion from current state estimates (q0,q1,q2,q3)
    UnitQuaternion attitude(x(0), 
                            x(1),
                            x(2),
                            x(3));

    // Inverse: {B} to {0}
    UnitQuaternion invq = attitude.inverse();

    // Accelerometer
    Eigen::VectorXd acc_pred = invq.vector_rotation_by_quaternion(g0);

    // Magnetomer
    Eigen::VectorXd mag_pred = invq.vector_rotation_by_quaternion(m0);

    // Z prediction
    Eigen::VectorXd z_pred_sigma(acc_pred.size() + mag_pred.size());
    z_pred_sigma << acc_pred, mag_pred; 

    return z_pred_sigma;
}


// --- Process & Measurement Model with Quaternion and angular velocity ---
// Process Model
Eigen::VectorXd UKF::f_quaternion_ang_vec_model(Eigen::VectorXd x, double dt)
{
    /***
    Nonlinear process model for Orientation estimation with Quaternions 

    Inputs:
    x: current sigma point of state estimate x = [q0 q1 q2 q3 omega_x omega_y omega_z].T
    u_t: Current input to nonlinear process model
    dt: delta time

    Outputs:
    predicted_sigmas: sigma points after being propagated through nonlinear process model
    ***/ 
    // State space x = [q0 q1 q2 q3 omega_x, omega_y, omega_z].T
    // Extract quaternion from current state estimates
    UnitQuaternion attitude(x(0), 
                            x(1),
                            x(2),
                            x(3));


    // Estimated attitude update with incremental rotation update
    // EQN 3.26 & EQN 3.17 (Exponential with skew matrix and delta_t)
    UnitQuaternion uq_omega = UnitQuaternion::omega(x(4)*dt, 
                                                x(5)*dt, 
                                                x(6)*dt);
    attitude = attitude * uq_omega;

    // Update the state space on sigmas
    Eigen::VectorXd predicted_sigma(7);


    // Quaternions
    predicted_sigma(0) = attitude.s;
    predicted_sigma(1) = attitude.v_1;
    predicted_sigma(2) = attitude.v_2;
    predicted_sigma(3) = attitude.v_3;
    
    // Angular velocity
    predicted_sigma(4) = x(4);
    predicted_sigma(5) = x(5);
    predicted_sigma(6) = x(6);

    return predicted_sigma;
}

// Measurement Model
Eigen::VectorXd UKF::h_quaternion_ang_vec_model(Eigen::VectorXd x)
{
    /***
    Nonlinear measurement model for Orientation estimation with Quaternions

    Inputs:
    x: current sigma point of state estimate x = [q0 q1 q2 q3 omega_x omega_y omega_z].T

    Outputs:
    z_pred_sigma: sigma point after being propagated through nonlinear measurement model
    ***/ 
    // --- Measurement model ---
    // Extract quaternion from current state estimates
    UnitQuaternion attitude(x(0), 
                            x(1),
                            x(2),
                            x(3));

    // Inverse: {B} to {0}
    UnitQuaternion invq = attitude.inverse();

    // Accelerometer
    Eigen::VectorXd acc_pred = invq.vector_rotation_by_quaternion(g0);

    // Magnetomer
    Eigen::VectorXd mag_pred = invq.vector_rotation_by_quaternion(m0);


    // Gyroscope
    Eigen::VectorXd gyro_pred(3);
    gyro_pred << x(4), x(5), x(6);

    // Z prediction
    Eigen::VectorXd z_pred_sigma(gyro_pred.size() + acc_pred.size() + mag_pred.size());
    z_pred_sigma << gyro_pred, acc_pred, mag_pred; 

    return z_pred_sigma;
}

// --- Radar example (Used for testing) ---
Eigen::VectorXd UKF::f_cv_radar(Eigen::VectorXd x, double dt)
{
    /***
    Nonlinear process model for Radar Example
    (Note: This is used for testing UKF when coding it!) 
    ***/ 

    Eigen::MatrixXd F_mat(4,4);
    F_mat << 1, dt, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, dt,
        0, 0, 0, 1;

    return F_mat * x;

}

Eigen::VectorXd UKF::h_radar(Eigen::VectorXd x)
{
    /***
    Nonlinear measurement model for Radar Example
    (Note: This is used for testing UKF when coding it!) 
    ***/     
    double dx = x(0) - 0;
    double dy = x(2) - 0;

    double slant_range = sqrt(dx*dx + dy*dy);
    double elevation_angle = atan2(dy, dx);

    Eigen::VectorXd res(2);
    res << slant_range, elevation_angle;

    return res;
} 

/*** Arduino Debugging ***/
void UKF::debug()
{
    /***
    Debugging Unscented Kalman Filter to Arduino Serial
    ***/
    Serial.println("------- Debugging UKF -------");
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