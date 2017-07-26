#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // state dimension
  n_x_ = 5;

  // augmented state dimension
  n_aug_ = 7;

  // lidar measurement dimension
  n_lidar_ = 2;

  // radar measurement dimension
  n_radar_ = 3;

  // sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(1 / (2 * (lambda_ + n_aug_)));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  // setup measurement covariance matrix
  R_radar_ = MatrixXd(n_radar_, n_radar_);
  R_radar_.fill(0.0);
  R_radar_(0,0) = std_radr_ * std_radr_;
  R_radar_(1,1) = std_radphi_ * std_radphi_;
  R_radar_(2,2) = std_radrd_ * std_radrd_;

  R_laser_ = MatrixXd(n_lidar_, n_lidar_);
  R_laser_.fill(0.0);
  R_laser_(0,0) = std_laspx_ * std_laspx_;
  R_laser_(1,1) = std_laspy_ * std_laspy_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  //initialize state vector (x_) and covariance matrix (P_)
  if (!is_initialized_) {
    time_us_ = meas_package.timestamp_;
    x_.fill(0.0);
    P_ = MatrixXd::Identity(n_x_, n_x_);

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      //convert radar from polar to cartesian coordinates to init state vector
      double range = meas_package.raw_measurements_[0];
      double angle = meas_package.raw_measurements_[1];
      double px = range * cos(angle);
      double py = range * sin(angle);
      x_ << px, py, 0, 0, 0;
      is_initialized_ = true;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      //extract lidar cartesian measurements to initialize state vector
      double px = meas_package.raw_measurements_[0];
      double py = meas_package.raw_measurements_[1];

      if (fabs(px) < 0.001 && fabs(py) < 0.001){
         px = 0.001;
         py = 0.001;
      }
      x_ << px, py, 0, 0, 0;
      is_initialized_ = true;
    }
  }

  //process measurement case object is already initialized
  else {
    //compute the time elapsed between the current and previous measurements
    //dt - expressed in seconds
  	float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  	time_us_ = meas_package.timestamp_;

    //skip prediction step case measurements are received in close timestamps
    if (dt > 0.001) {
      Prediction(dt);
    }

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      UpdateRadar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      UpdateLidar(meas_package);
    }
  }
  return;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  //augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  AugmentedSigmaPoints(Xsig_aug);

  //predict sigma points (Xsig_pred_)
  SigmaPointPrediction(Xsig_aug, delta_t);

  //predict state mean (x_) and state covariance matrix (P_)
  PredictMeanAndCovariance();

  return;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  MatrixXd Zsig   = MatrixXd(n_lidar_, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_lidar_);
  MatrixXd S      = MatrixXd(n_lidar_, n_lidar_);

  std::cout << std::endl;
  std::cout << "UpdateLidar" << std::endl;

  PredictLaserMeasurement(Zsig, z_pred, S);
  UpdateState(Zsig, z_pred, S, meas_package.raw_measurements_, n_lidar_);

  return;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  MatrixXd Zsig   = MatrixXd(n_radar_, 2 * n_aug_ + 1);
  VectorXd z_pred = VectorXd(n_radar_);
  MatrixXd S      = MatrixXd(n_radar_, n_radar_);

  std::cout << std::endl;
  std::cout << "UpdateRadar" << std::endl;

  PredictRadarMeasurement(Zsig, z_pred, S);
  UpdateState(Zsig, z_pred, S, meas_package.raw_measurements_, n_radar_);

  return;
}

/**
 * Creates Augmented Sigma Points Matrix based on augmented state, lambda and
 * covariance matrix P and process noise Q.
 * @param {MatrixXd} Xsig_aug The output augmented sigma points.
 */
void UKF::AugmentedSigmaPoints(MatrixXd& Xsig_aug) {

  //create local augmented mean state vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create local augmented state process covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //fill augmented mean state vector
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //fill augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;

  //append covariance matrix Q
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  //square root matrix from augmented covariance matrix
  MatrixXd L = P_aug.llt().matrixL();

  //process output - augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  //print result
  std::cout << std::endl;
  std::cout << "AugmentedSigmaPoints" << std::endl;
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  return;
}

/**
 * Updates global Sigma Points matrix (Xsig_pred_) based on elapsed time since
 * last update
 * @param {MatrixXd} Xsig_aug The input matrix with augmented sigma points.
 * @param {double} delta_t The time elapsed since last measurement.
 */
void UKF::SigmaPointPrediction(const MatrixXd& Xsig_aug, const double delta_t) {

  for (int i=0; i< 2 * n_aug_ + 1; i++) {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin(yaw + yawd*delta_t) - sin(yaw) );
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + ( v * delta_t * cos(yaw) );
        py_p = p_y + ( v * delta_t * sin(yaw) );
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }

  //print result
  std::cout << std::endl;
  std::cout << "SigmaPointPrediction" << std::endl;
  std::cout << "Xsig_pred = " << std::endl << Xsig_pred_ << std::endl;

  return;
}

/**
 * Predicts Mean state (x_) And Covariance matrix (P_) based on
 * Sigma Points matrix (Xsig_pred_)
 */
void UKF::PredictMeanAndCovariance() {

  //predict state mean
  x_.fill(0.0);
  for (int i=0; i < 2 * n_aug_ + 1; i++) {
    x_ = x_ + ( weights_(i) * Xsig_pred_.col(i) );
  }

  //predict state covariance matrix
  P_.fill(0.0);
  for (int i=0; i < 2 * n_aug_ + 1; i++) {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    //angle normalization
    x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));

    P_ = P_ + ( weights_(i) * x_diff * x_diff.transpose() );
  }

  //print result
  std::cout << std::endl;
  std::cout << "PredictMeanAndCovariance" << std::endl;
  std::cout << "Predicted state - x" << std::endl;
  std::cout << x_ << std::endl;
  std::cout << "Predicted covariance matrix - P" << std::endl;
  std::cout << P_ << std::endl;

  return;
}

/**
 * Predicts Lidar Measurement Mean (z_pred) and Covariance (S) by means of
 * predicted Sigma Points (Xsig_pred_), Mean (x_) and Covariance (P_)
 * @param {MatrixXd} Zsig The predicted sigma points in measurement space
 * @param {VectorXd} z_pred The predicted radar measurement mean vector
 * @param {MatrixXd} S The predicted radar measurement covariance matrix
 */
void UKF::PredictLaserMeasurement(MatrixXd& Zsig, VectorXd& z_pred,
                                  MatrixXd& S) {

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);

    // measurement model
    Zsig(0,i) = p_x;
    Zsig(1,i) = p_y;
  }

  //mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + R_laser_;

  //print result
  std::cout << std::endl;
  std::cout << "PredictLaserMeasurement" << std::endl;
  std::cout << "Predicted Laser Measurement - z_pred " << std::endl;
  std::cout << z_pred << std::endl;
  std::cout << "Predicted Measurement covariance - S " << std::endl;
  std::cout << S << std::endl;

  return;
}

/**
 * Predicts Radar Measurement Mean (z_pred) and Covariance (S) by means of
 * predicted Sigma Points (Xsig_pred_), Mean (x_) and Covariance (P_)
 * @param {MatrixXd} Zsig The predicted sigma points in measurement space
 * @param {VectorXd} z_pred The predicted radar measurement mean vector
 * @param {MatrixXd} S The predicted radar measurement covariance matrix
 */
void UKF::PredictRadarMeasurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S) {

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v  = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
	//yawdot is not required for the transformation into radar measurement space

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    if (fabs(p_x) < 0.001 && fabs(p_y) < 0.001){
       p_x = 0.001;
       p_y = 0.001;
    }

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //measurement covariance matrix S
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
	z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));
    //update predicted state measurement covariance
    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  S = S + R_radar_;

  //print result
  std::cout << std::endl;
  std::cout << "PredictRadarMeasurement" << std::endl;
  std::cout << "Predicted Radar measurement - z_pred " << std::endl;
  std::cout << z_pred << std::endl;
  std::cout << "Predicted Measurement covariance - S " << std::endl;
  std::cout << S << std::endl;

  return;
}

/**
 * Updates state (x_) and covariance matrix (P_) based on sensor measurement (z)
 * @param {MatrixXd} Zsig The sigma points in the measurement space
 * @param {VectorXd} z_pred The predicted sensor measurement mean vector
 * @param {MatrixXd} S The predicted sensor measurement covariance matrix
 * @param {VectorXd} z The vector with incoming measurement
 * @param int n_z The measurement dimension
 */
void UKF::UpdateState(const MatrixXd& Zsig, const VectorXd& z_pred,
                      const MatrixXd& S, const VectorXd& z, const int n_z) {

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  //print result
  std::cout << std::endl;
  std::cout << "UpdateState" << std::endl;
  std::cout << "Updated state - x_: " << std::endl;
  std::cout << x_ << std::endl;
  std::cout << "Updated state covariance - P_: " << std::endl;
  std::cout << P_ << std::endl;

  return;
}
