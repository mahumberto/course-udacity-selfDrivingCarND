#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "tools.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* laser measurement noise covariance matrix
  MatrixXd R_laser_;

  ///* radar measurement noise covariance matrix
  MatrixXd R_radar_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Lidar measurement dimension
  int n_lidar_;

  ///* Radar measurement dimension
  int n_radar_;

  ///* Sigma point spreading parameter
  double lambda_;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

private:

  /**
   * Creates Augmented Sigma Points Matrix based on augmented state, lambda and
   * covariance matrix (P_) and process noise (Q_)
   * @param {MatrixXd} Xsig_aug The augmented sigma points
   */
  void AugmentedSigmaPoints(MatrixXd& Xsig_aug);

  /**
   * Predicts Sigma Points matrix (Xsig_pred_) based on elapsed time since
   * last update
   * @param {MatrixXd} Xsig_aug The input matrix with augmented sigma points
   * @param {double} delta_t The time elapsed since last measurement
   */
  void SigmaPointPrediction(const MatrixXd& Xsig_aug, const double delta_t);

  /**
   * Predicts Mean state (x_) And Covariance matrix (P_) based on predicted
   * Sigma Points matrix (Xsig_pred_)
   */
  void PredictMeanAndCovariance();

  /**
   * Predicts Radar Measurement Mean (z_pred) and Covariance (S) by means of
   * predicted Sigma Points (Xsig_pred_), Mean (x_) and Covariance (P_)
   * @param {MatrixXd} Zsig The predicted sigma points in measurement space
   * @param {VectorXd} z_pred The predicted radar measurement mean vector
   * @param {MatrixXd} S The predicted radar measurement covariance matrix
   */
  void PredictLaserMeasurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S);

  /**
   * Predicts Radar Measurement Mean (z_pred) and Covariance (S) by means of
   * predicted Sigma Points (Xsig_pred_), Mean (x_) and Covariance (P_)
   * @param {MatrixXd} Zsig The predicted sigma points in measurement space
   * @param {VectorXd} z_pred The predicted radar measurement mean vector
   * @param {MatrixXd} S The predicted radar measurement covariance matrix
   */
  void PredictRadarMeasurement(MatrixXd& Zsig, VectorXd& z_pred, MatrixXd& S);

  /**
   * Updates state (x_) and covariance matrix (P_) based on sensor measurement (z)
   * @param {MatrixXd} Zsig The sigma points in the measurement space
   * @param {VectorXd} z_pred The predicted sensor measurement mean vector
   * @param {MatrixXd} S The predicted sensor measurement covariance matrix
   * @param {VectorXd} z The vector with incoming measurement
   * @param int n_z The measurement dimension
   */
  void UpdateState(const MatrixXd& Zsig, const VectorXd& z_pred,
                   const MatrixXd& S, const VectorXd& z, const int n_z);

};

#endif /* UKF_H */
