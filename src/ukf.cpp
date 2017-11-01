#include "ukf.h"
#include "Eigen/Dense"
#include "tools.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//////////////////////////////////////////////////////////////////////////////////////
//
// INTERFACE FUNCTIONS
//

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // state dimension
  n_x_ = 5;

  // augmented state dimension
  n_aug_ = 7;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2;
 
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;

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

  // sigma point Lambda
  lambda_ = 3 - n_x_;

  // initialize sigma point weights
  InitWeights();

  // predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // initial NIS values
  NIS_laser_ = 0.0;
  NIS_radar_ = 0.0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  ///////////////////////////////////////////////////////////////////////////////
  //
  // initialization
  //

  if (!is_initialized_) {
    P_ = MatrixXd::Identity(P_.rows(), P_.cols());

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      // convert radar from polar to cartesian coordinates and initialize state.
      auto rho = meas_package.raw_measurements_[0];
      auto phi = meas_package.raw_measurements_[1];
      x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
    }

    time_us_ = meas_package.timestamp_;

    is_initialized_ = true;
    return;                                     // exit !!!
  }

  ///////////////////////////////////////////////////////////////////////////////
  //
  // prediction
  //

  float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);

  ///////////////////////////////////////////////////////////////////////////////
  //
  // measurement update
  //

  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  } else {
    std::cerr << "Unknown sensor type (" << meas_package.sensor_type_ << ")" << std::endl;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  // generate sigma points
  MatrixXd Xsig = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  GenerateAugmentedSigmaPoints(Xsig);

  // predict sigma points
  PredictSigmaPoints(Xsig, delta_t, Xsig_pred_);

  // predict state and state covariance
  PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

  if (!use_laser_)
    return;

  // measurement matrix laser
  auto H = MatrixXd(2, 5);
  H << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0;
  auto Ht = H.transpose();

  // measurement covariance matrix laser
  auto R = MatrixXd(2, 2);
  R << std_laspx_ * std_laspx_, 0,
       0, std_laspy_ * std_laspy_;

  VectorXd z_diff = meas_package.raw_measurements_ - H * x_;
  MatrixXd S = H * P_ * Ht + R;
  MatrixXd K = P_ * Ht * S.inverse();

  // new state
  x_ = x_ + (K * z_diff);
  P_ -= K * H * P_;

  // calculate NIS
  NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {

  if (!use_radar_)
    return;

  auto n_z = 3;
  auto z_pred = VectorXd(n_z);
  auto S = MatrixXd(n_z, n_z);
  auto Zsig = MatrixXd(n_z, Xsig_pred_.cols());
  auto Tc = MatrixXd(n_x_, n_z);

  PredictRadarMeasurement(Zsig, z_pred, S);
  CrossCorrelationMatrix(Zsig, z_pred, Tc);

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;
  z_diff(1) = Tools::NormalizeAngle(z_diff(1));

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // calculate NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}

//////////////////////////////////////////////////////////////////////////////////////
//
// HELPER FUNCTIONS
//

void UKF::InitWeights() {
  weights_ = VectorXd(2 * n_aug_ + 1);

  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i=1;i<weights_.rows();++i) {
    weights_(i) = 1 / (2 * (lambda_ + n_aug_));
  }
}

void UKF::GenerateSigmaPoints(MatrixXd &Xsig) {

  MatrixXd A = P_.llt().matrixL();

  MatrixXd offset = sqrt(lambda_ + n_x_) * A;
  
  Xsig.col(0) = x_;
  
  for (int i=0; i < n_x_;++i) {
    Xsig.col(1+i) = x_ + offset.col(i);
    Xsig.col(6+i) = x_ - offset.col(i);
  }
}

void UKF::GenerateAugmentedSigmaPoints(MatrixXd &Xsig_aug) {
  // create augmented mean state
  auto x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(n_x_) = 0.0;
  x_aug(n_x_ + 1) = 0.0;

  // create augmented covariance matrix
  auto Q = MatrixXd(2,2);
  Q << std_a_ * std_a_, 0,
       0, std_yawdd_ * std_yawdd_;
      
  auto P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(2, 2) = Q;

  // create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  // create augmented sigma points
  MatrixXd offset = sqrt(lambda_ + n_aug_) * A;

  Xsig_aug.col(0) = x_aug;

  for (int i=0; i < n_aug_;++i) {
    Xsig_aug.col(1+i) = x_aug + offset.col(i);
    Xsig_aug.col(8+i) = x_aug - offset.col(i);
  }
}

void UKF::PredictSigmaPoints(const MatrixXd &Xsig_aug, double delta_t, MatrixXd &Xsig_pred) {

  for (int i=0; i < Xsig_aug.cols();++i) {
    auto px   = Xsig_aug(0, i);
    auto py   = Xsig_aug(1, i);
    auto v    = Xsig_aug(2, i);
    auto yaw  = Xsig_aug(3, i); 
    auto yawd = Xsig_aug(4, i);
    auto na   = Xsig_aug(5, i);
    auto nyaw = Xsig_aug(6, i);
    
    VectorXd v1 = VectorXd(n_x_);
    VectorXd v2 = VectorXd(n_x_);
    
    if (fabs(yawd) > 0.0001) {
      v1 << (v / yawd) * (sin(yaw + yawd * delta_t) - sin(yaw)),
            (v / yawd) * (-cos(yaw + yawd * delta_t) + cos(yaw)),
            0, 
            yawd * delta_t,
            0;
    } else {
      v1 << v * cos(yaw) * delta_t,
            v * sin(yaw) * delta_t,
            0,
            yawd * delta_t,
            0;
    }
    
    v2 << 0.5 * delta_t * delta_t * cos(yaw) * na,
          0.5 * delta_t * delta_t * sin(yaw) * na,
          delta_t * na,
          0.5 * delta_t * delta_t * nyaw,
          delta_t * nyaw;
          
    Xsig_pred.col(i) = Xsig_aug.col(i).head(n_x_) + v1 + v2;
  }
}

void UKF::PredictMeanAndCovariance() {
   
   // predict state mean
   x_ = Xsig_pred_ * weights_;
   
   // predict state covariance matrix
   P_ = MatrixXd::Zero(n_x_, n_x_);
   
   for (int i=0;i<Xsig_pred_.cols();++i) {
     VectorXd v = Xsig_pred_.col(i) - x_;
     v(3) = Tools::NormalizeAngle(v(3));
     P_ += weights_(i) * (v * v.transpose());
   }
}

void UKF::PredictRadarMeasurement(MatrixXd &Zsig, VectorXd &z_pred, MatrixXd &S) {

  const int n_z = 3;

  // transform sigma points into measurement space
  for (int i=0; i < Xsig_pred_.cols(); ++i) {
    double px   = Xsig_pred_(0, i);
    double py   = Xsig_pred_(1, i);
    double v    = Xsig_pred_(2, i);
    double yaw  = Xsig_pred_(3, i);
    
    Zsig(0, i) = sqrt(px * px + py * py);
    Zsig(1, i) = atan2(py, px);
    Zsig(2, i) = (px * cos(yaw) * v + py * sin(yaw) * v) / Zsig(0, i);
  }

  // calculate mean predicted measurement
  z_pred = Zsig * weights_;

  // calculate measurement covariance matrix S
  S = MatrixXd::Zero(n_z, n_z);

  for (int i=0;i<Zsig.cols();++i) {
    VectorXd v = Zsig.col(i) - z_pred;
    v(1) = Tools::NormalizeAngle(v(1));
    S += weights_(i) * (v * v.transpose()) ;
  }

  // add measurement noise
  S(0,0) += std_radr_ * std_radr_;
  S(1,1) += std_radphi_ * std_radphi_;
  S(2,2) += std_radrd_ * std_radrd_;
}

void UKF::CrossCorrelationMatrix(const MatrixXd &Zsig, const VectorXd &z_pred, MatrixXd &Tc) {
  Tc.fill(0.0);

  for (int i = 0; i < Xsig_pred_.cols(); i++) {  
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    z_diff(1) = Tools::NormalizeAngle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = Tools::NormalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }
}