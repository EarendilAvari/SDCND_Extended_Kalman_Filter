#include "kalman_filter.h"
#include <iostream>

using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

KalmanFilter::KalmanFilter(int X_length, MatrixXd P_in, MatrixXd F_in, MatrixXd Q_in, 
                           MatrixXd H_laser_in, MatrixXd R_laser_in, 
                           MatrixXd H_radar_in, MatrixXd R_radar_in) {
  x_ = VectorXd(X_length);
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
  H_laser_ = H_laser_in;
  R_laser_ = R_laser_in;
  H_radar_ = H_radar_in;
  R_radar_ = R_radar_in;
}

void KalmanFilter::InitX(VectorXd x_init) {
  x_ = x_init;
}

void KalmanFilter::Predict() {
  /**
   * predict the next state
   */
   x_ = F_*x_;
   P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}

VectorXd KalmanFilter::getX() {
  return x_;
}

float KalmanFilter::getX_n(int n){
  return static_cast<float>(x_(n));
}

MatrixXd KalmanFilter::getP() {
  return P_;
}

void KalmanFilter::updateF(MatrixXd newF) {
  F_ = newF;
}

void KalmanFilter::updateQ(MatrixXd newQ) {
  Q_ = newQ;
}

void KalmanFilter::printAllVariables() {
  cout << "x (state vector): " << x_ << endl;
  cout << "P (state covariance matrix): " << P_ << endl;
  cout << "F (state transition matrix): " << F_ << endl;
  cout << "Q (process covariance matrix): " << Q_ << endl;
  cout << "R_laser (measurement covariance matrix for laser): " << R_laser_ << endl;
  cout << "R_radar (measurement covariance matrix for radar): " << R_radar_ << endl;
  cout << "H_laser (observation matrix for laser) " << H_laser_ << endl;
  cout << "H_radar (observation matrix for radar) " << H_radar_ << endl;
  
}


