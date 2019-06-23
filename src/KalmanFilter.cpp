#include "KalmanFilter.h"
#include <iostream>

using std::cout;
using std::endl;

/* 
 * Please note that the Eigen library does not initialize 
 * VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

KalmanFilter::KalmanFilter(int X_length, MatrixXd P_in, MatrixXd F_in, MatrixXd Q_in, 
                           MatrixXd H_in, MatrixXd R_in) {
  x_ = VectorXd(X_length);
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
  H_ = H_in;
  R_ = R_in;
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
  // It gets the position measurement predicted before and compares it agaist the real measurements from the sensor
  VectorXd z_pred = H_*x_; // (2x4)x(4x1) = 2x1
  VectorXd y = z - z_pred; // (2x1)

  // It calculates the Kalman gain (K matrix) using the state covariance matrix (P) and the observation matrix (H)
  MatrixXd S = H_*P_*H_.transpose() + R_; // (2x4)x(4x4)x(4x2) + (2x2) = (2x2)
  MatrixXd K = P_*H_.transpose()*S.inverse(); // (4x4)x(4x2)x(2x2) = (4x2)

  // It updates the measurements of position and speed (x) with the difference between the predicted positions and real positions (y) and the 
  // Kalman gain (K)
  x_ = x_ + K*y; // (4x1) + (4x2)*(2x1) = (4x1)

  // It updates the P matrix
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K*H_)*P_;
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
  cout << "R_laser (measurement covariance matrix for laser): " << R_ << endl;
  cout << "H_laser (observation matrix for laser) " << H_ << endl;
  
}


