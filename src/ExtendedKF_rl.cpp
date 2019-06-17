#include "ExtendedKF_rl.h"
#include <iostream>

using std::cout;
using std::endl;

ExtendedKF_rl::ExtendedKF_rl() {}

ExtendedKF_rl::~ExtendedKF_rl() {}

ExtendedKF_rl::ExtendedKF_rl(int X_length, MatrixXd P_in, MatrixXd F_in, MatrixXd Q_in, 
                             MatrixXd H_laser_in, MatrixXd R_laser_in, 
                             MatrixXd H_radar_in, MatrixXd R_radar_in) {
  x_ = VectorXd(X_length);
  P_ = P_in;
  F_ = F_in;
  Q_ = Q_in;
  H_ = H_laser_in;
  R_ = R_laser_in;
  H_radar_ = H_radar_in;
  R_radar_ = R_radar_in;
}

void ExtendedKF_rl::UpdateEKF(const VectorXd &z) {
  /**
   * Update the state by using Extended Kalman Filter equations
   */

  // It gets the position measurement predicted before and converts it to polar coordinated in order to be able to compare it against the real radar measurement
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float px2py2_sqrt = sqrt(pow(px,2) + pow(py,2));
  float pxpy_atan = atan2(py,px);

  VectorXd z_pred(3);
  z_pred << px2py2_sqrt, 
            pxpy_atan,
            (px*vx + py*vy)/px2py2_sqrt;

  // It gets the difference between the transformed predicted measurement and the real measurement from the radar
  VectorXd y = z - z_pred; // (3x1) + (3x1) = (3x1)

  // It calculates the Kalman gain (K matrix) using the state covariance matrix (P) and the observation matrix (H)
  // which in this case corresponds to the Jacobian of the h function used to convert the predicted measurement from
  // cartesian coordinates to polar coordinates.
  MatrixXd S = H_radar_*P_*H_radar_.transpose() + R_radar_; // (3x4)x(4x4)x(4x3) + (3x3) = (3x3)
  MatrixXd K = P_*H_radar_.transpose()*S.inverse(); // (4x4)x(4x3)x(3x3) = (4x3)

  // It updates the measurements of position and speed (x) with the difference between the predicted positions and real positions (y) and the 
  // Kalman gain (K)
  x_ = x_ + K*y;  // (4x1) + (4x3)x(3x1) = (4x1)
}

void ExtendedKF_rl::UpdateH_radar() {
  /**
  * It calculates the Jacobian Matrix to be used as H matrix (observation matrix) for radar observations of a Kalman Filter
  */
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  float px2py2 = pow(px,2) + pow(py,2);
  float px2py2_sqrt = sqrt(px2py2);
  float px2py2_32 = pow(px2py2, 1.5);

  MatrixXd Hj(3,4);

  Hj << px/px2py2_sqrt, py/px2py2_sqrt, 0, 0,
        -py/px2py2, -px/px2py2, 0, 0,
        py*(vx*py - vy*px)/px2py2_32, px*(vy*px - vx*py)/px2py2_32, px/px2py2_sqrt, py/px2py2_sqrt;

  H_radar_ = Hj;
}


void ExtendedKF_rl::printAllVariables() {
  cout << "x (state vector): " << endl << x_ << endl << endl;
  cout << "P (state covariance matrix): " << endl << P_ << endl << endl;
  cout << "F (state transition matrix): " << endl << F_ << endl << endl;
  cout << "Q (process covariance matrix): " << endl << Q_ << endl << endl;
  cout << "R_laser (measurement covariance matrix for laser): " << endl << R_ << endl << endl;
  cout << "R_radar (measurement covariance matrix for radar): " << endl << R_radar_ << endl << endl;
  cout << "H_laser (observation matrix for laser) " << endl << H_ << endl << endl;
  cout << "H_radar (observation matrix for radar) " << endl << H_radar_ << endl << endl;
  
}