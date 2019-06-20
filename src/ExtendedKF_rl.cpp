#include "ExtendedKF_rl.h"
#include <iostream>
#include <cmath>

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
  // cout << px2py2_sqrt << endl << endl;
  if (px2py2_sqrt == 0) {
    return;
  }


  float pxpy_atan;
  if (px == 0){
    pxpy_atan = 0;
  }
  else {
    pxpy_atan = atan2(py,px);
  }
  // cout << pxpy_atan << endl << endl;

  VectorXd z_pred(3);
  z_pred << px2py2_sqrt, 
            pxpy_atan,
            (px*vx + py*vy)/px2py2_sqrt;

  // cout << z_pred << endl << endl;

  // It gets the difference between the transformed predicted measurement and the real measurement from the radar
  VectorXd y = z - z_pred; // (3x1) + (3x1) = (3x1)

  // It corrects the phi value on the y vector if it is bigger than pi or smalled than -pi.
  float corrected_phi;
  if (y[1] > M_PI) {
    corrected_phi = y[1] - 2.0*M_PI;
  } 
  else if (y[1] < -M_PI) {
    corrected_phi = y[1] + 2.0*M_PI; 
  }
  else {
    corrected_phi = y[1];
  }

  // It makes other vector with the corrected phi and the other two values.
  VectorXd y_corr(3);
  y_corr << y[0], corrected_phi, y[2];
  
  // cout << y << endl << endl;

  // It calculates the Kalman gain (K matrix) using the state covariance matrix (P) and the observation matrix (H)
  // which in this case corresponds to the Jacobian of the h function used to convert the predicted measurement from
  // cartesian coordinates to polar coordinates.
  // cout << H_radar_ << endl << endl;
  // cout << P_ << endl << endl;
  // cout << H_radar_.transpose() << endl << endl;
  // cout << R_radar_ << endl << endl;
  MatrixXd S = H_radar_*P_*H_radar_.transpose() + R_radar_; // (3x4)x(4x4)x(4x3) + (3x3) = (3x3)
  MatrixXd K = P_*H_radar_.transpose()*S.inverse(); // (4x4)x(4x3)x(3x3) = (4x3)
  // cout << S << endl << endl;
  // cout << K << endl << endl;

  // It updates the measurements of position and speed (x) with the difference between the predicted positions and real positions (y) and the 
  // Kalman gain (K)
  x_ = x_ + K*y_corr;  // (4x1) + (4x3)x(3x1) = (4x1)
  // cout << x_ << endl << endl;

  // It updates the P matrix
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K*H_radar_)*P_;
  // cout << P_ << endl << endl;
}

void ExtendedKF_rl::UpdateH_radar() {
  /**
  * It calculates the Jacobian Matrix to be used as H matrix (observation matrix) for radar observations of a Kalman Filter
  */
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  MatrixXd Hj(3,4);

  float px2py2 = pow(px,2) + pow(py,2);
  // cout << px2py2 << endl << endl;

  if (px2py2 == 0) {
    Hj << 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0;
  }
  else {
    float px2py2_sqrt = sqrt(px2py2);
    // cout << px2py2_sqrt << endl << endl;
    float px2py2_32 = px2py2*px2py2_sqrt; // Exponents = 1 + 1/2 = 3/2. This is faster than using pow again.
    // cout << px2py2_32 << endl << endl;

    Hj << px/px2py2_sqrt, py/px2py2_sqrt, 0.0, 0.0,
          -py/px2py2, px/px2py2, 0.0, 0.0,
          py*(vx*py - vy*px)/px2py2_32, px*(vy*px - vx*py)/px2py2_32, px/px2py2_sqrt, py/px2py2_sqrt;
    // cout << Hj << endl << endl;
  }
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