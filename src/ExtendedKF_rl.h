#ifndef EXTENDEDKF_RL
#define EXTENDEDKF_RL

#include "Eigen/Dense"
#include "kalman_filter.h"

class ExtendedKF_rl: public KalmanFilter {
  public: 

  ExtendedKF_rl();
  ExtendedKF_rl(int X_length, MatrixXd P_in, MatrixXd F_in, MatrixXd Q_in, 
                MatrixXd H_laser_in, MatrixXd R_laser_in, 
                MatrixXd H_radar_in, MatrixXd R_radar_in);

  virtual ~ExtendedKF_rl();

    /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  /**
   * Updates the H matrix used for radar measurements calculating the Jacobian of the h function used to convert
   * the estimated radar measurements from cartesian coordinates to polar coordinates
   */

  void UpdateH_radar();

  /**
  * Prints all variables of Kalman Filter for debugging purposes
  */
  void printAllVariables();

  private:
  // measurement covariance matrix for radar
  Eigen::MatrixXd R_radar_;

  // observation matrix for radar (jacobian)
  Eigen::MatrixXd H_radar_;

};

#endif //EXTENDEDKF_RL