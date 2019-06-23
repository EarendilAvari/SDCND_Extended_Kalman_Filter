#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  cycle_number = 0;

  // Creates local variables to initialize the Kalman filter matrices
  MatrixXd P_initial(4,4);
  MatrixXd F_initial(4,4);
  MatrixXd Q_initial(4,4);

  MatrixXd R_laser_initial(2,2);
  MatrixXd R_radar_initial(3,3);

  MatrixXd H_laser_initial(2,4);
  MatrixXd H_radar_initial(3,4);

  // P is initialized with 
  P_initial << 1.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 1000.0, 0.0,
               0.0, 0.0, 0.0, 1000.0;

  F_initial << 1.0, 0.0, 1.0, 0.0,
               0.0, 1.0, 0.0, 1.0,
               0.0, 0.0, 1.0, 0.0, 
               0.0, 0.0, 0.0, 1.0;

  Q_initial << 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0;

  R_laser_initial << 0.0225, 0.0,
                     0.0, 0.0225;

  R_radar_initial << 0.09, 0.0, 0.0,
                     0.0, 0.0009, 0.0,
                     0.0, 0.0, 0.09;

  H_laser_initial << 1.0, 0.0, 0.0, 0.0,
                     0.0, 1.0, 0.0, 0.0;

  H_radar_initial << 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0;
  

  // Initial values of other matrices of Kalman Filter
  ekf_ = ExtendedKF_rl(4, P_initial, F_initial, Q_initial, H_laser_initial, 
                      R_laser_initial, H_radar_initial, R_radar_initial);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

VectorXd FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    VectorXd firstMeasurement(4);
    float px_0, py_0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      px_0 = measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]);
      py_0 = measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      px_0 = measurement_pack.raw_measurements_[0];
      py_0 = measurement_pack.raw_measurements_[1];
    }

    firstMeasurement << px_0, py_0, 0.0, 0.0;
    ekf_.InitX(firstMeasurement);
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    
    /** Print 
    cout << "Kalman Filter initialized with: " << endl;
    ekf_.printAllVariables();
    **/

    return ekf_.getX();
  }

  /**
   * Prediction
   */

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  MatrixXd updated_F(4,4);
  updated_F << 1.0, 0.0, dt, 0.0,
               0.0, 1.0, 0.0, dt,
               0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 1.0;

  ekf_.updateF(updated_F);
  
  /**
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float noise_ax = 9.0;
  float noise_ay = 9.0;

  MatrixXd updated_Q(4,4);
  updated_Q << (pow(dt,4)/4.0)*noise_ax, 0.0, (pow(dt,3)/2.0)*noise_ax, 0.0,
               0.0, (pow(dt,4)/4.0)*noise_ay, 0.0, (pow(dt,3)/2.0)*noise_ay, 
               (pow(dt,3)/2.0)*noise_ax, 0.0, pow(dt,2.0)*noise_ax, 0.0,
               0.0, (pow(dt,3)/2.0)*noise_ay, 0.0, pow(dt,2)*noise_ay;

  ekf_.updateQ(updated_Q);

  ekf_.Predict();

  /**
   * 
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  cout << endl << endl << endl;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // Updates the H matrix for radar measurements
    cout << "Radar measurement" << endl << endl;
    ekf_.UpdateH_radar(); 
    
    VectorXd z(3);
    z << measurement_pack.raw_measurements_[0],
         measurement_pack.raw_measurements_[1],
         measurement_pack.raw_measurements_[2];

    ekf_.UpdateEKF(z);

  } else {
    // Laser updates
    cout << "Laser measurement" << endl << endl;
    VectorXd z(2);
    z << measurement_pack.raw_measurements_[0],
         measurement_pack.raw_measurements_[1];
    
    ekf_.Update(z);
    //ekf_.printAllVariables();
  }

  cycle_number++;
  if (cycle_number == 271) {
    cycle_number = 271;
  }

  // print the output
  cout << "Cycle: " << cycle_number << endl << endl;
  cout << "x_: " << endl << ekf_.getX() << endl << endl;
  cout << "P_: " << endl << ekf_.getP() << endl;

  return ekf_.getX();
}

VectorXd FusionEKF::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                  const std::vector<Eigen::VectorXd> &ground_truth) {
  
  VectorXd RMSE(4);
  RMSE << 0.0, 0.0, 0.0, 0.0;
  
  // Checks the validity of the inputs:
  if(estimations.size() == 0) {
    cout << "ERROR: Estimations vector has size 0" << endl;
    return RMSE;
  }
  else if (ground_truth.size() == 0) {
    cout << "ERROR: Ground truth vector has size 0" << endl;
    return RMSE;
  }
  else if (estimations.size() != ground_truth.size()) {
    cout << "ERROR: Estimation vector has different size than ground truth vector" << endl;
    return RMSE;
  }

  // Accumulates the square differences between estimations and ground truth in a vector
  VectorXd xt_diff2_sum(4);
  xt_diff2_sum << 0.0,0.0,0.0,0.0;

  for (int i=0; i < estimations.size(); i++) {
    VectorXd xt_est = estimations[i];
    VectorXd xt_true = ground_truth[i];

    VectorXd xt_diff = xt_est - xt_true;
    VectorXd xt_diff2 = xt_diff.array()*xt_diff.array();

    xt_diff2_sum += xt_diff2;
  }

  // Calculates the mean using the sum of the square differences
  float multiplier = (1.0/static_cast<float>(estimations.size()));
  VectorXd xt_diff2_mean = multiplier*xt_diff2_sum.array();

  // Calculates the squared root of the mean
  RMSE = xt_diff2_mean.array().sqrt();

  return RMSE;

}
