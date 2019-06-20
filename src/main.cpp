#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "FusionEKF.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;
using std::vector;
using std::ifstream;
using std::cout;
using std::endl;
using std::istringstream;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  vector<VectorXd> est_vector;
  vector<VectorXd> gt_vector;

  h.onMessage([&fusionEKF,&est_vector,&gt_vector]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          string sensor_measurement = j[1]["sensor_measurement"];
          
          MeasurementPackage meas_package;
          std::istringstream iss(sensor_measurement);
          
          long long timestamp;

          // reads first element from the current line
          string sensor_type;
          iss >> sensor_type;

          if (sensor_type.compare("L") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          } else if (sensor_type.compare("R") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, theta, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          }

          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;

          VectorXd gt_values(4);
          gt_values(0) = x_gt;
          gt_values(1) = y_gt; 
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;
          gt_vector.push_back(gt_values);
          
          // Call ProcessMeasurement(meas_package) for Kalman filter
          VectorXd est_values(4);
          est_values = fusionEKF.ProcessMeasurement(meas_package);     

          // Push the current estimated x,y positon from the Kalman filter's 
          //   state vector
          est_vector.push_back(est_values);  

          VectorXd RMSE = fusionEKF.CalculateRMSE(est_vector, gt_vector);

          json msgJson;
          msgJson["estimate_x"] = est_values(0);
          msgJson["estimate_y"] = est_values(1);
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }  // end "telemetry" if

      } else {
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if

  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}

int main_test() {
  /**
   * Set Measurements
   */
  vector<MeasurementPackage> measurement_pack_list;
  
  // Vector to save estimation values to calculate RSME
  vector<VectorXd> est_vector;
  // Vector to save ground truth values to calculate RSME
  vector<VectorXd> gt_vector;

  // hardcoded input file with laser and radar measurements
  string in_file_name = "debugDataFile2.txt";
  ifstream in_file(in_file_name.c_str(), ifstream::in);

  if (!in_file.is_open()) {
    cout << "Cannot open input file: " << in_file_name << endl;
  }

  string line;

  // Set i to get only first 6 measurements

  int i = 0;
  while (getline(in_file, line)) {

    MeasurementPackage meas_package;

    istringstream iss (line);

    string sensor_type;
    iss >> sensor_type; // Reads the first elemenf from the current line
    int64_t timestamp;

    if (sensor_type.compare("L") == 0) { // Laser measurement
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;

      meas_package.raw_measurements_ << x, y;

      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);

    }
    else if (sensor_type.compare("R") == 0) {
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float rho;
      float theta;
      float rho_dot;
      iss >> rho;
      iss >> theta;
      iss >> rho_dot;

      meas_package.raw_measurements_ << rho, theta, rho_dot;

      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

    VectorXd gt_values(4);
    float gt_px;
    float gt_py;
    float gt_vx;
    float gt_vy;

    iss >> gt_px;
    iss >> gt_py;
    iss >> gt_vx;
    iss >> gt_vy;

    gt_values << gt_px, gt_py, gt_vx, gt_vy;

    gt_vector.push_back(gt_values);

    i++;
  }

  // Create a FusionEKF instance
  FusionEKF fusionEKF;

  // Call the ProcessMeasurement() function for each measurement
  size_t N = measurement_pack_list.size();

  VectorXd est_values(4);
  for (size_t k = 0; k < N; k++) {
    est_values = fusionEKF.ProcessMeasurement(measurement_pack_list[k]);
    est_vector.push_back(est_values);
  }

  VectorXd filter_RMSE(4);
  filter_RMSE = fusionEKF.CalculateRMSE(est_vector, gt_vector);

  cout << endl << "The RMSE (Root mean squared value) of the filter with " << N << " measurements is: " << endl;
  cout << "px: " << filter_RMSE[0] << endl;
  cout << "py: " << filter_RMSE[1] << endl;
  cout << "vx: " << filter_RMSE[2] << endl;
  cout << "vy: " << filter_RMSE[3] << endl;

  if (in_file.is_open()) {
    in_file.close();
  }

}