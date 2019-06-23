# Self Driving Car Nanodegree 

## Project 5: Extended Kalman Filter

This project consists of a sensor fusion system with radar and laser which measure the position of another car respective to an origin point where the sensors are located. In order to combinate the data, an extended Kalman filter is used. The system estimates the 2D position of the other car and its speed. 

This software is programmed in C++ 11 using the standard template library and the external library Eigen for linear algebra calculations. 

The software was debugged using 500 radar and laser measurements saved in a csv file. 

In order to visualize the results of the software, the [ Udacity term 2 simulator ](https://github.com/udacity/self-driving-car-sim) is used. To communicate this software with the simulator the [uWebSockets API](https://github.com/uNetworking/uWebSockets) is used. 

To compile the software, GCC 7.4.0 and CMake 3.15.0-rc1 is used.

### About the algorithms

The purpose of this software is to implement an extended Kalman filter which can combinate radar and laser data. In this section it is explained briefly how the algorithm works.

A Kalman filter is an algorithm which uses noisy measurements of different sensors in order to get more accurate values of the observable variables (the ones sensed) and estimate other non observable variables. These values are represented by a gaussian distribution with mean x and covariance P. x is a vector of length n including n estimated variables and P is a matrix of dimensions nxn including the covariances between the n estimated variables.

The algorithm is a linear dynamic system and therefore can be expressed using matrices. It consists of two separated phases: a prediction phase where the algorithm predicts the measurements using the system dynamics and without using the sensor data and other phase where the algorithm updates the measurements using the sensor data. 

#### Prediction phase

The first phase which is called "prediction phase" can be described using the following equation:

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;x_{k}&space;=&space;F&space;\cdot&space;x_{k-1}&space;&plus;&space;u_{k}&space;(1)&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;x_{k}&space;=&space;F&space;\cdot&space;x_{k-1}&space;&plus;&space;u_{k}&space;(1)&space;$$" title="$$ x_{k} = F \cdot x_{k-1} + u_{k} (1) $$" /></a>

Where:
- <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;x_{k}&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;x_{k}&space;$$" title="$$ x_{k} $$" /></a> : Mean of predicted measurements on this cycle.

- <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;F&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;F&space;$$" title="$$ F $$" /></a> : State transition matrix.

- <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;x_{k-1}&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;x_{k-1}&space;$$" title="$$ x_{k-1} $$" /></a> : Mean of updated measurements on the last cycle.
- <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;u_{k}&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;u_{k}&space;$$" title="$$ u_{k} $$" /></a> : External input.

The vector <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;x&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;x&space;$$" title="$$ x $$" /></a> can be descomposed as following: 

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;x&space;=&space;\begin{pmatrix}&space;p_{x}&space;\\&space;p_{y}&space;\\&space;v_{x}&space;\\&space;v_{y}&space;\end{pmatrix}&space;(2)&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;x&space;=&space;\begin{pmatrix}&space;p_{x}&space;\\&space;p_{y}&space;\\&space;v_{x}&space;\\&space;v_{y}&space;\end{pmatrix}&space;(2)&space;$$" title="$$ x = \begin{pmatrix} p_{x} \\ p_{y} \\ v_{x} \\ v_{y} \end{pmatrix} (2) $$" /></a>

In the case of the filter used on this project, it is assumed that the speed stays constant and there is no external input, so:

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;p_{x_{k}}&space;=&space;p_{x_{k-1}}&space;&plus;&space;\Delta&space;t&space;\cdot&space;v_{x_{k-1}}&space;(3)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;p_{x_{k}}&space;=&space;p_{x_{k-1}}&space;&plus;&space;\Delta&space;t&space;\cdot&space;v_{x_{k-1}}&space;(3)$$" title="$$ p_{x_{k}} = p_{x_{k-1}} + \Delta t \cdot v_{x_{k-1}} (3)$$" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;p_{y_{k}}&space;=&space;p_{y_{k-1}}&space;&plus;&space;\Delta&space;t&space;\cdot&space;v_{y_{k-1}}&space;(4)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;p_{y_{k}}&space;=&space;p_{y_{k-1}}&space;&plus;&space;\Delta&space;t&space;\cdot&space;v_{y_{k-1}}&space;(4)$$" title="$$ p_{y_{k}} = p_{y_{k-1}} + \Delta t \cdot v_{y_{k-1}} (4)$$" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;v_{x_{k}}&space;=&space;v_{x_{k-1}}&space;(5)&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;v_{x_{k}}&space;=&space;v_{x_{k-1}}&space;(5)&space;$$" title="$$ v_{x_{k}} = v_{x_{k-1}} (5) $$" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;v_{y_{k}}&space;=&space;v_{y_{k-1}}&space;(6)&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;v_{y_{k}}&space;=&space;v_{y_{k-1}}&space;(6)&space;$$" title="$$ v_{y_{k}} = v_{y_{k-1}} (6) $$" /></a>

From these equations, the F matrix can be defined as: 

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;F&space;=&space;\begin{pmatrix}&space;1&space;&&space;0&space;&&space;\Delta&space;t&space;&&space;0&space;\\&space;0&space;&&space;1&space;&&space;0&space;&&space;\Delta&space;t&space;\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{pmatrix}(7)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;F&space;=&space;\begin{pmatrix}&space;1&space;&&space;0&space;&&space;\Delta&space;t&space;&&space;0&space;\\&space;0&space;&&space;1&space;&&space;0&space;&&space;\Delta&space;t&space;\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{pmatrix}(7)$$" title="$$ F = \begin{pmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{pmatrix}(7)$$" /></a>

Since there is no external input, <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;u&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;u&space;$$" title="$$ u $$" /></a> is 0 in all its components and therefore not part of the algorithm.

The matrix <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;P&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;P&space;$$" title="$$ P $$" /></a> is not used on this phase but it is also predicted here using the following equation:

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;P_{k}&space;=&space;F&space;\cdot&space;P_{k-1}&space;\cdot&space;F^{T}&space;&plus;&space;Q&space;(8)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;P_{k}&space;=&space;F&space;\cdot&space;P_{k-1}&space;\cdot&space;F^{T}&space;&plus;&space;Q&space;(8)$$" title="$$ P_{k} = F \cdot P_{k-1} \cdot F^{T} + Q (8)$$" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;Q&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;Q&space;$$" title="$$ Q $$" /></a> corresponds to the process noise matrix which is given as:

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;Q&space;=&space;\begin{pmatrix}&space;\frac{\Delta&space;t^{4}}{4}&space;\sigma_{ax}^{2}&space;&&space;0&space;&&space;\frac{\Delta&space;t^{3}}{2}&space;\sigma_{ax}^{2}&space;&&space;0&space;\\&space;0&space;&&space;\frac{\Delta&space;t^{4}}{4}&space;\sigma_{ay}^{2}&space;&&space;0&space;&&space;\frac{\Delta&space;t^{3}}{2}&space;\sigma_{ay}^{2}&space;\\&space;\frac{\Delta&space;t^{3}}{2}&space;\sigma_{ax}^{2}&space;&&space;0&space;&&space;\Delta&space;t^{2}&space;\sigma_{ax}^{2}&space;&&space;0&space;\\&space;0&space;&&space;\frac{\Delta&space;t^{3}}{2}&space;\sigma_{ay}^{2}&space;&&space;0&space;&&space;\Delta&space;t^{2}&space;\sigma_{ay}^{2}&space;\end{pmatrix}(9)$$." target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;Q&space;=&space;\begin{pmatrix}&space;\frac{\Delta&space;t^{4}}{4}&space;\sigma_{ax}^{2}&space;&&space;0&space;&&space;\frac{\Delta&space;t^{3}}{2}&space;\sigma_{ax}^{2}&space;&&space;0&space;\\&space;0&space;&&space;\frac{\Delta&space;t^{4}}{4}&space;\sigma_{ay}^{2}&space;&&space;0&space;&&space;\frac{\Delta&space;t^{3}}{2}&space;\sigma_{ay}^{2}&space;\\&space;\frac{\Delta&space;t^{3}}{2}&space;\sigma_{ax}^{2}&space;&&space;0&space;&&space;\Delta&space;t^{2}&space;\sigma_{ax}^{2}&space;&&space;0&space;\\&space;0&space;&&space;\frac{\Delta&space;t^{3}}{2}&space;\sigma_{ay}^{2}&space;&&space;0&space;&&space;\Delta&space;t^{2}&space;\sigma_{ay}^{2}&space;\end{pmatrix}(9)$$." title="$$ Q = \begin{pmatrix} \frac{\Delta t^{4}}{4} \sigma_{ax}^{2} & 0 & \frac{\Delta t^{3}}{2} \sigma_{ax}^{2} & 0 \\ 0 & \frac{\Delta t^{4}}{4} \sigma_{ay}^{2} & 0 & \frac{\Delta t^{3}}{2} \sigma_{ay}^{2} \\ \frac{\Delta t^{3}}{2} \sigma_{ax}^{2} & 0 & \Delta t^{2} \sigma_{ax}^{2} & 0 \\ 0 & \frac{\Delta t^{3}}{2} \sigma_{ay}^{2} & 0 & \Delta t^{2} \sigma_{ay}^{2} \end{pmatrix}(9)$$." /></a>

In both <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;F&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;F&space;$$" title="$$ F $$" /></a> and <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;Q&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;Q&space;$$" title="$$ Q $$" /></a>, <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;\Delta&space;t&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;\Delta&space;t&space;$$" title="$$ \Delta t $$" /></a> corresponds to the time difference in seconds between the current and last measurement.

#### Update phase

In this phase, the measurement from radar or laser is used in order to update the measurement mean vector <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;x&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;x&space;$$" title="$$ x $$" /></a> and measurement covariance matrix <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;P&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;P&space;$$" title="$$ P $$" /></a>. 

For that, the following equations are used:

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;x_{upd}&space;=&space;x_{pred}&space;&plus;&space;K&space;y&space;(10)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;x_{upd}&space;=&space;x_{pred}&space;&plus;&space;K&space;y&space;(10)$$" title="$$ x_{upd} = x_{pred} + K y (10)$$" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;P_{upd}&space;=&space;(I&space;-&space;K&space;H)P_{pred}&space;(11)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;P_{upd}&space;=&space;(I&space;-&space;K&space;H)P_{pred}&space;(11)$$" title="$$ P_{upd} = (I - K H)P_{pred} (11)$$" /></a>

Where:

- <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;y&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;y&space;$$" title="$$ y $$" /></a>: Difference between predicted observable variables and estimated observable variables.
- <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;K&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;K&space;$$" title="$$ K $$" /></a>: Kalman gain.
- <a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;H&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;H&space;$$" title="$$ H $$" /></a>: Observation matrix.

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;y&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;y&space;$$" title="$$ y $$" /></a> is calculated as:

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;y&space;=&space;z&space;-&space;H&space;x_{pred}(12)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;y&space;=&space;z&space;-&space;H&space;x_{pred}(12)$$" title="$$ y = z - H x_{pred}(12)$$" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;z&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;z&space;$$" title="$$ z $$" /></a> corresponds to the input from the sensor which is (position x, position y) in the case of laser and (position module, position angle, speed module) in the case of radar.

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;H&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;H&space;$$" title="$$ H $$" /></a> is the matrix which transforms the vector x with all observable and not observable values to a vector with only the observable values.

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;K&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;K&space;$$" title="$$ K $$" /></a> can be calculated using the following equations:

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;K&space;=&space;P_{pred}&space;H^{T}&space;S^{-1}(13)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;K&space;=&space;P_{pred}&space;H^{T}&space;S^{-1}(13)$$" title="$$ K = P_{pred} H^{T} S^{-1}(13)$$" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;S&space;=&space;H&space;P_{pred}&space;H_{T}&space;&plus;&space;R&space;(14)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;S&space;=&space;H&space;P_{pred}&space;H_{T}&space;&plus;&space;R&space;(14)$$" title="$$ S = H P_{pred} H_{T} + R (14)$$" /></a>

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$$&space;R&space;$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$$&space;R&space;$$" title="$$ R $$" /></a> is the measurement covariance matrix and is of dimensions 2x2 for laser measurements and 3x3 for radar measurements.

#### Extended Kalman filter

The Kalman filter works directly only with laser measurements, since a laser measures the position x and y directly. The radar instead measures the distance to the object from the radar, the angle respective to the radar and the speed of the object. In the equation (12) there is no H matrix which can convert the predicted values from cartesian coordinates to polar coordinates. Instead, a non linear function needs to be used. This function is defined as:

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;h(x_{pred})&space;=&space;\begin{pmatrix}&space;\sqrt{p_{x}^{2}&plus;p_{y}^{2}}\\&space;arctan(p_{y}/p_{x})&space;\\&space;\frac{p_{x}v_{x}&space;&plus;&space;p_{y}v_{y}}{\sqrt{p_{x}^{2}&plus;p_{y}^{2}}}&space;\end{pmatrix}&space;(15)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;h(x_{pred})&space;=&space;\begin{pmatrix}&space;\sqrt{p_{x}^{2}&plus;p_{y}^{2}}\\&space;arctan(p_{y}/p_{x})&space;\\&space;\frac{p_{x}v_{x}&space;&plus;&space;p_{y}v_{y}}{\sqrt{p_{x}^{2}&plus;p_{y}^{2}}}&space;\end{pmatrix}&space;(15)$$" title="$$ h(x_{pred}) = \begin{pmatrix} \sqrt{p_{x}^{2}+p_{y}^{2}}\\ arctan(p_{y}/p_{x}) \\ \frac{p_{x}v_{x} + p_{y}v_{y}}{\sqrt{p_{x}^{2}+p_{y}^{2}}} \end{pmatrix} (15)$$" /></a>

For the equations (13) and (14) the Jacobian matrix of the h function is used. This matrix corresponds to:

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;H_{j}&space;=&space;\begin{pmatrix}&space;\frac{p_{x}}{\sqrt{p_{x}^{2}&plus;p_{y}^{2}}}&space;&&space;\frac{p_{y}}{\sqrt{p_{y}^{2}&plus;p_{y}^{2}}}&space;&&space;0&space;&&space;0&space;\\&space;-&space;\frac{p_{y}}{p_{x}^{2}&plus;p_{y}^{2}}&space;&&space;\frac{p_{x}}{p_{x}^{2}&plus;p_{y}^{2}}&space;&&space;0&space;&&space;0&space;\\&space;\frac{p_{y}(v_{x}p_{y}-&space;v_{y}p_{x})}{(p_{x}^{2}&plus;p_{y}^{2})^{3/2}}&space;&&space;\frac{p_{x}(v_{y}p_{x}-&space;v_{x}p_{y})}{(p_{x}^{2}&plus;p_{y}^{2})^{3/2}}&space;&&space;\frac{p_{x}}{\sqrt{p_{x}^{2}&plus;p_{y}^{2}}}&space;&&space;\frac{p_{y}}{\sqrt{p_{y}^{2}&plus;p_{y}^{2}}}&space;\end{pmatrix}(16)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;H_{j}&space;=&space;\begin{pmatrix}&space;\frac{p_{x}}{\sqrt{p_{x}^{2}&plus;p_{y}^{2}}}&space;&&space;\frac{p_{y}}{\sqrt{p_{y}^{2}&plus;p_{y}^{2}}}&space;&&space;0&space;&&space;0&space;\\&space;-&space;\frac{p_{y}}{p_{x}^{2}&plus;p_{y}^{2}}&space;&&space;\frac{p_{x}}{p_{x}^{2}&plus;p_{y}^{2}}&space;&&space;0&space;&&space;0&space;\\&space;\frac{p_{y}(v_{x}p_{y}-&space;v_{y}p_{x})}{(p_{x}^{2}&plus;p_{y}^{2})^{3/2}}&space;&&space;\frac{p_{x}(v_{y}p_{x}-&space;v_{x}p_{y})}{(p_{x}^{2}&plus;p_{y}^{2})^{3/2}}&space;&&space;\frac{p_{x}}{\sqrt{p_{x}^{2}&plus;p_{y}^{2}}}&space;&&space;\frac{p_{y}}{\sqrt{p_{y}^{2}&plus;p_{y}^{2}}}&space;\end{pmatrix}(16)$$" title="$$ H_{j} = \begin{pmatrix} \frac{p_{x}}{\sqrt{p_{x}^{2}+p_{y}^{2}}} & \frac{p_{y}}{\sqrt{p_{y}^{2}+p_{y}^{2}}} & 0 & 0 \\ - \frac{p_{y}}{p_{x}^{2}+p_{y}^{2}} & \frac{p_{x}}{p_{x}^{2}+p_{y}^{2}} & 0 & 0 \\ \frac{p_{y}(v_{x}p_{y}- v_{y}p_{x})}{(p_{x}^{2}+p_{y}^{2})^{3/2}} & \frac{p_{x}(v_{y}p_{x}- v_{x}p_{y})}{(p_{x}^{2}+p_{y}^{2})^{3/2}} & \frac{p_{x}}{\sqrt{p_{x}^{2}+p_{y}^{2}}} & \frac{p_{y}}{\sqrt{p_{y}^{2}+p_{y}^{2}}} \end{pmatrix}(16)$$" /></a>

For the prediction phase the equations used on this project are the same for radar and laser measurements.

### About the software structure

The software is object oriented structured, with four classes:
- MeasurementPackage: Objects of this class contain the raw data received from the CSV file or the simulator.
- KalmanFilter: This class contains the variables and methods needed to run a normal Kalman filter.
- ExtendedKF_rl: This class inherits the variables and methods of the KalmanFilter class and has additionaly some other variables and methods in order to run an extended Kalman filter. Different than in the normal KalmanFilter class, this class is not general, it is made exactly for this application.
- FusionEKF: This class runs the extended Kalman filter algorithm using radar or laser measurements.

The structure can be explained with the following class diagram:

![ Image1](./Images/ClassDiagram.png  "Class diagram")


The method ProcessMeasurement of the class FusionEKF is the core point of this project, it is called by the main function for every measurement package. The following diagram describes how this method works:

![ Image2](./Images/ProcessMeasurement.png  "Process measurement")

The file main.cpp contains two main functions, one used mainly for debugging which uses a txt file with measurements to run the methods ProcessMeasurement and CalculateRMSE of FusionEKF. The other main function uses uWebSockets to communicate with the Udacity term 2 simulator. There it can be seen how the object actually moves arround the measuring car, together with the raw laser and radar measurements and the estimations of the algorithm.

### Results

By running the software at the first time on the file obj_pose-laser-radar-synthetic-input.txt the RMSE was very high, reaching 2 for the speed on the x axis. After adding to the method UpdateEKF of ExtendedKF_rl a correction step for the measured angle on y_ (see equation 12) the results improved drastically, reaching an RMSE of 0.097, 0.085, 0.450, 0.440. By running the software on the second dataset using the simulator an RMSE of 0.0723, 0.0969, 0.4137, 0.5277 is reached.

### How to use

The software is already compiled in this repository in the folder "build", but here is explained how it can be compiled again:

- Install the required dependencies: gcc/g++ >= 5.4, make >= 4.1, cmake >= 3.5.
- Install uWebSockets using the file /install-linux.sh
- Delete the directory build and create a new one with: mkdir build.
- Move to the build directory with: cd build.
- Compile with: cmake .. && make.
- Run the program with: ./ExtendedKF.
- Open the term 2 simulator and select Project 1/2: EKF and UKF.
- Press start and see how the car moves and the measurements and estimations are shown.

