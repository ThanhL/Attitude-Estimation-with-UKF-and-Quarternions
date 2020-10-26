# Attitude-Estimation-with-UKF-and-Quarternions

**Unscented Kalman filter implementation for Teensy/Arduino**

This repository covers an attitude estimation system with the use of quaternions and Unscented Kalman Filters. This project using the LIS3MDL+LSM6DS33 sensor module that provides accelerometer, gyroscope and magnetometer data. The code can be uploaded to a Teensy/Arduino which does the Unscented Kalman Filter calculations onboard the microcontroller and prints the quaternion results to Serial.

To visualize the quaternion, a Processing sketch was developed (located in the Processing folder). The sketch reads the quaternion data from USB serial outputted by the Teensy/Arduino and then applies the rotation onto an object for visualization as shown:

*** Insert Video ***


## Setup/Installation
### Hardware Requirements
The following code was tested/developed with the following hardware:

`Adafruit LIS3MDL+LSM6DS33 8 DoF sensor module`: This IMU module provides us with accelerometer, gyroscope and magnetometer data. These motion data are fed into the Unscented Kalman Filter and fused together to estimate the orientation. The sensor can be purchased here: https://www.adafruit.com/product/4485

`Teensy 4.0`:  The Unscented Kalman Filter code is uploaded onto this microcontroller. The Teensy 4.0 was used to take advantage of 32 bit ARM Cortex-M7, which provides a significantly faster/more powerful processor than the Arduino microcontrollers. This allowed for fast matrix math when using Eigen without having to worry about the 8-bit constraints of the Arduino microcontroller! The microcontroller can be purchased here: https://www.adafruit.com/product/4323

#### Wiring it up!

For this project we just need to connect the IMU to teensy over I2C as shown in the schematic below. The Serial communications + Power would just come from the USB port! 

*** Insert Wiring Diagram ***

### Library Dependencies

`Eigen`: The code uses Eigen library for linear alebgra, matrix and vector operations that is used in the Unscented Kalman Filter Math. A Eigen port to Arduino/Teensy + installation steps can be found here: https://github.com/moritzmaier/Eigen. 


## Calibration

To get the very best results, the sensors need to be first calibrated to ensure they work properly:

`Accelerometer Calibration`: The accelerometer did not need to be calibrated for this project as the sensor itself gave quite 'clean' values already. However, we could potentially improve the orientation estimation by looking into calibrating the accelerometer as well.

`Gyroscope Calibration`: Gyroscopes needed to be calibrated since they have some zero-offset (this is due to how it was manufactured) that could affect the results of the orientation estimation in UKF. Luckily, Adafruit provides a python jupyter notebook that calculates these gyroscope offsets provided by this link: https://learn.adafruit.com/adafruit-sensorlab-gyroscope-calibration/gyro-calibration-with-jupyter. After running the gyroscope calibration procedure, the offsets were then placed into global variables of the main Arduino script (attitude_estimation.ino) in the variables;

```C++
const float gyro_offset_x = 0.06285;
const float gyro_offset_y = -0.08785;
const float gyro_offset_z = -0.06815;
```
where `gyro_offset_x, gyro_offset_y, gyro_offset_z` are the offsets calculated from the gyroscope in the x,y,z direction respectively.   


`Magnetometer Calibration`: Magnetometers are probably the most essential sensors that need to be calibrated as they have to measure Earth's magnetic field to detect orientation. In the presence of enviromental magnetic noise and manufacturing defects, these factors can drastically affect the readings of magnetometer if calibration is taken into consideration. There are too main factors to be considered when calibrating magnetometers:

* Soft Iron Effects: Caused by objects near the sensors that distort the surrounding magnetic field. These soft iron effects causes a stretch and tilt of the ideal magnetometer measurements (more information about ideal magnetometer readings: https://au.mathworks.com/help/nav/ug/magnetometer-calibration.html). Despite these, they  *don't contribute as much to the noise of the magnetometer readings therefore for the purpose of simplification the soft iron effects are ignored for this project for now*. However in future, a more sophiscated calibration procedure that accounts for these effects may be implemented.

* Hard Iron Effects: Caused by stationary interfering magnetic noise sources, the hard iron effects can significantly degrade the magnetometer's readings. The hardiron effects shifts the origin of the ideal magnetic readings (more information about ideal magnetometer readings: https://au.mathworks.com/help/nav/ug/magnetometer-calibration.html). They have a much larger contribution to the error in magnetomer's measurements than Soft Iron effects. Therefore for the purposes of this project, the magnetometer is calibrated to adjust for these hardiron offsets. These hardiron offset values can be calculated using Adafruit's python jupyter notebook calibration: https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration/magnetic-calibration-with-jupyter. These offsets were then accounted for inside attitude_estimation.ino in the following variables;

```C++
const float mag_hardiron_offset_x = -31.71;
const float mag_hardiron_offset_y = 28.61;
const float mag_hardiron_offset_z = 33.985;
```
where `mag_hardiron_offset_x, mag_hardiron_offset_y, mag_hardiron_offset_z` are the hardiron offsets calculated from the magnetometer in the x,y,z direction respectively.

## Uploading the Unscented Kalman Filter and Visualize the Orientation Estimation

After you've installed the wire up the sensors, installed library dependencies and followed the calibration process, we are now ready to upload the unscented Kalman Filter code and visualize the orientation! First open the attitude_estimation.ino project into Arduino IDE and click upload! Check that the quaternion numbers are outputting to the Serial Monitor of Arduino (there should be four numbers which represent q0,q1,q2,q3 quaternion elements in the quaternion `q0 + q1*i + q2*j + q3*k`).

Once the quaternion elements are verified to be outputting to Serial, we can now visualize the orientation estimation in Processing. Open up the OrientationVisualization.pde sketch in Processing and click the play button. This should open up a GUI that will rotate the object as you move the IMU around (as shown in the video Youtube).

## Technical Details/Improvements

### Unscented Kalman Filter Process/Measurement Models

The Unscented Kalman Filter largely follows the process model and measurement model from Edgar Kraft's *A Quaternion-based Unscented Kalman Filter for Orientation Tracking* paper. More details about UKF process+measurement model can be found in the Jupyter Notebook [here](https://github.com/ThanhL/Attitude-Estimation-with-UKF-and-Quarternions/blob/master/Notebooks/Unscented%20Kalman%20Filter%20Orientation%20Estimation%20Math.ipynb) (located in the Notebooks folder of this repo).


### Possible Improvements

`Tuning Noise Matrices Q,R:` The noise matrices Q (process noise matrix) and R (measurement noise matrix) were tuned simplistically for this project. However in future, a better tuning of these matrices could potentially improve the performance of the orientation estimation. 

`Improve Calibration Process:` We could potentially improve the calibration process of the sensors. Possible calibrations improvements could be:
* Add a calibration process for the accelerometer to get cleaner readings for the accelerometer
* Improve the calibration process of the magnetometer to account for *both* hard iron offsets and soft iron offsets
* Research/potentially improve the gyroscope calibration process
* Add a calibration process in the intial setup of the IMU (in the Teensy/Arduino code) so we don't have to calibrate in separate environment (i.e finding calibrated values in a jupyter notebook and incorporating them into the Teensy/Arduino code).

`Improve Process/Measurement Models and Sigma Points:` We could potentially improve the process model/measurement model to include more states such as biases from accelerometer/angular velocity/magnetometer. We could also tune the parameters of merwe sigma points (alpha, beta, kappa) to potentially improve the UKF.

`Smooth Orientation Estimation:` The UKF orientation is able to track the orientation pretty well, however the estimations outputted aren't quite smooth as they vary quite a bit around steady state estimates (hence the jerky motion). A Kalman smoother can be added to improve tracking of the Kalman filters, enabling a smoother steady state + smoother transitions from state estimates.

## References
[1] Merwe, Rudolph., Wan, Eric. *Sigma-Point Kalman Filters for ProbabilisticInference in Dynamic State-Space Models*, 2004.

[2] Kraft, Edgar., *A Quaternion-based Unscented Kalman Filter for Orientation Tracking*, 2003.
https://natanaso.github.io/ece276a2017/ref/2_Kraft_UKF.pdf

[3] Corke, Peter., *Robotics, Vision and Control*, 2016.

[4] Thrun, Sebastian., Burgard, Wolfram., Fox, Dieter. *Probablistic Robotics*, 2005.

[5] Kalman and Bayesian Filters in Python - Unscenter Kalman Filters, Roger Labbe. https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/10-Unscented-Kalman-Filter.ipynb