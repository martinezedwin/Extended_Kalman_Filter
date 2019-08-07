# Extended Kalman Filter Project Starter Code

Project Environment
---
In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements.

This project involves the Udacity Self-Driving Car Nanodegree Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

**Other Important Dependencies**
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)


Extended Kalman Filter
---
The EKF takes in sensor data, in this case lidar (red dots) and radar (blue dots). It then tries to predict the state (px, py, vx, vy) basead on the previous measurement in a loop. 

The predictions made can be seen in green triangles. These predictions where then compared to the actual ground truth value using the Root Mean Squared Error (RMSE) and aggregated for each point. 

The final RMSE values where [0.0945, 0.0851, 0.3953, 0.4590] for Data1 coming from the obj_pose-laser-radar-synthetic-input.txt.

[![Final result video](./Images/Extended_kalman_filter_video_cover.jpg)](https://www.youtube.com/watch?v=6V0R5MHgLpk)