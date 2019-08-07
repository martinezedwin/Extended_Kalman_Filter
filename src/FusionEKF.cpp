#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  /**
   FusionEKF() initializes the folowing matricies: x, R_laser, R_radar, H_laser,F, P, Q
   */

  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  //Initialize x, F, H_laser, H_jacobian, P, noise
  //ekf_.x_ = VectorXd(4);     //State vector will contain [px, py, vx, vy]            
  ekf_.F_ = MatrixXd(4, 4);  //State transition matrix
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0 ,1;

  ekf_.P_ = MatrixXd(4, 4);  //Uncertainty covariance
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0 ,1;

  ekf_.Q_ = MatrixXd(4, 4);  //Process covariance matrix

  Hj_ = MatrixXd(3, 4);      //Jacobian matrix
  Hj_<< 1.0,0.0,0.0,0.0,
        0.0,1.0,0.0,0.0,
        0.0,0.0,1.0,0.0;
  
  //measurement covariance matrix
  H_laser_<< 1, 0, 0, 0,
             0, 1, 0, 0;

}
/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /** 
   * ProcessMeasurement() takes in the data and passes the data on to the prediciton and update step
     If it's a radar measurement, it will convert the incoming polar coordinates to cartisian px,py,vx,vy
        It also checks that the converted values are not too smal (i.e. <0.0001)
     If it's a laser measurment, it will leave it as is.
     It then assigns the cartitian px, and py values to x_
   */
  if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;
    //TODO: Initialize the state ekf_.x_ with the first measurement.
    ekf_.x_ = VectorXd(4);
    //ekf_.x_ << px, py, vx, vy
    std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!VERY INITIAL X!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    ekf_.x_ << 1.0, 1.0, 0.0, 0.0; //Assign initial values to ekf.x_

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!INITIALIZED WITH RADAR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";

      float rho = measurement_pack.raw_measurements_[0];      //range
      float phi = measurement_pack.raw_measurements_[1];      //bearing
      float rho_dot = measurement_pack.raw_measurements_[2];  //velocity
      std::cout<<" Radar - rho, phi, rho_dot: "<<rho<<" "<<phi<<" "<<" "<<rho_dot<<"\n";
      float px = rho*cos(phi);
      float py = rho*sin(phi);
      float vx = rho_dot*cos(phi);
      float vy = rho_dot*sin(phi);
      //std::cout<<" Radar - px, py, vx, vy: "<<px<<" "<<py<<" "<<vx<<" "<<vy<<"\n";
      ekf_.x_ << px,py,vx,vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!INITIALIZED LASER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
      // TODO: Initialize state.
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
      ekf_.x_(0) = px;
      ekf_.x_(1) = py;
    // done initializing, no need to predict or update
    
    cout << "Completed initialization of FusionEKF.\n";
    return;
    }
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }
  /***************************************************************************************************************
   *                                              PREDICTION STEP
   **************************************************************************************************************/
  //Define dt in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  std::cout<<"Delta t: "<<dt<<"\n";
  //Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  float noise_ax = 9.0;
  float noise_ay = 9.0;

  //TODO: Update the state transition matrix F according to the new elapsed time.
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;

  //TODO: Update the process noise covariance matrix.
  ekf_.Q_ << dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
            0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
            dt3/2*noise_ax, 0, dt2*noise_ax, 0,
            0, dt3/2*noise_ay, 0, dt2*noise_ay;

  ekf_.Predict();
  /*******************************************************************************************************************
   *                                                 UPDATE STEP
   ********************************************************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!RADAR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
    // TODO: Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_; //TODO: Update the state and covariance matrices.
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!LASER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
    // TODO: Laser updates
    ekf_.H_= H_laser_;
    ekf_.R_ = R_laser_; //TODO: Update the state and covariance matrices.
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = \n" << ekf_.x_ << endl;
  cout << "P_ = \n" << ekf_.P_ << endl;
}