#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}
void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  std::cout<<"PREDICTION\n";
  x_ = F_ * x_;

  MatrixXd F_transpose = F_.transpose();
  P_ = F_ * P_ * F_transpose + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  std::cout<<"UPDATE\n";
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  

  VectorXd y = z - (H_ * x_);
  
  MatrixXd K =  P_ * Ht * Si;

  // new state
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  std::cout<<"UPDATEEKF\n";
  float px=x_(0);
  float py=x_(1);
  float vx=x_(2);
  float vy=x_(3);

  float rho = sqrt(px*px + py*py);
  float phi = atan2(py, px);

  if(rho < 0.0001){
    rho = 0.0001;
  }

  float rho_dot = (px*vx + py*vy)/rho;

  VectorXd h = VectorXd(3);
  h << rho, phi, rho_dot;

  VectorXd y = z - h;

  //Normalizing [-PI, PI]
  if (y(1) > M_PI){
    y(1) = y(1) - 2*M_PI;
  }

  else if (y(1) < -M_PI){
    y(1) = y(1) + 2*M_PI;
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_ * Ht * Si;

  // new state
  x_ = x_ + (K * y);
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K * H_) * P_;
}
