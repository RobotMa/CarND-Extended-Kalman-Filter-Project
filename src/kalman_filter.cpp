#include "kalman_filter.h"
#include<iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  TODO:
    * predict the state
  */
    x_ = F_*x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_* P_*Ft + Q_; 
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_*x_;
  VectorXd y = z_pred - z;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd K = P_*Ht*S.inverse();

  x_ = x_ + K*y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  
  VectorXd z_pred(z.size());
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  z_pred << sqrt(px*px + py*py),
            atan2(py, px),
            (px*vx + py*vy)/sqrt(px*px + py*py);

  VectorXd y = z_pred - z;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;

  std::cout << "y is " << y << std::endl;
  std::cout << "Ht is " << Ht << std::endl;
  std::cout <<  "S is " << S << std::endl;

  MatrixXd K = P_*Ht*S.inverse();

  x_ = x_ + K*y;
  P_ = (I - K*H_)*P_;
}
