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


  std::cout << "Size of x_in is " << x_in.size() << std::endl; 
  std::cout << "Size of P_in is " << P_in.size() << std::endl; 
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
  VectorXd z_pred = H_*z;
  VectorXd y = z_pred - z;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd K = P_*Ht + S.inverse();

  x_ = x_ + K*y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  MatrixXd Hj(3, 4);
  float px = z[0];
  float py = z[1];
  float vx = z[2];
  float vy = z[3];

  float c1 = px*px + py*py;
  float c2 = sqrt(c1);
  float c3 = c1*c2;

  if (c1 < 1E6)
  {
    return;
  }

  Hj << (px/c2), (py/c2), 0, 0,
       -(py/c1), (px/c1), 0, 0,
         py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  
  VectorXd z_pred = H_*z;
  VectorXd y = z_pred - z;
  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj*P_*Ht + R_;
  MatrixXd K = P_*Ht + S.inverse();

  x_ = x_ + K*y;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (I - K*H_)*P_;
}
