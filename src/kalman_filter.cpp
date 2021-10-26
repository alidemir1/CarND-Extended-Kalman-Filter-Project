#include "kalman_filter.h"
#include <math.h>


using Eigen::MatrixXd;
using Eigen::VectorXd;



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
  
  MatrixXd Ft = F_.transpose();
  x_ = F_*x_;
  P_ = F_* P_ * Ft + Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  
  MatrixXd Ht = H_.transpose();
  
  
  VectorXd y = z - (H_ * x_);
  MatrixXd S = H_ * P_ * Ht + R_;
  
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  
  long x_size = x_.size(); 
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  
  x_ = x_ + (K*y);
  P_ = (I - K*H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  
  
  MatrixXd Ht = H_.transpose();
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  
  float distance = sqrt(px*px + py*py);
  float angle = atan2(py, px);
  float rho_speed = (px*vx + py*vy)/distance;
  
  VectorXd h = VectorXd(3);
  h << distance, angle, rho_speed;
  
  
  
  VectorXd y = z - h;
  
  
  if(y(1) >= M_PI)
  {
  	y(1) -= 2.0 * M_PI;
  }
  else if(y(1) <= -M_PI)
  {
  	y(1) += 2.0 * M_PI;
  }
  
  MatrixXd S = H_ * P_ * Ht + R_;
  
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;
  
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  
  x_ = x_ + (K*y);
  P_ = (I - K*H_) * P_;
  
}
