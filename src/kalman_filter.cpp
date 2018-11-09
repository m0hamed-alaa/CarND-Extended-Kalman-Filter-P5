#include "kalman_filter.h"
#include "tools.h"

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

  x_ = F_*x_ ;

  MatrixXd F_t = F_.transpose() ; 

  P_ = (F_*P_*F_t) + Q_ ;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  VectorXd y = z - (H_*x_) ; 

  MatrixXd H_t = H_.transpose() ; 

  MatrixXd S = (H_*P_*H_t) + R_ ;

  MatrixXd S_i = S.inverse() ;

  MatrixXd K = P_*H_t*S_i ; 

  // new update 

  x_ = x_ + (K*y) ; 

  long x_size = x_.size() ;
  
  MatrixXd I = MatrixXd::Identity(x_size , x_size) ;

  P_ = ( I - (K*H_))*P_ ;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

 

 float px = x_(0) ;
 float py = x_(1) ;
 float vx = x_(2) ;
 float vy = x_(3) ;

 float t = px*px + py*py ;

 if (fabs(t) < 0.0001)
 {
   return ;
 }

  

  VectorXd h(3) ;

  h << sqrt(t) , atan2(py,px) , (px*vx + py*vy)/sqrt(t) ;

  VectorXd y = z - h ; 

  // normalizing phi angles to be in range -pi to pi

  float phi = y(1) ;

  while(phi > M_PI) 
  {
    phi = phi - 2*M_PI ;
  }

  while(phi < -M_PI)
  {
    phi = phi + 2*M_PI ;
  }
  
  y(1) = phi ;

  // note H_ here is Hj
  
  MatrixXd Hj_t = H_.transpose() ; 

  MatrixXd S = (H_*P_*Hj_t) + R_ ;

  MatrixXd S_i = S.inverse() ;

  MatrixXd K = P_*Hj_t*S_i ; 

  // new update 

  x_ = x_ + (K*y) ; 

  long x_size = x_.size() ;

  MatrixXd I = MatrixXd::Identity(x_size , x_size) ;

  P_ = ( I - (K*H_))*P_ ;


}
