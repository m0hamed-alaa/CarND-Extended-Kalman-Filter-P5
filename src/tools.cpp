#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse(estimations[0].size()) ;
  rmse << 0 , 0 , 0 , 0 ;

  if((estimations.size() != ground_truth.size()) || (estimations.size() == 0))
  {
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

  
  VectorXd residual(estimations[0].size()) ;
  residual << 0 , 0 , 0 , 0 ;
  

  for(unsigned int i=0 ; i < estimations.size() ; i++)
  {
    residual = estimations[i] - ground_truth[i] ;
    residual = residual.array()*residual.array() ;
    rmse = rmse + residual ;
  }

  rmse = rmse/float(estimations.size()) ;
  rmse = rmse.array().sqrt() ;

  return rmse ;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd Hj(3,4) ; 

  float px = x_state(0) ;
  float py = x_state(1) ;
  float vx = x_state(2) ; 
  float vy = x_state(3) ;

  float t = px*px + py*py ;

  if (fabs(t) < 0.0001)
  {
    Hj<< 0 , 0 , 0 , 0 ,
         0 , 0 , 0 , 0 ,
         0 , 0 , 0 , 0 ;
  }

  else 
  {
    Hj << px/sqrt(t) , py/sqrt(t) , 0 , 0 ,

        -py/t        , px/t       , 0 , 0 ,

        (py*(vx*py-vy*px))/(t*sqrt(t)) , (px*(vy*px-vx*py))/(t*sqrt(t)) , px/sqrt(t) , py/sqrt(t) ;
  }
  
  return Hj ; 
}
