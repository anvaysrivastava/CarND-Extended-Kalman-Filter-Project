#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

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
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;
    
 //new estimate
 x_ = x_ + (K * y);
 long x_size = x_.size();
 MatrixXd I = MatrixXd::Identity(x_size, x_size);
 P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    
    float measured_range = z(0);
    float measured_angle = z(1);
    float measured_range_rate = z(2);
    
    float measured_px = measured_range*cos(measured_angle);
    float measured_py = measured_range*sin(measured_angle);
    float measured_vx = measured_range_rate*cos(measured_angle);
    float measured_vy = measured_range_rate*sin(measured_angle);
    cout<<"Measured px="<<measured_px<<" py="<<measured_py<<" vx="<<measured_vx<<" vy="<<measured_vy<<endl;
    
    float predicted_px = x_(0);
    float predicted_py = x_(1);
    float predicted_vx = x_(2);
    float predicted_vy = x_(3);
    
    cout<<"Predicted px="<<predicted_px<<" py="<<predicted_py<<" vx="<<predicted_vx<<" vy="<<predicted_vy<<endl;
    
    float predicted_range = sqrt(predicted_px*predicted_px+predicted_py*predicted_py);
    float predicted_angle = atan2(predicted_py,predicted_px);
    float predicted_range_rate = (predicted_px*predicted_vx + predicted_py*predicted_vy)/predicted_range;

    if(fabs(predicted_px)< 0.0001 || fabs(predicted_range)< 0.0001){
        cout << "UpdateEKF () - Error - Division by Zero" << endl;
        return;
    }
    
    VectorXd predicted_z = VectorXd(3);
    predicted_z << predicted_range,predicted_angle,predicted_range_rate;
    VectorXd y = z - predicted_z;
    
    y(1) = atan2(sin(y(1)), cos(y(1)));
    const double pi = 3.14159265358979323846;
    if (y(1) < -pi) {
        cout << "UpdateEKF() -- PHI < -3.14!!!!  PHI= " << y(1);
        y(1) = y(1) + 2*pi;
        cout << " .....Now it's: " << y(1) << endl;
        
    }
    else if (y(1) > pi) {
        cout << "PHI > 3.14!!!!  PHI= " << y(1);
        y(1) = y(1) - 2*pi;
        cout << " .....Now it's: " << y(1) << endl;
    }

    
    MatrixXd Ht = H_.transpose();
    MatrixXd PHt = P_ * Ht;
    MatrixXd S = H_ * PHt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = PHt * Si;
    
    //new estimate
    x_ = x_ + (K * y);
    cout<<"New Value px="<<x_(0)<<" py="<<x_(1)<<" vx="<<x_(2)<<" vy="<<x_(3)<<endl;
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
