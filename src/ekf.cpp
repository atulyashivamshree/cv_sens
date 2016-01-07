/*
 * ekf.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: atulya
 */
#include "ekf.h"

IMU_CV_EKF::IMU_CV_EKF(double del_t)
{
  dt = del_t;

  x_hat_k_k << 1,0,-0.2, 1, 1;

  P_k_k = MatrixXd::Identity(5,5);
  P_k_k(0,0) = 2;
  P_k_k(2,2) = 0.3;
  P_k_k(3,3) = 4;
  P_k_k(4,4) = 0.4;

  x_hat_kplus1_k = x_hat_k_k;
  P_kplus1_k = P_k_k;
  x_hat_kplus1_kplus1 = x_hat_k_k;
  P_kplus1_kplus1 = P_k_k;

  V_k = MatrixXd::Identity(5,5);
  V_k(0,0) = 0.3*0.3*dt;
  V_k(1,1) = 0.6*0.6*dt;
  V_k(2,2) = 0.04*0.04*dt;
  V_k(3,3) = 0.003*dt*dt;
  V_k(4,4) = 0.001*dt*dt;

  W_v << 0.1;
  W_u << 0.02;

  g = 9.81;

  A_k = MatrixXd::Zero(5,5);
  A_k(0,1) = 1;
  A_k(1,2) = 1;

  F_k = MatrixXd::Identity(5,5) + A_k*dt;

  B_k = MatrixXd::Zero(5,1);
  B_k(1,0) = dt;

  H_u = MatrixXd::Zero(1,5);
  H_v = MatrixXd::Zero(1,5);
  H_u(0,0) = 1;

  std::cout<<"ekf initialised\n";

}

void IMU_CV_EKF::initializeState(float z)
{
    x_hat_k_k << z,0,-0.2, 1, 1;

    P_k_k = MatrixXd::Identity(5,5);
    P_k_k(0,0) = 2;
    P_k_k(2,2) = 0.3;
    P_k_k(3,3) = 4;
    P_k_k(4,4) = 0.4;

    x_hat_kplus1_k = x_hat_k_k;
    P_kplus1_k = P_k_k;
    x_hat_kplus1_kplus1 = x_hat_k_k;
    P_kplus1_kplus1 = P_k_k;

}

void IMU_CV_EKF::prediction(double u)
{
  x_hat_k_k = x_hat_kplus1_kplus1;
  P_k_k = P_kplus1_kplus1;

  x_hat_kplus1_k = F_k*x_hat_k_k + B_k*u;
  P_kplus1_k = F_k*P_k_k*F_k.transpose() + V_k;

  x_hat_kplus1_kplus1 = x_hat_kplus1_k;
  P_kplus1_kplus1 = P_kplus1_k;

//  ROS_INFO("prediction %f", x_hat_kplus1_k(0,0));
//  std::cout<<"state is"<<x_hat_kplus1_k;
}
void IMU_CV_EKF::updateVSLAM(double z_v)
{
  double z_hat_kplus1 = 1/x_hat_kplus1_k(3,0)*(x_hat_kplus1_k(0,0) - x_hat_kplus1_k(4,0));

  double error = z_v - z_hat_kplus1;
  H_v << 1/x_hat_kplus1_k(3,0), 0, 0,
      (-1/(x_hat_kplus1_k(3,0)*x_hat_kplus1_k(3,0)) *(x_hat_kplus1_k(0,0) - x_hat_kplus1_k(4,0))),
      -1/x_hat_kplus1_k(3,0);
  MatrixXd S = H_v*P_kplus1_k*(H_v.transpose()) + W_v;
  MatrixXd R = P_kplus1_k*H_v.transpose()*S.inverse();

  x_hat_kplus1_kplus1 = x_hat_kplus1_k + R*error;
  P_kplus1_kplus1 = P_kplus1_k - R*H_v*P_kplus1_k;

//  std::cout<<"state is"<<x_hat_kplus1_k;
//  ROS_INFO("z_v is %f, a_hat_kplus1 is %f,  err is %f and S is %f",z_v, z_hat_kplus1, error, S(0,0));
//  std::cout<<"H_v is"<<H_v<<"R vslam is "<<R<<"\n";

}

void IMU_CV_EKF::updateUltrasonic(double z_u)
{
  double z_hat_kplus1 = H_u*x_hat_kplus1_k;

  double error = z_u - z_hat_kplus1;
  MatrixXd S = H_u*P_kplus1_k*(H_u.transpose()) + W_u;
  MatrixXd R = P_kplus1_k*H_u.transpose()*S.inverse();

  x_hat_kplus1_kplus1 = x_hat_kplus1_k + R*error;
  P_kplus1_kplus1 = P_kplus1_k - R*H_u*P_kplus1_k;

//  ROS_INFO("updating ultrasonics filter with z as %f ,z_u %f and S as%f", z_hat_kplus1, z_u, S(0,0));

}

void IMU_CV_EKF::updateScaleBiasSVO(float scale, float z0)
{
  x_hat_kplus1_k(3,0) = scale;
  x_hat_kplus1_k(4,0) = z0;
  x_hat_kplus1_kplus1(3,0) = scale;
  x_hat_kplus1_kplus1(4,0) = z0;
}
