/*
 * ekf.h
 *
 *  Created on: Mar 6, 2015
 *      Author: atulya
 */

#include <Eigen/Dense>

#include <iostream>
#include <fstream>

#ifndef EKF_H_
#define EKF_H_

using namespace Eigen;

class IMU_CV_EKF
{
public:
  Matrix<double, 5, 1> x_hat_kplus1_kplus1;
private:
  Matrix<double, 5, 1> x_hat_kplus1_k;
  Matrix<double, 5, 1> x_hat_k_k;

  Matrix<double, 5, 5> P_kplus1_kplus1;
  Matrix<double, 5, 5> P_kplus1_k;
  Matrix<double, 5, 5> P_k_k;

  Matrix<double, 1, 1> Q_k;
  Matrix<double, 1, 1> W_v;
  Matrix<double, 1, 1> W_u;

  Matrix<double, 5, 5> A_k;
  Matrix<double, 5, 1> B_k;

  Matrix<double, 5, 5> F_k;
  Matrix<double, 5, 1> L_k;
  Matrix<double, 1, 5> H_v;
  Matrix<double, 1, 5> H_u;

  double dt;
  double g;

public:

  IMU_CV_EKF(double del_t);

  //perform the prediction step of the imu
  void prediction(double u);

  //initialize the state on the basis of a measurement value
  void initializeState(float z);

  //perform the update step of the VSLAM
  void updateVSLAM(double y);

  //perform the update step of the VSLAM
  void updateUltrasonic(double y);

  //update the scale and bias of vSLAM
  void updateScaleBiasSVO(float scale, float z0);

};
#endif /* EKF_H_ */
