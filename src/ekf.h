/*
 * ekf.h
 *
 *  Created on: Mar 6, 2015
 *      Author: atulya
 */

#include <Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

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

  Matrix<double, 5, 5> V_k;
  Matrix<double, 1, 1> W_v;
  Matrix<double, 1, 1> W_u;

  Matrix<double, 5, 5> A_k;
  Matrix<double, 5, 1> B_k;

  Matrix<double, 5, 5> F_k;
  Matrix<double, 1, 5> H_v;
  Matrix<double, 1, 5> H_u;

  double dt;
  double g;

public:

  IMU_CV_EKF(double del_t);

  //perform the prediction step of the imu
  void prediction(double u);

  //perform the update step of the VSLAM
  void updateVSLAM(double y);

  //perform the update step of the VSLAM
  void updateUltrasonic(double y);

};

#endif /* EKF_H_ */
