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

  //initialize the state on the basis of a measurement value
  void initializeState(float z);

  //perform the update step of the VSLAM
  void updateVSLAM(double y);

  //perform the update step of the VSLAM
  void updateUltrasonic(double y);

  //update the scale and bias of vSLAM
  void updateScaleBiasSVO(float scale, float z0);

};

class Queue
{
  std::vector<float> bufferX;
  std::vector<float> bufferY;
  float var_sum;
  float sum;
  float std_dev_threshold;
  int max_buffer_size;
  int data_delay;               //if val is6 =>6*25ms = 150ms assuming sonar data comes after a delay of 150ms after vision data and sonar data is coming in at 40Hz

public:
  float mean;
  float std_dev;

public:
  //constructor for the queue
  Queue(float threshold, int delay_count, int max_buffer);

  //inserts elements into the queue
  void insertElement(float valx, float valy);

  //checks if the variance of the data meets a certain minimum threshold
  bool checkData();

  //returns the scale and constant of the vision estimate
  void calculateScale(float &scale, float &z_0);

  //clears the buffers and initializes the scale and bias to zero;
  void clear();
};

#endif /* EKF_H_ */
