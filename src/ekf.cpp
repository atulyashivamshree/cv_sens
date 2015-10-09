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

  ROS_INFO("ekf initialised \n");

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

void Queue::insertElement(float valx, float valy)
{
  if(bufferX.size() < max_buffer_size)
  {
    bufferX.push_back(valx);
    bufferY.push_back(valy);
    var_sum += pow(valy,2);
    sum += valy;
  }

  else
  {
    var_sum -= pow(bufferY[0],2);
    sum -= bufferY[0];

    for(int i=0; i<max_buffer_size-1; i++)
    {
      bufferX[i] = bufferX[i+1];
      bufferY[i] = bufferY[i+1];
    }

    bufferX[max_buffer_size-1] = valx;
    bufferY[max_buffer_size-1] = valy;

    var_sum += pow(valy,2);
    sum += valy;
  }
}

bool Queue::checkData()
{
  if(bufferX.size()==max_buffer_size)
  {
    mean = sum/max_buffer_size;
    float temp = var_sum/max_buffer_size - pow(mean,2);
    if(temp > 0)
    {
      std_dev = sqrt(temp);
      std::cout<<"Last 5 seconds buffer std_dev is:"<<std_dev<<"; (Must be 0.3 for VSLAM to proceed)";
//      std::cout<<"Buffer std_dev:"<<std_dev<<"sum_var is:"<<var_sum<<"buffer mean:"<<mean<<"; size:"<<bufferX.size()<<"; Data: ";
//      for(int i=0;i<bufferY.size(); i++)
//        std::cout<<bufferY[i]<<", ";
      std::cout<<"\n";
      if(std_dev > std_dev_threshold)
        return true;
      else
        return false;
    }
    else
    {
      bufferX.clear();
      bufferY.clear();
      return false;
    }
  }
  else
    return false;
}

void Queue::calculateScale(float &scale, float &z_0)
{
//  Matrix<float, max_buffer_size-data_delay, 2> A;
//  Matrix<float, max_buffer_size-data_delay, 1> b;
  MatrixXf A = MatrixXf::Zero(max_buffer_size-data_delay,2);
  MatrixXf b = MatrixXf::Zero(max_buffer_size-data_delay,1);
  Matrix<float, 2, 1> X;

//  std::string meas_filename = "../queue.csv";
//    std::ofstream outfile_meas;
//    outfile_meas.open(meas_filename.c_str());

  if(checkData() == false)
    return;

  for(int i=0; i<max_buffer_size-data_delay; i++)
  {
    A(i,0) = bufferX[i];
    A(i,1) = 1;
    b(i,0) = bufferY[(i+data_delay)%max_buffer_size];
//    outfile_meas<<bufferX[i]<<","<<b(i,0)<<"\n";
  }

//  std::cout<<A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
  X=A.colPivHouseholderQr().solve(b);
  scale = X(0,0);
  z_0 = X(1,0);
//  outfile_meas.close();
//  MatrixXf A = MatrixXf::Random(3, 2);
//  VectorXf b = VectorXf::Random(3);
//  std::cout << "The solution using the QR decomposition is:\n"
//       << A.colPivHouseholderQr().solve(b) << std::endl;
//  std::cout<<X;

}
void Queue::clear()
{
  bufferX.clear();
  bufferY.clear();
  var_sum = 0;
  sum = 0;
  mean = 0;
  std_dev =0;
}
Queue::Queue(float threshold, int delay_count, int max_buffer)
{
  std_dev_threshold = threshold;
  data_delay = delay_count;
  max_buffer_size = max_buffer;
}
