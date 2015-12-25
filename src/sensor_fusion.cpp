/*
 * camera_calib.cpp
 *
 *  Created on: 18-Nov-2015
 *      Author: atulya
 */

#include "sensor_fusion.h"

double phi, theta, psi;
bool flag_sonar_enabled = true;
bool flag_sonar_initialized = false;
bool flag_lamda_initialized = false;
int sonar_lost_count = 0;

float chnl_switching = 0;
float chnl_original;
float chnl_switching_cutoff_freq = 0.6;
float chnl_switching_dt = 1/35.0f;

ros::Time last_cv_stamp;

Queue cv_sonar_correspondance(STD_DEV_THRESHOLD, DATA_DELAY, MAX_BUFFER_SIZE);

tf::Vector3 position_vgnd;

tf::Matrix3x3 R_HBF_to_GND;
tf::Matrix3x3 R_b_to_HBF;
tf::Matrix3x3 R_b_to_GND;
tf::Matrix3x3 R_b_to_VGND;
tf::Matrix3x3 R_vb_vgnd_t;
tf::Matrix3x3 R_calib;
double phi_v, theta_v, psi_v;

tf::Vector3 accel_hbf;

geometry_msgs::Vector3Stamped pos_msg_vgnd;
ros::Publisher pos_pub_vgnd;

geometry_msgs::Vector3Stamped pos_msg_vb;
ros::Publisher pos_pub_vb;

geometry_msgs::Vector3Stamped acc_msg_hbf;
ros::Publisher acc_pub_hbf;

geometry_msgs::Vector3Stamped z_hat_msg;
ros::Publisher z_hat_pub;

geometry_msgs::PoseStamped cv_pose_msg;
ros::Publisher cv_pose_pub;

std_msgs::String key_msg;
ros::Publisher svo_remote_key_pub;

geometry_msgs::Vector3 waypoint_msg_nedb;
ros::Publisher waypoint_pub;

visualization_msgs::Marker points_msg;
ros::Publisher points_pub;

bool SVO_INITIATED = false;
bool SVO_RESET_SENT = false;
bool FLAG_CAMERA_CALIBRATED = false;

IMU_CV_EKF ekf_z(1/IMU_FREQUENCY);
geometry_msgs::Vector3Stamped z_static_states_msg;
ros::Publisher z_static_states_pub;

std::vector<Vector3f> ground_points(GROUND_PLANE_POINTS);
int ground_points_index;
Vector3f normal;
float depth;

std::vector<double> rolling_q(ROLLING_FILTER_WINDOW, DEFAULT_HEIGHT);
int start, end;

void calibrateCamera()
{
  //TODO right now the calibration is suspect to outliers, implement ransac to remove them
  printf("calibrating the camera :\n");
  MatrixXf A = MatrixXf::Zero(GROUND_PLANE_POINTS,3);
  MatrixXf b = MatrixXf::Zero(GROUND_PLANE_POINTS,1);
  Matrix<float, 3, 1> X;

  std::string meas_filename = "/home/atulya/data/queue.csv";
  std::ofstream outfile_meas;
  outfile_meas.open(meas_filename.c_str());

  for(int i=0; i<GROUND_PLANE_POINTS; i++)
  {
    A(i,0) = ground_points[i](0);
    A(i,1) = ground_points[i](1);
    A(i,2) = 1;
    b(i,0) = -ground_points[i](2);
    outfile_meas<<A(i,0)<<","<<A(i,1)<<","<<A(i,2)<<","<<b(i,0)<<"\n";
  }

  //  std::cout<<A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
  X=A.colPivHouseholderQr().solve(b);
  normal(0) = X(0,0);
  normal(1) = X(1,0);
  normal(2) = 1;
  depth = -X(2,0);

  float theta = atan2(normal(0), normal(2));
  float phi = atan2(-normal(1), sqrt(pow(normal(0),2)+ pow(normal(2),2)));

  R_calib.setRPY(phi,theta, 0);

  printf("normal is [%f, %f, %f]", normal(0), normal(1), normal(2));
  printf("depth is %f", depth);
  ROS_INFO("Roll detected: %f; Pitch detected: %f", phi, theta);
  outfile_meas.close();
  ROS_INFO("R is \n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
           R_calib.getRow(0).getX(), R_calib.getRow(0).getY(), R_calib.getRow(0).getZ(),
           R_calib.getRow(1).getX(), R_calib.getRow(1).getY(), R_calib.getRow(1).getZ(),
           R_calib.getRow(2).getX(), R_calib.getRow(2).getY(), R_calib.getRow(2).getZ());

}

bool isOutlier(double z)
{
  if(z == 0)
    return true;

  rolling_q[ROLLING_FILTER_WINDOW-1] = z;

  std::vector<double> sorted_q = rolling_q;
  std::sort(sorted_q.begin(), sorted_q.end());

  for(int i = 0; i<rolling_q.size()-1; i++)
    rolling_q[i] = rolling_q[i+1];

//  std::cout<<"rolled queue is\n";
//  for(int i = 0; i<rolling_q.size()-1; i++)
//      std::cout<<rolling_q[i]<<", ";
//  std::cout<<"\n";

  double median = sorted_q[ROLLING_FILTER_WINDOW/2];

//  std::cout<<"front is "<<sorted_q[0]<<"end-2 is "<<sorted_q[ROLLING_FILTER_WINDOW-3]<<"median is" <<median<<"z is "<<z<<"diff is "<<fabs(z-median)<<"\n";

  if(fabs(z-median) > SONAR_DELTA_READING_MAX)
    return true;
  else
    return false;

}

void initializeSONAR(float z)
{
  ekf_z.initializeState(z);
}

void resetCV()
{
  ground_points_index = 0;

  cv_sonar_correspondance.clear();
  flag_lamda_initialized = false;
  FLAG_CAMERA_CALIBRATED = false;
  position_vgnd.setZero();

  ROS_INFO("Visual reasings lost : lambda and bias will be reinitialized once VSLAM is recovered ");
}

void processSonarData(float z)
{
  if(flag_sonar_initialized == false)
  {
    if(z!=0)
    {
      initializeSONAR(z);
      flag_sonar_initialized = true;
    }
  }

  if(isOutlier(z))
  {
    rolling_q[ROLLING_FILTER_WINDOW-2] = ekf_z.x_hat_kplus1_kplus1(0,0);
    sonar_lost_count++;
    if(sonar_lost_count >= SONAR_LOSS_COUNT_THRESHOLD)
    {
      flag_sonar_initialized = false;
      sonar_lost_count = 0;
    }
//    std::cout<<"outlier detected";
  }
  else
  {
    if(flag_sonar_enabled)
    {
      rolling_q[ROLLING_FILTER_WINDOW-2] = z;
      ekf_z.updateUltrasonic(z);
      //finding out the scale and bias factor for the vision algorithm
      if(flag_lamda_initialized == false)               //TODO add a check to insert into queue only if timestamps are close
      {
        if(position_vgnd.getZ() != 0)
        {
          cv_sonar_correspondance.insertElement(position_vgnd.getZ(), z);
          if(cv_sonar_correspondance.checkData() == true)
          {
            float scale, z0;
            cv_sonar_correspondance.calculateScale(scale, z0);
            ekf_z.updateScaleBiasSVO(scale, z0);
            ROS_INFO("Scale: %f and bias:%f for VSLAM initialised", scale, z0);
            flag_lamda_initialized = true;
          }
        }
      }
    }
    else
      rolling_q[ROLLING_FILTER_WINDOW-2] = ekf_z.x_hat_kplus1_kplus1(0,0);
  }

}
