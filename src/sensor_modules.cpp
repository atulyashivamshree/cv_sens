/*
 * sensor_modules.cpp
 *
 *  Created on: 21-Jan-2016
 *      Author: atulya
 */

#include "cv_sens/sensor_modules.h"

SensorIMU::SensorIMU()
{
  a_b = tf::Vector3(0.0, 0.0, 0.0);
  phi = 0;
  theta = 0;
  psi = 0;
}


SensorSonar::SensorSonar()
{
  flag_sonar_enabled = true;
  flag_sonar_diverged = false;
  flag_sonar_lost = false;

  depth = 0;
  sonar_diverged_time = 0;
}

void SensorSonar::initialize(float z, ros::Time device_stamp, ros::Time system_stamp)
{
  depth = z;
  last_sonar_stamp_device = device_stamp;
  last_sonar_stamp_system = system_stamp;
}

bool SensorSonar::isGlitching(float z)
{
  bool all_ok = true;

  if(z < 0.3 || z > 4.5)
    all_ok = false;

  return !all_ok;
}

void SensorSonar::processData(float z, ros::Time device_stamp, ros::Time system_stamp)
{
  depth = z;
  last_sonar_stamp_device = device_stamp;
  last_sonar_stamp_system = system_stamp;
}


SensorCV::SensorCV():ground_points(GROUND_PLANE_POINTS)
{
  flag_VSLAM_initiated = false;
  flag_VSLAM_reset_sent = false;
  flag_camera_rotation_corrected = false;
  flag_VSLAM_lost = false;

  position_vgnd = tf::Vector3(0.0f,0.0f,0.0f);
  phi_v = 0.0f;
  theta_v = 0.0f;
  psi_v = 0.0f;

  ground_points_index = 0;

  R_calib.setRPY(0.0f, 0.0f, 0.0f);
}

void SensorCV::resetCV()
{
  flag_camera_rotation_corrected = false;
  ground_points_index = 0;
}

void SensorCV::correctCameraRotation()
{
  //TODO right now the calibration is suspect to outliers, implement ransac to remove them
  printf("calibrating the camera :\n");
  MatrixXf A = MatrixXf::Zero(GROUND_PLANE_POINTS,3);
  MatrixXf b = MatrixXf::Zero(GROUND_PLANE_POINTS,1);
  Matrix<float, 3, 1> X;

  // File to store data in case of debugging (Remeber to close the file read)
//  std::string meas_filename = "/home/atulya/data/queue.csv";
//  std::ofstream outfile_meas;
//  outfile_meas.open(meas_filename.c_str());

  for(int i=0; i<GROUND_PLANE_POINTS; i++)
  {
    A(i,0) = ground_points[i](0);
    A(i,1) = ground_points[i](1);
    A(i,2) = 1;
    b(i,0) = -ground_points[i](2);
//    outfile_meas<<A(i,0)<<","<<A(i,1)<<","<<A(i,2)<<","<<b(i,0)<<"\n";
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

  printf("normal is [%.3f, %.3f, %.3f]", normal(0), normal(1), normal(2));
  printf("depth is %.3f\n", depth);
  printf("Roll detected: %.3f; Pitch detected: %.3f\n", phi, theta);
//  outfile_meas.close();
//  printf("R is \n[%.3f, %.3f, %.3f]\n[%.3f, %.3f, %.3f]\n[%.3f, %.3f, %.3f]",
//           R_calib.getRow(0).getX(), R_calib.getRow(0).getY(), R_calib.getRow(0).getZ(),
//           R_calib.getRow(1).getX(), R_calib.getRow(1).getY(), R_calib.getRow(1).getZ(),
//           R_calib.getRow(2).getX(), R_calib.getRow(2).getY(), R_calib.getRow(2).getZ());

}
