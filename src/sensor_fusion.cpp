/*
 * camera_calib.cpp
 *
 *  Created on: 18-Nov-2015
 *      Author: atulya
 */

#include "sensor_fusion.h"

Queue::Queue(float threshold, int delay_count, int max_buffer)
{
  std_dev_threshold = threshold;
  data_delay = delay_count;
  max_buffer_size = max_buffer;
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
      std::cout<<"Last 5 seconds buffer std_dev is:"<<std_dev<<"; (Must be "<<std_dev_threshold<<" for VSLAM to proceed)";
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

SensorSonar::SensorSonar()
{
  flag_sonar_enabled = true;
  flag_sonar_initialized = false;
  flag_sonar_diverged = false;
  flag_sonar_lost = false;

  depth = 0;
  sonar_diverged_time = 0;
}

void SensorSonar::initialize(float z, ros::Time stamp)
{
  flag_sonar_initialized = true;
  last_sonar_stamp = stamp;
  depth = z;
}

bool SensorSonar::isGlitching(float z)
{
  bool all_ok = true;

  if(z < 0.3 || z > 4.5)
    all_ok = false;

  return !all_ok;
}

void SensorSonar::processData(float z, ros::Time stamp)
{
  depth = z;
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

SensorFusion::SensorFusion():ekf_z(1/IMU_FREQUENCY),
                            cv_sonar_correspondance(STD_DEV_THRESHOLD, DATA_DELAY, MAX_BUFFER_SIZE)
{
  position = tf::Vector3(0.0f,0.0f,0.0f);
  phi = 0.0f;
  theta = 0.0f;
  psi = 0.0f;

  flag_lamda_initialized = false;

  R_HBF_to_GND.setRPY(0.0f, 0.0f, 0.0f);
  R_b_to_GND.setRPY(0.0f, 0.0f, 0.0f);
  R_b_to_HBF.setRPY(0.0f, 0.0f, 0.0f);
  R_b_to_VGND.setRPY(0.0f, 0.0f, 0.0f);

  sensor_health.baro = UNINITIALIZED;
  sensor_health.cv = UNINITIALIZED;
  sensor_health.imu = UNINITIALIZED;
  sensor_health.sonar = UNINITIALIZED;
}

bool SensorFusion::isSonarDataOutlier(double z)
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

void SensorFusion::initializeSonar(double z, ros::Time stamp)
{
  sonar.initialize(z, stamp);
  ekf_z.initializeState(z);
}

void SensorFusion::updateIMUData(const sensor_msgs::Imu::ConstPtr& msg)
{
  geometry_msgs::Quaternion q_ = msg->orientation;
  tf::Quaternion q;
  tf::quaternionMsgToTF(q_, q);

  R_b_to_GND.setRotation(q);

  double psi_temp;

  R_b_to_GND.getRPY(phi, theta, psi_temp);

//  R_HBF_to_GND.setRPY(0, 0, psi);
  R_b_to_HBF.setRPY(phi, theta, 0);

  tf::Vector3 a_b(msg->linear_acceleration.x,
                  msg->linear_acceleration.y,
                  msg->linear_acceleration.z);

  tf::Vector3 g(0,0,-GRAVITY_MSS);      //the gravity vector

  accel_hbf = R_b_to_HBF*a_b + g;       //acceleration as obtained in the horizontal body frame and in NED coordinate system

  //IMU based prediction will work only as long as estimate is within acceptable limits of SONAR
  if(ekf_z.x_hat_kplus1_kplus1(0,0) > 0.3 && ekf_z.x_hat_kplus1_kplus1(0,0) < 4.5 )             // the absolute lower limit possible for ultrasonic sensors
    ekf_z.prediction(accel_hbf.getZ());

//  std::cout<<"EKF z predicted is"<<ekf_z.x_hat_kplus1_kplus1(0,0)<<"\n";

  tf::vector3TFToMsg(accel_hbf, acc_msg_hbf.vector);
  acc_msg_hbf.vector.x = z_hat_msg.vector.z;
  acc_msg_hbf.header.stamp = msg->header.stamp;

  acc_pub_hbf.publish(acc_msg_hbf);
}

bool SensorFusion::isVSLAMEstimationHealthy()
{
  bool all_ok = true;
  if(flag_lamda_initialized == false)
    all_ok = false;

  if(cv_slam.flag_camera_rotation_corrected == false)
    all_ok = false;

  return (!all_ok);
}

void SensorFusion::resetVSLAM()
{
  cv_sonar_correspondance.clear();
  flag_lamda_initialized = false;
  cv_slam.resetCV();

  position.setZero();
  phi = 0;
  theta = 0;
  psi = 0;

  ROS_INFO("Visual reasings lost : lambda and bias will be reinitialized once VSLAM is recovered ");
}

void SensorFusion::updateSonarData(double z, ros::Time stamp)
{
  //If sonar has not been initialized
  if(sonar.flag_sonar_initialized == false)
  {
    if(z!=0)
    {
      sonar.initialize(z, stamp);
    }
  }

  //Check if sonar data is an outlier
  if(isSonarDataOutlier(z))
  {
    rolling_q[ROLLING_FILTER_WINDOW-2] = ekf_z.x_hat_kplus1_kplus1(0,0);
    ros::Duration delt = stamp - sonar.last_sonar_stamp;
    sonar.sonar_diverged_time = delt.toSec();

    if(sonar.sonar_diverged_time >= SONAR_LOSS_TIME_THRESHOLD)
    {
      sonar.flag_sonar_initialized = false;
      sonar.flag_sonar_diverged = true;
      sonar.sonar_diverged_time = 0.0f;
    }
//    std::cout<<"outlier detected";
  }
  else
  {
    if(sonar.flag_sonar_enabled)
    {
      rolling_q[ROLLING_FILTER_WINDOW-2] = z;
      ekf_z.updateUltrasonic(z);
      //finding out the scale and bias factor for the vision algorithm
      if(flag_lamda_initialized == false)               //TODO add a check to insert into queue only if timestamps are close
      {
        if(cv_slam.position_vgnd.getZ() != 0)
        {
          cv_sonar_correspondance.insertElement(cv_slam.position_vgnd.getZ(), z);
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

  //set z coordinate of the estimated state to the ekf computed state
  position.setZ(ekf_z.x_hat_kplus1_kplus1(0,0));
}

void SensorFusion::updateVSLAMData(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  cv_slam.last_cv_stamp = msg->header.stamp;
  tf::Vector3 p(msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z);

  //correct the position according to the corrected camera rotaion matrix
  cv_slam.position_vgnd = cv_slam.R_calib.transpose()*p;

  //Read the values of euler angles from vslam
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  R_b_to_VGND.setRotation(q);
  R_b_to_VGND.getRPY(cv_slam.phi_v, cv_slam.theta_v, cv_slam.psi_v);

//  std::cout<<"psi_v i "<<psi_v<<"\n";
  R_HBF_to_GND.setRPY(0, 0, cv_slam.psi_v);

  //update the vslam algorithm according to the inertia matrix
//  if(flag_lamda_initialized)
//    ekf_z.updateVSLAM(position_vgnd.getZ());
  pos_pub_vgnd.publish(pos_msg_vgnd);

  //assign final psi and position according to the camera data
  psi = cv_slam.psi_v;
  position.setX(ekf_z.x_hat_kplus1_kplus1(3,0)*cv_slam.position_vgnd.getX());
  position.setY(ekf_z.x_hat_kplus1_kplus1(3,0)*cv_slam.position_vgnd.getY());
}

void SensorFusion::updateVSLAM_MapPointsRotationCorrection(const visualization_msgs::Marker::ConstPtr& msg)
{
  if(msg->ns.compare("pts") == 0 && cv_slam.flag_camera_rotation_corrected == false)
   {
     if(msg->pose.position.z == 0)
       return;
     cv_slam.ground_points[cv_slam.ground_points_index](0) = msg->pose.position.x;
     cv_slam.ground_points[cv_slam.ground_points_index](1) = msg->pose.position.y;
     cv_slam.ground_points[cv_slam.ground_points_index](2) = msg->pose.position.z;
     cv_slam.ground_points_index++;

     //if sufficient number of points have been acquired correct the map
     if(cv_slam.ground_points_index == GROUND_PLANE_POINTS)
     {
       cv_slam.correctCameraRotation();
       cv_slam.flag_camera_rotation_corrected = true;
     }
   }

  // if rotation has been corrected publish new points in the rotation corrected frame
   if(cv_slam.flag_camera_rotation_corrected == true)
   {
     tf::Vector3 p(msg->pose.position.x,
                   msg->pose.position.y,
                   msg->pose.position.z);

     tf::Vector3 p_calib = cv_slam.R_calib.transpose()*p;
   //  tf::Vector3 p_calib = p;

     map_points_msg.header.frame_id = "/world";
     map_points_msg.header.stamp = msg->header.stamp;
     map_points_msg.ns = "calib_"+msg->ns;
     map_points_msg.id = msg->id;
     map_points_msg.type = visualization_msgs::Marker::CUBE;
     map_points_msg.action = msg->action; // 0 = add/modify
     map_points_msg.scale.x = msg->scale.x;
     map_points_msg.scale.y = msg->scale.y;
     map_points_msg.scale.z = msg->scale.z;
     map_points_msg.color.a = 1.0;
     map_points_msg.color.r = 0;
     map_points_msg.color.g = 1;
     map_points_msg.color.b = 0;
     map_points_msg.lifetime = msg->lifetime;
     map_points_msg.pose.position.x = p_calib.getX()+1;
     map_points_msg.pose.position.y = p_calib.getY()+1;
     map_points_msg.pose.position.z = p_calib.getZ();
     map_points_pub.publish(map_points_msg);

     //  ROS_INFO("NS of the point is %s and index is %d", msg->ns.c_str(), ground_points_index);
   }
}

//Monitors the health of all sensors
void SensorFusion::monitorSensorHealth()
{
  /*check when was the last vision based reading obtained; if(greater than a threshold)
       * then we have to reinitalize the scale ,bias and plane calibration
       */
  ros::Duration delt = ros::Time::now() - cv_slam.last_cv_stamp;
  if(delt.toSec()>VISION_BREAKSIGNAL_THRESHOLD)
  {
    cv_slam.resetCV();
    sensor_health.cv = LOST;
  }
}

//Function for publishing some standard debug messages
void SensorFusion::publishDebugMessages(ros::Time stamp)
{
  z_static_states_msg.header.stamp = stamp;
  z_static_states_msg.vector.x = ekf_z.x_hat_kplus1_kplus1(3,0);
  z_static_states_msg.vector.y = ekf_z.x_hat_kplus1_kplus1(4,0);
  z_static_states_msg.vector.z = ekf_z.x_hat_kplus1_kplus1(5,0);

  z_hat_msg.header.stamp = stamp;
  z_hat_msg.vector.x = ekf_z.x_hat_kplus1_kplus1(0,0);
  z_hat_msg.vector.y = ekf_z.x_hat_kplus1_kplus1(3,0)*cv_slam.position_vgnd.getZ() + ekf_z.x_hat_kplus1_kplus1(4,0);
  z_hat_msg.vector.z = sonar.depth;

  //position in the vslam inertial frame used for debugging data
  pos_msg_vgnd.header.stamp = stamp;
  pos_msg_vgnd.vector.x = cv_slam.position_vgnd.getX();
  pos_msg_vgnd.vector.y = cv_slam.position_vgnd.getY();
  pos_msg_vgnd.vector.z = cv_slam.position_vgnd.getZ();

  z_hat_pub.publish(z_hat_msg);
  pos_pub_vgnd.publish(pos_msg_vgnd);
  z_static_states_pub.publish(z_static_states_msg);
}
