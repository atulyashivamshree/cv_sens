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

bool Queue::checkCompletion()
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

  if(checkCompletion() == false)
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
RollingQueue::RollingQueue(int length, float default_val ) : rolling_q(length, DEFAULT_HEIGHT)
{
  size = length;
  start = 0;
  end = 0;
  default_value = default_val;
}

void RollingQueue::printQueue()
{
  printf("Queue end is at %d and elements are ", end);
  for(int i=0; i<size; i++)
    printf("%.2f, ", rolling_q[i]);

  printf("\n");
}

void RollingQueue::reset()
{
  start = 0;
  end = 0;
  is_full = false;

  for(int i = 0; i<size; i++)
    rolling_q[i] = default_value;
}

void RollingQueue::insert(float z)
{
  if(is_full == true)
  {
    //TODO make it rolling so as to increase the window
    for(int i=0; i<size-1; i++)
      rolling_q[i] = rolling_q[i+1];
    rolling_q[end] = z;
  }

  //if queue is not full first put elements into it
  if(end < size && is_full == false)
  {
    rolling_q[end] = z;
    if(end == size-1)
      is_full = true;
    else
      end++;
  }
}

bool RollingQueue::isOutlierWithMedianFilter(float input, float threshold)
{

  std::vector<double> sorted_q = rolling_q;
  std::sort(sorted_q.begin(), sorted_q.end());

  double median = sorted_q[ROLLING_FILTER_WINDOW/2];

  //debugging step
//  std::cout<<"front is "<<sorted_q[0]<<"end-1 is "<<sorted_q[size-2]<<"median is" <<median<<"z is "<<input<<"diff is "<<fabs(input-median)<<"\n";

  if(fabs(input - median) > threshold)
    return true;
  else
    return false;

}

SensorFusion::SensorFusion():ekf_z(1/IMU_FREQUENCY),
                            cv_sonar_correspondance(STD_DEV_THRESHOLD, DATA_DELAY, MAX_BUFFER_SIZE),
                            sonar_queue(ROLLING_FILTER_WINDOW, DEFAULT_HEIGHT)
{
  position = tf::Vector3(0.0f,0.0f,0.0f);
  phi = 0.0f;
  theta = 0.0f;
  psi = 0.0f;

  flags.lambda_initialized = false;
  flags.ekf_prediction_enabled = false;

  R_HBF_to_GND.setRPY(0.0f, 0.0f, 0.0f);
  R_b_to_GND.setRPY(0.0f, 0.0f, 0.0f);
  R_b_to_HBF.setRPY(0.0f, 0.0f, 0.0f);
  R_b_to_VGND.setRPY(0.0f, 0.0f, 0.0f);

  sensor_health.baro = UNINITIALIZED;
  sensor_health.cv = UNINITIALIZED;
  sensor_health.imu = UNINITIALIZED;
  sensor_health.sonar = UNINITIALIZED;

  //TODO cross check if all variables have been initialized (also for other classes)

  ROS_INFO("Sensor Fusion instantiated");
}

inline std::string getBoolStatus(bool status)
{
  std::string str;

  if(status)
    str = "OK";
  else
    str = "FALSE";

  return str;
}

inline std::string getSensorStatus(sensor_status_t status)
{
  std::string str;
  switch(status)
  {
    case UNINITIALIZED:
      str = "UNINITIALIZED";
      break;
    case ALIVE:
      str = "ALIVE";
      break;
    case LOST:
      str = "LOST";
      break;
    case DIVERGED:
      str = "DIVERGED";
      break;
    default:
      str = "RANDOM";
      break;
  }
  return str;
}
void SensorFusion::updateIMUData(const sensor_msgs::Imu::ConstPtr& msg)
{

  if(sensor_health.imu == UNINITIALIZED || sensor_health.imu == LOST)
    sensor_health.imu = ALIVE;

  geometry_msgs::Quaternion q_ = msg->orientation;
  tf::Quaternion q;
  tf::quaternionMsgToTF(q_, q);

  R_b_to_GND.setRotation(q);

  double psi_temp;

  R_b_to_GND.getRPY(phi, theta, psi_temp);
  imu.phi = phi;
  imu.theta = theta;
  imu.psi = psi_temp;
  imu.last_imu_stamp = msg->header.stamp;

//  R_HBF_to_GND.setRPY(0, 0, psi);
  R_b_to_HBF.setRPY(phi, theta, 0);

  imu.a_b = tf::Vector3(msg->linear_acceleration.x,
                  msg->linear_acceleration.y,
                  msg->linear_acceleration.z);

  tf::Vector3 g(0,0,-GRAVITY_MSS);      //the gravity vector

  accel_hbf = R_b_to_HBF*imu.a_b + g;       //acceleration as obtained in the horizontal body frame and in NED coordinate system

  if(ekf_z.x_hat_kplus1_kplus1(0,0) > 0.3 && ekf_z.x_hat_kplus1_kplus1(0,0) < 4.5 )             // the absolute lower limit possible for ultrasonic sensors
  {
    if(flags.ekf_prediction_enabled == true)
    {
      //IMU based prediction will work only as long as estimate is within acceptable limits of SONAR
      ekf_z.prediction(accel_hbf.getZ());
    }
  }

//  std::cout<<"EKF z predicted is"<<ekf_z.x_hat_kplus1_kplus1(0,0)<<"\n";

}

void SensorFusion::initializeSonar(double z, ros::Time stamp)
{
  ROS_DEBUG("initializing sonar with z %f", z);

  ros::Time t_now = ros::Time::now();
  sonar.initialize(z, stamp, t_now);

  if(sonar_queue.isInitialized() == false)
  {
    // for initialization it has been assumed that sensor readings are below 1m for at least 1sec
    if(z < SONAR_INITIALIZATION_MAX_HEIGHT)
      sonar_queue.insert(z);
  }
  else
  {
    sensor_health.sonar = ALIVE;
    flags.ekf_prediction_enabled = true;
    ekf_z.initializeState(z);
    ROS_INFO("Sonar initialized");
  }
}

bool SensorFusion::isSonarDataOutlier(double z)
{
  if(sonar.isGlitching(z) == true)
  {
//    ROS_DEBUG("Outlier Detected Method1 %f", z);
    return true;
  }

  if(sonar_queue.isOutlierWithMedianFilter(z, SONAR_DELTA_READING_MAX) == true)
  {
//    ROS_DEBUG("Outlier Detected Method2 %f", z);
    return true;
  }

  return false;

}


void SensorFusion::resetSonar()
{
//  TODO cross check with a working case
  sonar_queue.reset();
  flags.ekf_prediction_enabled = false;
  ROS_DEBUG("disabling ekf prediction sonar");
  sensor_health.sonar = UNINITIALIZED;
}

void SensorFusion::updateSonarData(double z, ros::Time stamp)
{
  ros::Time t_now = ros::Time::now();

  //Store data from available sensors into structures
  sonar.processData(z, stamp, t_now);

  //debugging step
//  sonar_queue.printQueue();

  //If sonar has not been initialized
  if(sensor_health.sonar == UNINITIALIZED)
  {
    ROS_DEBUG("health of sonar is %s", getSensorStatus(sensor_health.sonar).c_str());
    if(isSonarDataOutlier(z)==false)
    {
      //sonar will take some predefined number of values before reaching to the state of initialization
      initializeSonar(z, stamp);
    }
    else
    {
      ROS_DEBUG("Initialization failed because of glitched data : Try to hold the sonar at approx 0.5m from an obstacle constantly for 1sec");
      resetSonar();
    }

    return;
  }

  //Check if sonar data is an outlier
  if(isSonarDataOutlier(z))
  {
    sonar_queue.insert(ekf_z.x_hat_kplus1_kplus1(0,0));

    //if outliers are detected for a long period of time mark the sonar as diverged
    ros::Duration delt = stamp - last_good_sonar_stamp;
    sonar.sonar_diverged_time = delt.toSec();
//    ROS_DEBUG("SONAR divering time is %f", sonar.sonar_diverged_time);

    //Take some action if sonar has diverged
    if(sonar.sonar_diverged_time >= SONAR_LOSS_TIME_THRESHOLD)
    {
      ROS_ERROR("Sonar data has diverged : Abort auto mode immediately ");
      sensor_health.sonar = DIVERGED;
      sonar.flag_sonar_diverged = true;
      sonar.sonar_diverged_time = 0.0f;
      resetSonar();
    }
//    std::cout<<"outlier detected";
  }
  else
  {
//    ROS_DEBUG("using Sonar data");

    //sonar data is good so insert it into the rolling queue and also update the EKF
    last_good_sonar_stamp = stamp;
    sonar_queue.insert(z);
    ekf_z.updateUltrasonic(z);

    //========================COMPUTING SCALE AND BIAS ==========================
    if(sensor_health.cv == ALIVE && flags.lambda_initialized == false)
    {
      ros::Duration delt;
      delt = t_now - cv_slam.last_cv_stamp;
      ROS_DEBUG("Sonar is %u, cv is %u, diff is %f",t_now.sec, cv_slam.last_cv_stamp.sec, delt.toSec());

      if(cv_slam.flag_camera_rotation_corrected == true
          && delt.toSec() < 0.4f)
      {
        if(cv_slam.position_vgnd.getZ() != 0)
        {
          //insert corresponding values of sonar and vision in a queue
          cv_sonar_correspondance.insertElement(cv_slam.position_vgnd.getZ(), z);

          //check whether sufficient data points have been gathered and if true compute the bias and scale
          if(cv_sonar_correspondance.checkCompletion() == true)
          {
            float scale, z0;
            cv_sonar_correspondance.calculateScale(scale, z0);
            ekf_z.updateScaleBiasSVO(scale, z0);

            ROS_INFO("Scale: %f and bias:%f for VSLAM initialised", scale, z0);
            ROS_INFO("VSLAM is now Healthy");
            flags.lambda_initialized = true;
          }
        }
      }
    }

    //-----------------------------------------------------------------------------
  }

  //set z coordinate of the estimated state to the ekf computed state
  position.setZ(ekf_z.x_hat_kplus1_kplus1(0,0));
}

void SensorFusion::resetVSLAM()
{
  cv_sonar_correspondance.clear();
  flags.lambda_initialized = false;
  cv_slam.resetCV();

  position.setZero();
  phi = 0;
  theta = 0;
  psi = 0;

  ROS_WARN("Visual readings lost : lambda and bias will be reinitialized once VSLAM is recovered ");
}


void SensorFusion::updateVSLAMData(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  if(sensor_health.cv == UNINITIALIZED)
    sensor_health.cv = ALIVE;

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
//  if(flags.lambda_initialized)
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
       ROS_INFO("CV SLAM Rotation Corrected");
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

bool SensorFusion::isVSLAMEstimationHealthy()
{
  bool all_ok = true;
  if(flags.lambda_initialized == false)
    all_ok = false;

  if(cv_slam.flag_camera_rotation_corrected == false)
    all_ok = false;

  if(isAltitudeHealthy() == false)
    all_ok = false;

  return all_ok;
}

bool SensorFusion::isAltitudeHealthy()
{
  bool all_ok = true;

  if(sensor_health.sonar != ALIVE)
    all_ok = false;

  if(sensor_health.imu != ALIVE)
    all_ok = false;

  return all_ok;
}

//Monitors the health of all sensors
void SensorFusion::monitorSensorHealth(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
  ros::Time t_now = ros::Time::now();
  ros::Duration delt_imu, delt_sonar, delt_cv;

  delt_imu = t_now - imu.last_imu_stamp;

  if(delt_imu.toSec()>IMU_LOSS_THRESHOLD && sensor_health.imu != UNINITIALIZED)
  {
    sensor_health.imu = LOST;
  }


  delt_sonar = t_now - sonar.last_sonar_stamp_system;
  if(delt_sonar.toSec()>SONAR_LOSS_TIME_THRESHOLD && sensor_health.sonar != UNINITIALIZED)
  {
    sensor_health.sonar = LOST;
  }

  /*check when was the last vision based reading obtained; if(greater than a threshold)
       * then we have to reinitalize the scale ,bias and plane calibration
       */
  delt_cv = t_now - cv_slam.last_cv_stamp;
  if(delt_cv.toSec()>VISION_BREAKSIGNAL_THRESHOLD && sensor_health.cv != UNINITIALIZED)
  {
    cv_slam.resetCV();
    flags.lambda_initialized = false;
    sensor_health.cv = LOST;
  }

//  ROS_DEBUG("SENSOR time is %u", sonar.last_sonar_stamp_system.sec);
//  ROS_DEBUG("Current tie is %u", t_now.sec);

  //IMU and Sonar are critical sensors, if they have timed out flash an error
  if(sensor_health.imu != ALIVE || sensor_health.sonar != ALIVE)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "IMU and/or Sonar timed out");
  }
  else
  {
    if(sensor_health.cv != ALIVE)
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Only Altitude ACTIVE; VSLAM not sending out any data");
    else
    {
      if(flags.lambda_initialized && cv_slam.flag_camera_rotation_corrected)
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Vision+Altitude based feedback is active");
      else
        stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Only Altitude ACTIVE; VSLAM not sending out any data");
    }
  }
  stat.add("IMU", getSensorStatus(sensor_health.imu));
  stat.add("Sonar", getSensorStatus(sensor_health.sonar));
  stat.add("EKF: Prediction", getBoolStatus(flags.ekf_prediction_enabled));
  stat.add("Baro", getSensorStatus(sensor_health.baro));
  stat.add("VSLAM", getSensorStatus(sensor_health.cv));
  stat.add("VSLAM: lambda", getBoolStatus(flags.lambda_initialized));
  stat.add("VSLAM: Rot_calib", getBoolStatus(cv_slam.flag_camera_rotation_corrected));
}

//Function for publishing some standard debug messages
void SensorFusion::publishDebugMessages(ros::Time stamp)
{

  //static states of the EKF
  z_static_states_msg.header.stamp = stamp;
  z_static_states_msg.vector.x = ekf_z.x_hat_kplus1_kplus1(2,0);
  z_static_states_msg.vector.y = ekf_z.x_hat_kplus1_kplus1(3,0);
  z_static_states_msg.vector.z = ekf_z.x_hat_kplus1_kplus1(4,0);

  //the important states of the EKF
  z_hat_msg.header.stamp = stamp;
  z_hat_msg.vector.x = ekf_z.x_hat_kplus1_kplus1(0,0);
  z_hat_msg.vector.y = ekf_z.x_hat_kplus1_kplus1(3,0)*cv_slam.position_vgnd.getZ() + ekf_z.x_hat_kplus1_kplus1(4,0);
  z_hat_msg.vector.z = sonar.depth;

  //position in the vslam inertial frame used for debugging data
  pos_msg_vgnd.header.stamp = stamp;
  pos_msg_vgnd.vector.x = cv_slam.position_vgnd.getX();
  pos_msg_vgnd.vector.y = cv_slam.position_vgnd.getY();
  pos_msg_vgnd.vector.z = cv_slam.position_vgnd.getZ();

  //acceleration in HBF
  tf::vector3TFToMsg(accel_hbf, acc_msg_hbf.vector);
  acc_msg_hbf.header.stamp = stamp;

  z_hat_pub.publish(z_hat_msg);
  pos_pub_vgnd.publish(pos_msg_vgnd);
  z_static_states_pub.publish(z_static_states_msg);
  acc_pub_hbf.publish(acc_msg_hbf);
}
