/*
 * gps_position.cpp
 *
 *  Created on: 29-May-2015
 *      Author: atulya
 */

#include "sensor_fusion.h"
#include "ekf.h"

void statsCallback(const mavros_msgs::RCIn::ConstPtr& msg)
{
  //TODO change this channel when configuring remote
  int chnl_switch = msg->channels[5];

  if(chnl_switch > 2048 && SVO_INITIATED == false && SVO_RESET_SENT == false)
  {
    key_msg.data = "r";
    svo_remote_key_pub.publish(key_msg);
    SVO_RESET_SENT = true;

  }
  if(chnl_switch > 2048 && SVO_INITIATED == false && SVO_RESET_SENT == true)
  {
    key_msg.data = "s";
    svo_remote_key_pub.publish(key_msg);

    SVO_INITIATED = true;
  }
  if(chnl_switch < 2048)
  {
    SVO_RESET_SENT = false;
    SVO_INITIATED = false;
  }
}

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

void visualizationMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  if(msg->ns.compare("pts") == 0 && FLAG_CAMERA_CALIBRATED == false)
  {
    if(msg->pose.position.z == 0)
      return;
    ground_points[ground_points_index](0) = msg->pose.position.x;
    ground_points[ground_points_index](1) = msg->pose.position.y;
    ground_points[ground_points_index](2) = msg->pose.position.z;
    ground_points_index++;
    if(ground_points_index == GROUND_PLANE_POINTS)
    {
      calibrateCamera();
      FLAG_CAMERA_CALIBRATED = true;
    }
  }
  if(FLAG_CAMERA_CALIBRATED == true)
  {
    tf::Vector3 p(msg->pose.position.x,
                  msg->pose.position.y,
                  msg->pose.position.z);

    tf::Vector3 p_calib = R_calib.transpose()*p;
  //  tf::Vector3 p_calib = p;


    points_msg.header.frame_id = "/world";
    points_msg.header.stamp = msg->header.stamp;
    points_msg.ns = "calib_"+msg->ns;
    points_msg.id = msg->id;
    points_msg.type = visualization_msgs::Marker::CUBE;
    points_msg.action = msg->action; // 0 = add/modify
    points_msg.scale.x = msg->scale.x;
    points_msg.scale.y = msg->scale.y;
    points_msg.scale.z = msg->scale.z;
    points_msg.color.a = 1.0;
    points_msg.color.r = 0;
    points_msg.color.g = 1;
    points_msg.color.b = 0;
    points_msg.lifetime = msg->lifetime;
    points_msg.pose.position.x = p_calib.getX()+1;
    points_msg.pose.position.y = p_calib.getY()+1;
    points_msg.pose.position.z = p_calib.getZ();
    points_pub.publish(points_msg);

    //  ROS_INFO("NS of the point is %s and index is %d", msg->ns.c_str(), ground_points_index);
  }
}


void waypointNEDICallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  tf::Vector3 wp_nedi(msg->x, msg->y, msg->z);
  //converting from nedi to nedb
  tf::Vector3 wp_nedb =  R_vb_vgnd_t*wp_nedi;

  tf::vector3TFToMsg(wp_nedb, waypoint_msg_nedb);
  waypoint_pub.publish(waypoint_msg_nedb);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
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

  tf::Vector3 g(0,0,-9.81);	//the gravgndty vector

  accel_hbf = R_b_to_HBF*a_b + g;           //acceleration as obtained in the horizontal body frame and in NED coordinate system

  if(ekf_z.x_hat_kplus1_kplus1(0,0) > 0.3 && ekf_z.x_hat_kplus1_kplus1(0,0) < 4 )             // the absolute lower limit possible for ultrasonic sensors
    ekf_z.prediction(accel_hbf.getZ());

//  std::cout<<"EKF z predicted is"<<ekf_z.x_hat_kplus1_kplus1(0,0)<<"\n";

  tf::vector3TFToMsg(accel_hbf, acc_msg_hbf.vector);
  acc_msg_hbf.vector.x = z_hat_msg.vector.z;
  acc_msg_hbf.header.stamp = msg->header.stamp;

  acc_pub_hbf.publish(acc_msg_hbf);

  //check when was the last vision based reading obtained; if(greater than a threshold)then we have to reinitalize the scale and bias
//  ros::Duration delt = msg->header.stamp - last_cv_stamp;
  ros::Duration delt = ros::Time::now() - last_cv_stamp;
//  std::cout<<"time difference is "<<delt.toSec()<<"\n";
  if(delt.toSec()>5 && flag_lamda_initialized == true)
  {
    cv_sonar_correspondance.clear();
    flag_lamda_initialized = false;
    FLAG_CAMERA_CALIBRATED = false;
    position_vgnd.setZero();
    ROS_INFO("Visual reasings lost : lambda and bias will be reinitialized once VSLAM is recovered ");
  }
}

void vslamCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  last_cv_stamp = ros::Time::now();
  tf::Vector3 p(msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z);

  position_vgnd = R_calib.transpose()*p;

  //position in the vslam inertial frame used for debugging data
  pos_msg_vgnd.header.stamp = msg->header.stamp;
  pos_msg_vgnd.vector.x = position_vgnd.getX();
  pos_msg_vgnd.vector.y = position_vgnd.getY();
  pos_msg_vgnd.vector.z = position_vgnd.getZ();

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
  R_b_to_VGND.setRotation(q);
  R_b_to_VGND.getRPY(phi_v, theta_v, psi_v);

//  std::cout<<"psi_v i "<<psi_v<<"\n";
  R_HBF_to_GND.setRPY(0,0,psi_v);

  //update the vslam algorithm according to the inertial matrix
//  if(flag_lamda_initialized)
//    ekf_z.updateVSLAM(position_vgnd.getZ());

  //publishing pos and velocity in visual GND frame(which is at some psi_0 with respect to the GND frame) calculated to the HLP
  cv_pose_msg.header.stamp = msg->header.stamp;
  cv_pose_msg.pose.position.x = ekf_z.x_hat_kplus1_kplus1(3,0)*position_vgnd.getX();
  cv_pose_msg.pose.position.y = ekf_z.x_hat_kplus1_kplus1(3,0)*position_vgnd.getY();
  cv_pose_msg.pose.position.z = ekf_z.x_hat_kplus1_kplus1(0,0);
  tf::Quaternion q_hbf_to__vgnd;
  R_HBF_to_GND.getRotation(q_hbf_to__vgnd);
  tf::quaternionTFToMsg(q_hbf_to__vgnd, cv_pose_msg.pose.orientation);

  z_static_states_msg.header.stamp = msg->header.stamp;
//  z_static_states_msg.vector.x = ekf_z.x_hat_kplus1_kplus1(2,0);
//  z_static_states_msg.vector.y = ekf_z.x_hat_kplus1_kplus1(3,0);
//  z_static_states_msg.vector.z = ekf_z.x_hat_kplus1_kplus1(4,0);
  z_static_states_msg.vector.x = ekf_z.x_hat_kplus1_kplus1(3,0)*p.getZ() + ekf_z.x_hat_kplus1_kplus1(4,0) ;
  z_static_states_msg.vector.y = ekf_z.x_hat_kplus1_kplus1(3,0)*position_vgnd.getZ() + ekf_z.x_hat_kplus1_kplus1(4,0) ;
  z_static_states_msg.vector.z = ekf_z.x_hat_kplus1_kplus1(0,0);

  pos_pub_vgnd.publish(pos_msg_vgnd);

  if(flag_lamda_initialized && FLAG_CAMERA_CALIBRATED)
    cv_pose_pub.publish(cv_pose_msg);

  z_static_states_pub.publish(z_static_states_msg);

//  ROS_INFO("psi is %f", psi);
}

bool isOutlier(double z)
{
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

  if(fabs(z-median) > 0.6)
    return true;
  else
    return false;

}

void px4flowCallback(const px_comm::OpticalFlow::ConstPtr& msg)
{
  double z = msg->ground_distance;

  if(isOutlier(z))
  {
    rolling_q[ROLLING_FILTER_WINDOW-2] = ekf_z.x_hat_kplus1_kplus1(0,0);
//    std::cout<<"outlier detected";
  }
  else
  {
    if(flag_sonar_enabled)
    {
      rolling_q[ROLLING_FILTER_WINDOW-2] = z;
      ekf_z.updateUltrasonic(z);
      //finding out the scale and bias factor for the vision algorithm
      if(flag_lamda_initialized == false)
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


  z_hat_msg.header.stamp = msg->header.stamp;
  z_hat_msg.vector.x = ekf_z.x_hat_kplus1_kplus1(0,0);
//  z_hat_msg.vector.x = z_filter;
//  ROS_INFO("Scale: %f and bias:%f for VSLAM initialised", ekf_z.x_hat_kplus1_kplus1(3,0), ekf_z.x_hat_kplus1_kplus1(4,0));
  z_hat_msg.vector.y = ekf_z.x_hat_kplus1_kplus1(3,0)*position_vgnd.getZ() + ekf_z.x_hat_kplus1_kplus1(4,0) ;
//  z_hat_msg.vector.z = ekf_z.x_hat_kplus1_kplus1(2,0);

  z_hat_msg.vector.z = z;

//  z_hat_msg.vector.x = position_vgnd.getX();
//  z_hat_msg.vector.y = position_vgnd.getY();
//  z_hat_msg.vector.z = position_vgnd.getZ();

  z_hat_pub.publish(z_hat_msg);

  if(flag_lamda_initialized == false)
  {
    cv_pose_msg.header.stamp = msg->header.stamp;
    cv_pose_msg.pose.position.x = 0;
    cv_pose_msg.pose.position.y = 0;
    cv_pose_msg.pose.position.z = ekf_z.x_hat_kplus1_kplus1(0,0);
    cv_pose_pub.publish(cv_pose_msg);
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ekf_vslam_fusion");
  ros::NodeHandle n;

  R_vb_vgnd_t.setRPY(0,0,0);

  flag_sonar_enabled = true;
  flag_lamda_initialized = false;

  ros::Subscriber vslam_sub = n.subscribe("svo/pose", 10, vslamCallback);
  ros::Subscriber stats_sub = n.subscribe("/mavros/rc/in", 100, statsCallback);
  ros::Subscriber rpy_sub = n.subscribe("imu/data_raw", 100, imuCallback);
  ros::Subscriber px4flow_sub = n.subscribe("px4flow/opt_flow", 10, px4flowCallback);
  ros::Subscriber waypoint_sub = n.subscribe("waypoint_ned_i", 5, waypointNEDICallback);
  ros::Subscriber points_sub = n.subscribe("svo/points", 10, visualizationMarkerCallback);

  pos_pub_vgnd = n.advertise<geometry_msgs::Vector3Stamped>("cv_pose", 100);
  acc_pub_hbf = n.advertise<geometry_msgs::Vector3Stamped>("sens_acc_hbf", 100);
  z_hat_pub = n.advertise<geometry_msgs::Vector3Stamped>("z_hat_ekf", 100);
  cv_pose_pub = n.advertise<geometry_msgs::PoseStamped>("sens_pos", 100);
  svo_remote_key_pub = n.advertise<std_msgs::String>("svo/remote_key", 5);
  z_static_states_pub = n.advertise<geometry_msgs::Vector3Stamped>("z_static_states", 100);
  waypoint_pub = n.advertise<geometry_msgs::Vector3>("waypoint", 5);
  points_pub = n.advertise<visualization_msgs::Marker>("calib_points", 100);

//  Queue rt;

  //DEBUGGING THE QUEUE
//  for(int i=0; i<MAX_BUFFER_SIZE+5; i++)
//  {
//    float x,y, A=0.3;
//    if(i<50)
//      x = A;
//    if(i>=50 && i<100)
//      x = A + (i-50)/50.0*(-2*A);
//    if(i>=100)
//      x = -A;
//
//    y = 3.2*x + 2.4;
//    rt.insertElement(x,y);
//
//    if(rt.checkData() == true)
//      break;
//  }
//
//  float scale, z0;
//  if(rt.checkData() == true)
//    rt.calculateScale(scale, z0);

//  std::cout<<"for data mean is "<<rt.mean<<"; std_dev: "<<rt.std_dev<<"\n";
//  std::cout<<"calculated scale is "<<scale<<" ; "<<z0<<"\n";

  ros::spin();
}
