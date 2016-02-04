/*
 * pattern_track.cpp
 *
 *  Created on: 02-Feb-2016
 *      Author: atulya
 */

#include "cv_sens/pattern_track.h"

void printRotationMatrix(tf::Matrix3x3 R)
{
  printf("[%f. %f. %f\n%f, %f, %f\n %f, %f, %f]\n",
               R.getRow(0).getX(), R.getRow(0).getY(), R.getRow(0).getZ(),
               R.getRow(1).getX(), R.getRow(1).getY(), R.getRow(1).getZ(),
               R.getRow(2).getX(), R.getRow(2).getY(), R.getRow(2).getZ());
}


Tracker::Tracker(ros::NodeHandle nh, float z): it(nh)
{
  R_EB_CIB.setRPY(M_PI , 0.0f, -M_PI/2);
  R_CB_CIB_ideal.setRPY(0.0f, 0.0f, 0.0f);
  R_CB_CIB.setRPY(0.0f,0.0f, 0.0f);
  R_CB_EM.setRPY(0.0f,0.0f, 0.0f);
  R_EE_EB.setRPY(0.0f,0.0f, 0.0f);
  R_EE_EM.setRPY(0.0f,0.0f, 0.0f);

  target_height = z;

  flag_calibration_started = false;
  flag_calibration_image_acquired = false;
  flag_calibration_finished = false;
  calibration_count = 0;
  calibration_inlier_count = 0;

  cam_info_received = false;

  keyboard_input = boost::make_shared<vk::UserInputThread>();
}

Tracker::~Tracker()
{
  keyboard_input->stop();
}

void Tracker::initializeRotation()
{
  R_EE_EM = R_EE_EB;
  flag_calibration_started = true;
  double phi, theta, psi;
  R_EE_EM.getRPY(phi, theta, psi);
  ROS_INFO("Calibration Initiated\n Rot from IMU is [R: %.3f, P: %.3f, Y: %.3f]\n Press C when there is good tracking to calibrate rotation correction\n",
                                     phi, theta, psi);
}

void Tracker::useImageForCalibration()
{

  double phi_v, theta_v, psi_v;

  flag_calibration_image_acquired = true;

  R_CB_CIB = R_CB_EM*(R_EE_EM.transpose())*R_EE_EB*R_EB_CIB;
//  ROS_INFO("Rotation matrix R_CB_EM is");
//  printRotationMatrix(R_CB_EM);
//  ROS_INFO("Rotation matrix R_EM_EE is");
//  printRotationMatrix(R_EE_EM.transpose());
//  ROS_INFO("Rotation matrix R_EB_EE is");
//  printRotationMatrix(R_EE_EB.transpose());
//  ROS_INFO("Rotation matrix R_CIB_EB is");
//  printRotationMatrix(R_EB_CIB.transpose());

  R_CB_CIB.transpose().getRPY(phi_v, theta_v, psi_v);

  ROS_INFO("Rotation from camera ideal to camera is [R: %.3f, P: %.3f, Y: %.3f]",
                                       phi_v, theta_v, psi_v);
}

void Tracker::bypassCalibration()
{
  flag_calibration_finished = true;
  ROS_INFO("Bypassing calibration: Assuming that camera is vertical");
}

void Tracker::image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
  if(cam_info_received)
  {
    try
    {
      inImage = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      markers.clear();
      //Ok, let's detect
      mDetector.detect(inImage, markers, camParam, marker_size, false);

      for(size_t i=0; i<markers.size(); ++i)
      {
        // only publishing the selected marker
        if(markers[i].id == marker_id)
        {
//          tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
//          tf::StampedTransform cameraToReference;
//          cameraToReference.setIdentity();

//          geometry_msgs::PoseStamped poseMsg;
//          tf::poseTFToMsg(transform, poseMsg.pose);
//          poseMsg.header.frame_id = reference_frame;
//          poseMsg.header.stamp = curr_stamp;

//          pose_pub.publish(poseMsg);

        }
        //Draw a boundary around each detected pattern
        markers[i].draw(inImage,cv::Scalar(0,0,255),2);

        //draw a 3d cube in each marker if there is 3d info
        if(camParam.isValid() && marker_size!=-1)
        {
          for(size_t i=0; i<markers.size(); ++i)
          {
//            CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          }
        }

        //show input with augmented information
//                 cv_bridge::CvImage out_msg;
//                 out_msg.header.stamp = curr_stamp;
//                 out_msg.encoding = sensor_msgs::image_encodings::RGB8;
//                 out_msg.image = inImage;
//                 image_pub.publish(out_msg.toImageMsg());

      }
    }
    catch (cv_bridge::Exception& e)
    {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
    }
  }
}

// wait for one camerainfo, then shut down that subscriber
void Tracker::cam_info_callback(const sensor_msgs::CameraInfo &cam_info)
{
  cv::Mat cameraMatrix(3, 3, CV_32FC1);
  cv::Mat distorsionCoeff(4, 1, CV_32FC1);
  cv::Size size(cam_info.height, cam_info.width);

  for(int i=0; i<9; ++i)
    cameraMatrix.at<float>(i%3, i-(i%3)*3) = cam_info.K[i];

  if(cam_info.D.size() >= 4)
  {
    for(int i=0; i<4; ++i)
      distorsionCoeff.at<float>(i, 0) = cam_info.D[i];
  }
  else
  {
    ROS_WARN("length of camera_info D vector is not 4, assuming zero distortion...");
    for(int i=0; i<4; ++i)
      distorsionCoeff.at<float>(i, 0) = 0;
  }

  camParam = aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);

//  // handle cartesian offset between stereo pairs
//  // see the sensor_msgs/CamaraInfo documentation for details
//  rightToLeft.setIdentity();
//  rightToLeft.setOrigin(
//      tf::Vector3(
//          -msg.P[3]/msg.P[0],
//          -msg.P[7]/msg.P[5],
//          0.0));

  cam_info_received = true;
  cam_info_sub.shutdown();
}

void Tracker::markerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  double phi_v, theta_v, psi_v;
  double phi, theta, psi;

  tf::Matrix3x3 R_aruco_correction;
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.orientation, q);
  R_CB_EM.setRotation(q);
  R_aruco_correction.setRPY(M_PI/2, 0, M_PI);
  R_CB_EM = R_CB_EM*R_aruco_correction;             // In aruco_ros package; file aruco_ros_utils.cpp has a function
                                                    //arucoMarker2Tf which multiplies the rotation matrix by this value
                                                    // before publishing it. Hence the correction is applied here

//  ROS_INFO("Rotation matrix R_CB_EM is");
//  printRotationMatrix(R_CB_EM);
  tf::Matrix3x3 R_eb_em_from_cam = R_EB_CIB*R_CB_CIB.transpose()*R_CB_EM;
  R_eb_em_from_cam.transpose().getRPY(phi_v, theta_v, psi_v);   //this actually gives EB to EE RPY
//  ROS_INFO("Rotation from camera is [Roll: %f, Pitch: %f, Yaw: %f]",
//                                       phi_v, theta_v, psi_v);
//  rpy_cam_msg.vector.x = phi_v;
//  rpy_cam_msg.vector.y = theta_v;
//  rpy_cam_msg.vector.z = psi_v;

//  ROS_INFO("Rotation matrix R_EB_EE is");
//  printRotationMatrix(R_EE_EB.transpose());
  tf::Matrix3x3 R_eb_em_from_imu = R_EE_EB.transpose()*R_EE_EM;
  R_eb_em_from_imu.transpose().getRPY(phi, theta, psi);   //this actually gives EB to EE RPY
//  ROS_INFO("Rotation from IMU is [Roll: %f, Pitch: %f, Yaw: %f]",
//                                     phi, theta, psi);

//  rpy_imu_msg.vector.x = phi;
//  rpy_imu_msg.vector.y = theta;
//  rpy_imu_msg.vector.z = psi;

  if(fabs(fabs(psi - psi_v) - (2*M_PI)) < 0.2 )
  {
    if(psi<0)
      psi += 2*M_PI;
    else
      psi_v += 2*M_PI;
  }

  float meas_diff = sqrt(pow((phi_v - phi),2) + pow((theta_v - theta),2) + pow((psi_v - psi),2));
//  rpy_imu_msg.vector.y = meas_diff;

//  ROS_INFO("angle diff IMU and camera : %f, %d, %d", meas_diff, calibration_count, calibration_inlier_count);

  if(flag_calibration_finished == false && flag_calibration_started == true && flag_calibration_image_acquired == true)
  {
    if(calibration_count > CALIBRATION_NUM_VARIABLES)
    {
      ROS_INFO("CALIBRATION UNSUCCESSFUL : Press C to acquire another image and wait till a success is shown");
      flag_calibration_image_acquired = false;
    }
    calibration_count++;
    if(meas_diff < CALIBRATION_INLIER_ERR_THRSH)
      calibration_inlier_count++;

    if(calibration_inlier_count >= CALIBRATION_INLIER_PERCENT/100.0f*CALIBRATION_NUM_VARIABLES)
    {
      ROS_INFO("CALIBRATION SUCCESSFUL :\nNow the marker can be freely moved anywhere  (%d of %d are inliers)",
               calibration_inlier_count, calibration_count);
      flag_calibration_finished = true;
    }
  }

  tf::Vector3 r_cb, r_ee, r_ee_ideal, r_ehbf, r_ehbf_ideal, r_em;                                             //position of marker center with respect to the EE frame attached to quad
  tf::pointMsgToTF(msg->pose.position, r_cb);
  r_ee = R_EE_EB*R_EB_CIB*R_CB_CIB.transpose()*r_cb;
  r_ee_ideal = R_EE_EB*R_EB_CIB*R_CB_CIB_ideal.transpose()*r_cb;
//  r_em = R_EE_EM.transpose()*R_EE_EB*R_EB_CIB*R_CB_CIB.transpose()*(-1*r_cb);                   //camera with respect to the marker

  r_ehbf = R_EE_EHBF.transpose()*r_ee;
  r_ehbf_ideal = R_EE_EHBF.transpose()*r_ee_ideal;
  rpy_imu_msg.vector.x = r_cb.getX();
  rpy_imu_msg.vector.y = r_cb.getY();
  rpy_imu_msg.vector.z = r_cb.getZ();
////  ROS_INFO("desired position is [%f, %f, %f]", r_ehbf.getX(), r_ehbf.getY(), r_ehbf.getZ());
//  rpy_imu_msg.vector.x = r_ehbf_ideal.getX();
//  rpy_imu_msg.vector.y = r_ehbf_ideal.getY();
//  rpy_imu_msg.vector.z = r_ehbf_ideal.getZ();

  rpy_cam_msg.vector.x = r_ehbf.getX();
  rpy_cam_msg.vector.y = r_ehbf.getY();
  rpy_cam_msg.vector.z = r_ehbf.getZ();

//  ROS_INFO("RPY is [%f, %f, %f]", phi_v, theta_v, psi_v);
  marker_pos[0] = r_ehbf.getX();
  marker_pos[1] = r_ehbf.getY();
  marker_pos[2] = r_ehbf.getZ();

}

bool Tracker::isCalibrationFinished()
{
  return flag_calibration_finished;
}

void Tracker::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  double phi, theta, psi;

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->orientation, q);

  R_EE_EB.setRotation(q);
  R_EE_EB.getRPY(phi, theta, psi);
//  ROS_INFO("Rotation from IMU is [R: %.3f, P: %.3f, Y: %.3f]",
//                                         phi, theta, psi);
  R_EE_EHBF.setRPY(0.0f ,0.0f , psi);
}


void Tracker::processUserAction()
{
  if(keyboard_input!= NULL)
  {
    char terminal_input = keyboard_input->getInput();
    if(terminal_input != 0)
    {
      switch(terminal_input)
      {
        case 'i':
          initializeRotation();
          break;
        case 'c':
          useImageForCalibration();
          break;
        case 'b':
          bypassCalibration();
        default:
          break;
      }
    }
  }
}


void Tracker::publishDebugMessages()
{
  rpy_cam_pub.publish(rpy_cam_msg);
  rpy_imu_msg.header.stamp = ros::Time::now();
  rpy_cam_msg.header.stamp = ros::Time::now();
  rpy_imu_pub.publish(rpy_imu_msg);
}
