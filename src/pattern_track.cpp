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

void Pattern_Tracker::draw3dAxis(aruco::Marker m)
{

    float size= m.ssize*3;

    vector<cv::Point3f> object_points(4, cv::Point3f(0.0, 0.0, 0.0));
    object_points[1].x = size;
    object_points[2].y = size;
    object_points[3].z = size;

    vector<cv::Point2f> image_points;
    cv::projectPoints( object_points, m.Rvec,m.Tvec, camParam.CameraMatrix, camParam.Distorsion, image_points);

    //draw lines of different colours
    cv::line(inImage,image_points[0],image_points[1],cv::Scalar(255,0,0,255),1,CV_AA);
    cv::line(inImage,image_points[0],image_points[2],cv::Scalar(0,255,0,255),1,CV_AA);
    cv::line(inImage,image_points[0],image_points[3],cv::Scalar(0,0,255,255),1,CV_AA);
    putText(inImage,"x", image_points[1],cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,0,0,255),2);
    putText(inImage,"y", image_points[2],cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0,255),2);
    putText(inImage,"z", image_points[3],cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255,255),2);
}


Pattern_Tracker::Pattern_Tracker(ros::NodeHandle nh, float target_z,
                                 float marker_size, int marker_id)
: it(nh)
{
  R_EB_CIB.setRPY(M_PI , 0.0f, -M_PI/2);
  R_CB_CIB_ideal.setRPY(0.0f, 0.0f, 0.0f);
  R_CB_CIB.setRPY(0.0f,0.0f, 0.0f);
  R_CB_EM.setRPY(0.0f,0.0f, 0.0f);
  R_EE_EB.setRPY(0.0f,0.0f, 0.0f);
  R_EE_EM.setRPY(0.0f,0.0f, 0.0f);

  target_height = target_z;             //target height may not be required here
  marker_size_ = marker_size;
  marker_id_ = marker_id;

  flag_calibration_started = false;
  flag_calibration_image_acquired = false;
  flag_calibration_finished = false;
  calibration_count = 0;
  calibration_inlier_count = 0;

  cam_info_received = false;

  image_sub = it.subscribe("image", 1, &Pattern_Tracker::imageCallback, this);
  cam_info_sub = nh.subscribe("camera_info", 1, &Pattern_Tracker::camInfoCallback, this);
  imu_sub = nh.subscribe("/mavros/imu/data", 5, &Pattern_Tracker::imuCallback, this);

  image_pub = it.advertise("result", 1);
  rpy_imu_pub = nh.advertise<geometry_msgs::Vector3Stamped>("rpy_imu", 100);
  rpy_cam_pub = nh.advertise<geometry_msgs::Vector3Stamped>("rpy_cam", 100);
  relative_position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("rel_position_marker", 100);

  keyboard_input = boost::make_shared<vk::UserInputThread>();
}

Pattern_Tracker::~Pattern_Tracker()
{
  keyboard_input->stop();
}

void Pattern_Tracker::initializeRotation()
{
  R_EE_EM = R_EE_EB;
  flag_calibration_started = true;
  double phi, theta, psi;
  R_EE_EM.getRPY(phi, theta, psi);
  ROS_INFO("Calibration Initiated\n Rot from IMU is [R: %.3f, P: %.3f, Y: %.3f]\n Press C when there is good tracking to calibrate rotation correction\n",
                                     phi, theta, psi);
}

void Pattern_Tracker::useImageForCalibration()
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

void Pattern_Tracker::bypassCalibration()
{
  flag_calibration_finished = true;
  ROS_INFO("Bypassing calibration: Assuming that camera is vertical");
}

void Pattern_Tracker::transformMarkerPoseToHBFFrame(tf::Matrix3x3 marker_rot, tf::Vector3 marker_pos)
{
  R_CB_EM = marker_rot;

  tf::Vector3 r_cb, r_ee, r_ee_ideal, r_ehbf, r_ehbf_ideal, r_em;                                             //position of marker center with respect to the EE frame attached to quad
  r_cb = marker_pos;
  t_em_cb = r_cb;

  r_ee = R_EE_EB*R_EB_CIB*R_CB_CIB.transpose()*r_cb;
  r_ee_ideal = R_EE_EB*R_EB_CIB*R_CB_CIB_ideal.transpose()*r_cb;
  //  r_em = R_EE_EM.transpose()*R_EE_EB*R_EB_CIB*R_CB_CIB.transpose()*(-1*r_cb);                   //camera with respect to the marker

  r_ehbf = R_EE_EHBF.transpose()*r_ee;
  r_ehbf_ideal = R_EE_EHBF.transpose()*r_ee_ideal;

  tf::Vector3 t_cb_em = R_CB_EM.transpose()*(-1*t_em_cb);
  tf::vector3TFToMsg(t_cb_em, relative_position_msg.vector);

//  tf::vector3TFToMsg(r_cb, rpy_imu_msg.vector);
//  tf::vector3TFToMsg(r_ehbf, rpy_cam_msg.vector);

  //  ROS_INFO("RPY is [%f, %f, %f]", phi_v, theta_v, psi_v);
  marker_pos_ = r_ehbf;

}

void Pattern_Tracker::updateCalibrationCheck()
{
  double phi_v, theta_v, psi_v;
  double phi, theta, psi;

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
}

void Pattern_Tracker::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  if(cam_info_received)
  {
    try
    {
      inImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

      markers.clear();
      //Ok, let's detect
//      ROS_INFO("TIC");
      mDetector.detect(inImage, markers, camParam, marker_size_, false);
//      ROS_INFO("TOC");

      for(size_t i=0; i<markers.size(); ++i)
      {
        // only publishing the selected marker
        if(markers[i].id == marker_id_)
        {
          cv::Mat rot(3, 3, CV_32FC1);
          cv::Rodrigues(markers[i].Rvec, rot);
          cv::Mat tran = markers[i].Tvec;

          tf::Matrix3x3 tf_rot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
                               rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
                               rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));
          tf::Vector3 tf_orig(tran.at<float>(0,0), tran.at<float>(1,0), tran.at<float>(2,0));
          tf::Transform transform(tf_rot, tf_orig);

//          ROS_INFO("Rotation matrix R_CB_EM from aruco raw is");
//          printRotationMatrix(tf_rot);

          //update a flag to indicate that a new marker has been received
          flag_new_marker_received = true;

          //transform marker coordinates to HBF frame
          transformMarkerPoseToHBFFrame(tf_rot, tf_orig);

          //check whether the calibration steps have been completed
          updateCalibrationCheck();

        }
        //Draw a boundary around each detected pattern
        markers[i].draw(inImage,cv::Scalar(0,0,255),2);
      }

      //draw a 3d cube in each marker if there is 3d info
      if(camParam.isValid() && marker_size_!=-1)
      {
        for(size_t i=0; i<markers.size(); ++i)
        {
          draw3dAxis(markers[i]);
        }
      }

      //show input with augmented information
      cv_bridge::CvImage out_msg;
      out_msg.header.stamp = msg->header.stamp;
      out_msg.encoding = sensor_msgs::image_encodings::RGB8;
      out_msg.image = inImage;
      image_pub.publish(out_msg.toImageMsg());

    }
    catch (cv_bridge::Exception& e)
    {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
    }
  }
}

// wait for one camerainfo, then shut down that subscriber
void Pattern_Tracker::camInfoCallback(const sensor_msgs::CameraInfo &cam_info)
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

  ROS_INFO("Camera parameters received");
  std::cout<<"Camera matrix is "<<cameraMatrix<<"\n";
  std::cout<<"distortion is "<<distorsionCoeff<<"\n";
  std::cout<<"size is "<<size<<"\n";

  cam_info_received = true;
  cam_info_sub.shutdown();
}

void Pattern_Tracker::markerCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  tf::Matrix3x3 R_aruco_correction, R_aruco_single;
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.orientation, q);
  R_aruco_single.setRotation(q);
  R_aruco_correction.setRPY(M_PI/2, 0, M_PI);
  R_aruco_single = R_aruco_single*R_aruco_correction;             // In aruco_ros package; file aruco_ros_utils.cpp has a function
                                                    //arucoMarker2Tf which multiplies the rotation matrix by this value
                                                    // before publishing it. Hence the correction is applied here

  ROS_INFO("Rotation matrix from aruco is");
  printRotationMatrix(R_aruco_single);

}

bool Pattern_Tracker::isCalibrationFinished() const
{
  return flag_calibration_finished;
}

void Pattern_Tracker::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
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


void Pattern_Tracker::processUserAction()
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

tf::Vector3 Pattern_Tracker::getCameraPosInMarkerFrame() const
{
  tf::Vector3 t_cb_em = R_CB_EM*(-1*t_em_cb);
  return t_cb_em;
}

tf::Vector3 Pattern_Tracker::getDesiredMovementToReachTarget(tf::Vector3 pos_target) const
{
  tf::Vector3 r_cb = R_CB_EM*pos_target + t_em_cb;
  tf::Vector3 r_ee = R_EE_EB*R_EB_CIB*R_CB_CIB.transpose()*r_cb;
  tf::Vector3 r_ehbf = R_EE_EHBF.transpose()*r_ee;

  return r_ehbf;
}


void Pattern_Tracker::publishDebugMessages()
{
  ros::Time stamp = ros::Time::now();
  rpy_imu_msg.header.stamp = stamp;
  rpy_cam_msg.header.stamp = stamp;
  relative_position_msg.header.stamp = stamp;

  rpy_cam_pub.publish(rpy_cam_msg);
  rpy_imu_pub.publish(rpy_imu_msg);
  relative_position_pub.publish(relative_position_msg);
}
