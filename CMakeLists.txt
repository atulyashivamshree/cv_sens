cmake_minimum_required(VERSION 2.8.3)
project(cv_sens)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs	
  dynamic_reconfigure
  #TODO remove dynamic reconfigure coz its probably not required
  pluginlib
  px_comm
  mavros
  mavros_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

include(EnableCXX11)
include(MavrosMavlink)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hello_asctec_imu
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs
  DEPENDS Eigen OpenCV
)

add_library(mavros_debug
	 src/plugins/hil_state.cpp
	 src/plugins/sim_state.cpp
	 src/plugins/attitude.cpp
	 src/plugins/local_position_ned.cpp
	 )
target_link_libraries(mavros_debug ${catkin_LIBRARIES})

add_executable(pos_sens_svo src/ekf_svo.cpp src/ekf.cpp src/sensor_fusion.cpp)
target_link_libraries(pos_sens_svo   ${catkin_LIBRARIES} )
 
add_executable(pos_sens_aruco src/ekf_aruco.cpp src/ekf.cpp)
target_link_libraries(pos_sens_aruco   ${catkin_LIBRARIES} )