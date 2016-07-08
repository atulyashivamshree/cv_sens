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
  tf
  cv_bridge
  image_transport
  pluginlib
  px_comm
  mavros
  mavros_msgs
  visualization_msgs
  vikit_common
  aruco
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
  ${mavlink_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs
  DEPENDS Eigen OpenCV
)

add_library(mavros_debug
	 src/plugins/hil_state.cpp
	 src/plugins/sim_state.cpp
	 src/plugins/attitude.cpp
	 src/plugins/local_position_ned.cpp
	 src/plugins/vision_pose_ned.cpp
	 )
target_link_libraries(mavros_debug ${catkin_LIBRARIES} ${mavros_LIBRARIES})

add_executable(position_fusion src/fusion_main.cpp src/ekf.cpp src/sensor_fusion.cpp src/sensor_modules.cpp)
target_link_libraries(position_fusion   ${catkin_LIBRARIES} ${mavros_LIBRARIES})

add_executable(setpoint_enu src/setpoint.cpp)
target_link_libraries(setpoint_enu   ${catkin_LIBRARIES} )

add_executable(trak_marker src/marker_track_main.cpp src/pattern_track.cpp)
target_link_libraries(trak_marker   ${catkin_LIBRARIES} )

add_executable(dummy_vision_pose src/dummy_position.cpp)
target_link_libraries(dummy_vision_pose   ${catkin_LIBRARIES} )
