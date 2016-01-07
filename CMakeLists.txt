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
  pluginlib
  px_comm
  mavros
  mavros_msgs
  visualization_msgs
)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

include(EnableCXX11)
include(MavrosMavlink)

include_directories(
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
	 )
target_link_libraries(mavros_debug ${catkin_LIBRARIES} ${mavros_LIBRARIES})

add_executable(pos_fusion src/fusion_main.cpp src/ekf.cpp src/sensor_fusion.cpp)
target_link_libraries(pos_fusion   ${catkin_LIBRARIES} ${mavros_LIBRARIES})

add_executable(setpoint_enu src/setpoint.cpp)
target_link_libraries(setpoint_enu   ${catkin_LIBRARIES} )

add_executable(trak_marker src/marker_track.cpp)
target_link_libraries(trak_marker   ${catkin_LIBRARIES} )

add_executable(dummy_visionpose_publisher src/dummy_position.cpp)
target_link_libraries(dummy_visionpose_publisher   ${catkin_LIBRARIES} )
