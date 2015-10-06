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
  px_comm
)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_C_FLAGS "-std=c99")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hello_asctec_imu
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs
#  DEPENDS system_lib
)
 
 add_executable(pos_sens src/ekf_vision.cpp src/ekf.cpp)
 target_link_libraries(pos_sens   ${catkin_LIBRARIES} )
