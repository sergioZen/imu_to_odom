cmake_minimum_required(VERSION 3.0.2)
project(imu_to_odom)

#############
## Package ##
#############

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  eigen_conversions
  tf
)

find_package(tf REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
)

# catkin_package(
#   INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
#   CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs eigen_conversions
#   DEPENDS EIGEN3
# )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/imu_to_odom_predictor.cpp
  src/imu_to_odom_chatgpt.cpp
)

# ############
# # BINARIES #
# ############
add_executable(${PROJECT_NAME}_node src/imu_to_odom_node.cpp)

target_link_libraries(imu_to_odom_node
   ${PROJECT_NAME}
   ${tf_LIBRARIES}
   ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS scripts/tf_broadcaster_imu.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
