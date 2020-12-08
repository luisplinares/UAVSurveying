cmake_minimum_required(VERSION 2.8.3)
project(livox_mapping)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  rosbag
)

## Generate messages in the 'msg' folder
add_message_files(
	FILES
    CustomPoint.msg
    CustomMsg.msg
    RawUbx.msg
    GSOF.msg
    GNSS.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    geometry_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(livox_mapping_case src/livox_mapping_case.cpp src/Utils.cpp src/minIni.cpp)
add_dependencies(livox_mapping_case ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(livox_mapping_case ${catkin_LIBRARIES})




