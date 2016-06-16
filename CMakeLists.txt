cmake_minimum_required(VERSION 2.8.3)
project(image_process)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)
find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_process
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(image_process_lib
  src/image_process.cpp 
)

add_executable(find_yaw_node src/find_yaw.cpp)
target_link_libraries(find_yaw_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  image_process_lib
)

add_executable(test src/test.cpp)
target_link_libraries(test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  image_process_lib
)

