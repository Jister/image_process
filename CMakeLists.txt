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

add_message_files(
  FILES
  drone_info.msg
  robot_info.msg
  boundary_info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


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

link_directories(lib)

add_library(image_process_lib
  src/image_process.cpp 
)

add_executable(find_yaw_node src/find_yaw.cpp)
target_link_libraries(find_yaw_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  image_process_lib
)

add_executable(find_robot_node src/find_robot.cpp)
target_link_libraries(find_robot_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  image_process_lib
)

add_executable(find_position src/position.cpp)
target_link_libraries(find_position
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  image_process_lib
)


add_executable(image_test src/test.cpp)
target_link_libraries(image_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  image_process_lib
)

add_executable(attitude_correct src/attitude_correct.cpp)
target_link_libraries(attitude_correct
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  image_process_lib
  libxl.so
)

