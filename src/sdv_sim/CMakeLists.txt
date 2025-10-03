cmake_minimum_required(VERSION 3.5)
project(sdv_sim)

# Default to C++14 (por si tienes nodos C++)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_controllers REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install xacro files
install(DIRECTORY xacro
  DESTINATION share/${PROJECT_NAME}/
)

# Install models/worlds
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
