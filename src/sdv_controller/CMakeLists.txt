cmake_minimum_required(VERSION 3.8)
project(sdv_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${LIBSERIAL_INCLUDE_DIRS}
)

add_executable(sdv_controller_node src/sdv_controller_node.cpp)

ament_target_dependencies(sdv_controller_node
  rclcpp std_msgs sensor_msgs geometry_msgs
)

install(TARGETS sdv_controller_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
