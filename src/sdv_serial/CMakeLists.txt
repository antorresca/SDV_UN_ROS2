cmake_minimum_required(VERSION 3.5)
project(sdv_serial)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# usar pkg-config para encontrar libserial
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

include_directories(
  include
  ${LIBSERIAL_INCLUDE_DIRS}
)

add_executable(sdv_serial_node src/sdv_serial_node.cpp)

ament_target_dependencies(sdv_serial_node
  rclcpp std_msgs sensor_msgs geometry_msgs
)

target_link_libraries(sdv_serial_node
  ${LIBSERIAL_LIBRARIES}
)

install(TARGETS sdv_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
