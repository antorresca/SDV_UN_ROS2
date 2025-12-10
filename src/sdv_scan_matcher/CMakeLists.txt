cmake_minimum_required(VERSION 3.5)
project(sdv_scan_matcher)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(sdv_scan_matcher src/sdv_scan_matcher.cpp)
ament_target_dependencies(sdv_scan_matcher
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
)

target_include_directories(sdv_scan_matcher PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(sdv_scan_matcher PRIVATE ${EIGEN3_LIBRARIES})

install(TARGETS
  sdv_scan_matcher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
