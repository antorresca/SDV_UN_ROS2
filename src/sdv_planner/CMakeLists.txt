cmake_minimum_required(VERSION 3.8)
project(sdv_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

add_executable(a_star_planner src/a_star_planner.cpp)
ament_target_dependencies(a_star_planner ${dependencies})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  a_star_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
