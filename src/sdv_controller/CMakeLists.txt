cmake_minimum_required(VERSION 3.8)
project(sdv_controller)

# Dependencias de ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Incluir directorios de cabeceras (para motor.h en include/)
include_directories(
  include
)

# Compilar motor como librería
add_library(motor
  src/motor.cpp
)
target_include_directories(motor PUBLIC include)
ament_target_dependencies(motor rclcpp std_msgs geometry_msgs sensor_msgs)

# Compilar el nodo principal
add_executable(sdv_controller_node
  src/sdv_controller_node.cpp
)
ament_target_dependencies(sdv_controller_node rclcpp std_msgs geometry_msgs sensor_msgs)

# Enlazar el nodo con la librería motor
target_link_libraries(sdv_controller_node motor)

# Instalar binarios
install(TARGETS
  sdv_controller_node
  motor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
