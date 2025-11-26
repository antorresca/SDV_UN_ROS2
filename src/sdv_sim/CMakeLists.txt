cmake_minimum_required(VERSION 3.5)
project(sdv_sim)

# Si tienes xacros o urdf, esto es suficiente
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Instalar launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Instalar xacro/urdf
install(
  DIRECTORY xacro
  DESTINATION share/${PROJECT_NAME}/
)

# Instalar meshes/modelos (si tienes)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(
    DIRECTORY models
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

# Si NO usas Gazebo, puedes eliminar esta sección
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/worlds")
  install(
    DIRECTORY worlds
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_package()
