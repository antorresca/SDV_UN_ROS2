import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Ruta del paquete sdv_description
    sdv_description = get_package_share_directory("sdv_sim")

    # Argumento para el modelo XACRO
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(sdv_description, "xacro", "sdv_description.xacro"),
        description="Absolute path to robot xacro file"
    )

    # Argumento para el mundo
    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty"
    )

    # Construcción de la ruta del mundo
    world_path = PathJoinSubstitution([
        get_package_share_directory("sdv_sim"),
        "worlds",
        "empty.sdf"
    ])

    # Directorios de modelos para Gazebo
    model_path = str(Path(sdv_description).parent.resolve())
    model_path += pathsep + os.path.join(sdv_description, "models")

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", model_path
    )

    # Comando para generar el URDF/XACRO
    is_ignition = "True"  # Siempre True para Ignition Gazebo
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}],
        output="screen"
    )

    # Lanzar Ignition Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py"
        ]),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items()
    )

    # Spawnear el robot
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "sdvun1"],
    )

    # Bridge de sensores (IMU, LIDAR)
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        remappings=[
            ('/imu', '/imu/out'),
        ],
        output="screen"
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])
