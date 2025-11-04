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
    # 1. Rutas y Nombres de Paquetes
    # Se asume que tu paquete de descripción es 'sdv_sim'
    sdv_description_pkg = get_package_share_directory("sdv_sim") 

    # 2. Argumentos de Lanzamiento
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(sdv_description_pkg, "xacro", "sdv_description.xacro"),
        description="Absolute path to robot xacro file"
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty"
    )

    # 3. Configuración de Entorno para Gazebo
    world_path = PathJoinSubstitution([
        sdv_description_pkg,
        "worlds",
        "empty.sdf"
    ])

    # Directorios de modelos para Gazebo
    model_path = str(Path(sdv_description_pkg).parent.resolve())
    model_path += pathsep + os.path.join(sdv_description_pkg, "models")

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", model_path
    )

    # 4. Generación del URDF
    is_ignition = "True"
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    # 5. Nodos de Publicación de Estado (Esenciales para RViz 2)
    
    # 5a. robot_state_publisher: Lee el URDF y publica los TFs de todos los links
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}],
        output="screen"
    )
    
    # 5b. joint_state_publisher: Publica el estado inicial de los joints no-fijos (ruedas)
    # NECESARIO para que robot_state_publisher pueda calcular las transformadas al inicio.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    # 6. Lanzamiento de Gazebo (Ignition)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py"
        ]),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items()
    )

    # 7. Spawnear el Robot en Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "sdvun1", 
                   "-x", "0", "-y", "0", "-z", "0.15" 
                   ],
    )

    # 8. Bridge ROS-Gazebo (Para Sensores)
    # Conecta los tópicos nativos de Gazebo con los tópicos de ROS 2.
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        remappings=[
            # Remapeación para conectar el tópico /imu del bridge con el tópico /imu/out 
            # que el plugin de Gazebo publica internamente (configurado en el XACRO).
            ('/imu', '/imu/out'), 
        ],
        output="screen"
    )

    # 9. Retorno de la Descripción del Lanzamiento
    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        joint_state_publisher_node, # <--- ¡La corrección!
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge
    ])