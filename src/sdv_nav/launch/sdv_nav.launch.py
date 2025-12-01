from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # =======================
    # DECLARACIÓN DE ARGUMENTOS (para AMCL)
    # =======================
    # Define la ruta al archivo de parámetros de AMCL (debes crearlo en sdv_nav/config)
    amcl_params_file = PathJoinSubstitution([
        get_package_share_directory("sdv_nav"),
        "config",
        "amcl_params.yaml" # <-- DEBES CREAR ESTE ARCHIVO
    ])
    
    # Argumento para usar el tiempo de simulación (útil si usas Gazebo o similar)
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # =======================
    # PATHS
    # =======================
    sdv_description_pkg = get_package_share_directory('sdv_sim')

    sick_launch = os.path.join(
        get_package_share_directory('sick_scan_xd'),
        'launch',
        'sick_nav_350.launch.py'
    )

    map_yaml_path = PathJoinSubstitution([
        get_package_share_directory("sdv_nav"),
        "maps",
        "LabFabEx.yaml"
    ])
    
    # =======================
    # AMCL (Adaptive Monte Carlo Localization)
    # =======================
    # Este nodo publica la transformación map -> odom para cerrar el ciclo TF
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            {"use_sim_time": LaunchConfiguration('use_sim_time')},
            amcl_params_file  # Carga los parámetros del nodo AMCL
        ],
        remappings=[
            ('scan', '/scan'),  # Asegura que AMCL escuche el tópico /scan del LIDAR
            # Agrega otros remapeos si tu odometría no está en /odom
            # ('odom', '/tu_topico_odom'), 
        ]
    )


    # =======================
    # TF STATIC: base_link → cloud (LIDAR)
    # =======================
    cloud_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser_base',
        output='screen',
        arguments=[
            "0.25", "0.0", "0.0",   # x y z del LIDAR respecto a base_link
            "0", "0", "0",          # roll pitch yaw
            "base_link",            # parent frame
            "cloud"                 # child frame (coincide con el scanner_frame del SICK)
        ]
    )

    # ... (URDF / XACRO, ROBOT STATE PUBLISHER, JOINT STATE PUBLISHER: SIN CAMBIOS) ...

    robot_description = {
        "robot_description": Command([
            "xacro ",
            PathJoinSubstitution([
                sdv_description_pkg,
                "xacro",
                "sdv_description.xacro"
            ])
        ])
    }

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description]
    )

    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen"
    )

    # =======================
    # SICK LIDAR (Driver)
    # =======================
    sick_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sick_launch)
    )

    # =======================
    # SERIAL NODE (publica odom → base_link)
    # =======================
    serial_node = Node(
        package="sdv_serial",
        executable="sdv_serial_node",
        name="sdv_serial_node",
        output="screen"
    )

    # =======================
    # CONTROLLER NODE
    # =======================
    controller_node = Node(
        package="sdv_controller",
        executable="sdv_controller_node",
        name="sdv_controller_node",
        output="screen"
    )

    # =======================
    # MAP SERVER (Mantiene el Mapa en Memoria)
    # =======================
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml_path},
            # Es importante que el map_server NO publique la TF. AMCL lo hará.
            {"frame_id": "map"} 
        ]
    )

    # =======================
    # RETURN
    # =======================
    return LaunchDescription([
        # Argumentos
        use_sim_time,
        
        # Base Robot
        robot_state_pub,
        joint_state_pub,
        cloud_tf,
        
        # Drivers y Control
        sick_node,
        serial_node,
        controller_node,
        
        # Localización y Mapa
        map_server,
        amcl_node, # <-- ¡NUEVO!
    ])