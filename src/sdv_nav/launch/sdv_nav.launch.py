from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node, LifecycleNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # =======================
    # DECLARACIÓN DE ARGUMENTOS
    # =======================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            get_package_share_directory("sdv_nav"),
            "maps",
            "LabFabEx.yaml"
        ]),
        description='Full path to map yaml file to load'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory("sdv_nav"),
            "config",
            "nav2_params.yaml"
        ]),
        description='Full path to the ROS2 parameters file to use'
    )
    
    # =======================
    # VARIABLES Y PATHS
    # =======================
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_path = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    
    sdv_description_pkg = get_package_share_directory('sdv_sim')
    sick_launch = os.path.join(
        get_package_share_directory('sick_scan_xd'),
        'launch',
        'sick_nav_350.launch.py'
    )
    
    # =======================
    # NAV2 LIFECYCLE MANAGER
    # =======================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )
    
    # =======================
    # MAP SERVER (como LifecycleNode)
    # =======================
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml_path}
        ]
    )
    
    # =======================
    # AMCL (como LifecycleNode)
    # =======================
    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
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
            "cloud"                 # child frame
        ]
    )
    
    # =======================
    # ROBOT DESCRIPTION
    # =======================
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
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # =======================
    # SICK LIDAR (Driver)
    # =======================
    sick_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sick_launch)
    )
    
    # =======================
    # SERIAL NODE
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
    # ARGUMENTOS Y NODOS
    # =======================
    return LaunchDescription([
        # Argumentos
        use_sim_time_arg,
        map_yaml_arg,
        params_file_arg,
        
        # Base Robot
        robot_state_pub,
        cloud_tf,
        
        # Drivers y Control
        sick_node,
        serial_node,
        controller_node,
        
        # Nav2 Localization Stack
        map_server,
        amcl,
        lifecycle_manager,
    ])