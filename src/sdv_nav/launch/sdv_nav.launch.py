from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
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
    
    map_yaml_path = PathJoinSubstitution([
        get_package_share_directory("sdv_nav"),
        "maps",
        "LabFabEx.yaml"
    ])
    
    amcl_params_file = PathJoinSubstitution([
        get_package_share_directory("sdv_nav"),
        "config",
        "amcl_fixed.yaml" 
    ])
    
    # =======================
    # VARIABLES Y PATHS
    # =======================
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    sdv_description_pkg = get_package_share_directory('sdv_sim')
    sick_launch = os.path.join(
        get_package_share_directory('sick_scan_xd'),
        'launch',
        'sick_nav_350.launch.py'
    )
    
    # =======================
    # TF STATIC: base_footprint → base_link (SI LO NECESITAS)
    # =======================
    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_footprint',
        output='screen',
        arguments=[
            "0.0", "0.0", "0.0",
            "0", "0", "0",
            "base_footprint",
            "base_link"
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
            "0.25", "0.0", "0.0",
            "0", "0", "0",
            "base_link",
            "cloud"
        ]
    )
    
    # =======================
    # MAP SERVER
    # =======================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml_path},
            {"frame_id": "map"}
        ]
    )
    
    # =======================
    # AMCL
    # =======================


    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
        {
        'use_sim_time': False,
        'base_frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'global_frame_id': 'map',
        'laser_frame_id': 'cloud',
        'odom_topic': 'odom',
        'scan_topic': 'scan',
        'map_topic': 'map',
        'tf_broadcast': True,
        'transform_tolerance': 1.0,
        'set_initial_pose': False,
        'min_particles': 100,
        'max_particles': 500,
        'update_min_d': 0.2,
        'update_min_a': 0.5,
        'laser_model_type': 'likelihood_field',
        'laser_max_range': 10.0,
        'max_beams': 30,
        'odom_model_type': 'diff',
        'alpha1': 0.2,
        'alpha2': 0.2,
        'alpha3': 0.2,
        'alpha4': 0.2
        }
        ],
        remappings=[
        ('scan', '/scan'),
        ('odom', '/odom'),
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
    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen"
    )
    
    # =======================
    # SICK LIDAR
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
    # PLANNER NODE
    # =======================
    planner_node = Node(
        package="sdv_planner",
        executable="a_star_planner",
        name="sdv_planner_node",
        output="screen"
    )
    # =======================
    # CONTROL NODE
    # =======================
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
        {
        'use_sim_time': False,
        'controller_frequency': 20.0,
        'FollowPath': {
        'plugin': 'dwb_core::DWBLocalPlanner',
        'critics': [
        'RotateToGoal',
        'Oscillation',
        'ObstacleFootprint',
        'GoalAlign',
        'PathAlign',
        'PathDist',
        'GoalDist'
        ],
        'base_frame_id': 'base_link',
        'global_frame_id': 'map',
        'odom_frame_id': 'odom',
        'min_vel_x': 0.0,
        'max_vel_x': 0.5,
        'min_vel_theta': -1.0,
        'max_vel_theta': 1.0,
        'vx_samples': 10,
        'vtheta_samples': 20,
        'acc_lim_x': 1.0,
        'acc_lim_theta': 2.0,
        'obstacle_radius': 0.22,
        'footprint_model': {
        'type': 'circle',
        'radius': 0.22
        }
        }
        }
        ],
        remappings=[
        ('cmd_vel', '/cmd_vel'),
        ('global_plan', '/a_star/path'),
        ('odom', '/odom')
        ]
    )
    
    # =======================
    # LIFECYCLE COMMANDS
    # =======================
    from launch.actions import ExecuteProcess, TimerAction
    import launch
    lifecycle_commands = [
    
        # =======================
        # MAP SERVER
        # =======================
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
                    output='screen'
                )
            ]
        ),
        # =======================
        # AMCL
        # =======================
        TimerAction(
            period=4.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/amcl', 'configure'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=5.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/amcl', 'activate'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=6.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/amcl', 'base_base_frame_id','base_link'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/amcl', 'deactivate'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/amcl', 'cleanup'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/amcl', 'configure'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=11.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/amcl', 'activate'],
                    output='screen'
                )
            ]
        ),
        # =======================
        # CONTROLLER SERVER (RPP)
        # =======================
        TimerAction(
            period=13.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/controller_server', 'configure'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=14.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', '/controller_server', 'activate'],
                    output='screen'
                )
            ]
        )
    ]
    
    # =======================
    # ARGUMENTOS Y NODOS
    # =======================
    return LaunchDescription([
        # Argumentos
        use_sim_time_arg,
        
        # Base Robot (solo si usas base_footprint)
        # base_tf,  # Descomentar si necesitas base_footprint
        
        # Transformada LIDAR
        cloud_tf,
        
        # Robot State Publisher
        robot_state_pub,
        joint_state_pub,
        
        # Drivers y Control
        sick_node,
        serial_node,
        controller_node,
        planner_node,
        controller_server,
        # Nav2 Localization
        map_server,
        amcl,
        
        # Lifecycle commands
        *lifecycle_commands
    ])