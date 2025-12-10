from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # ===========================
    # ARGUMENTOS
    # ===========================
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    map_yaml_path = PathJoinSubstitution([
        get_package_share_directory("sdv_nav"),
        "maps",
        "LabFabEx.yaml"
    ])

    sdv_description_pkg = get_package_share_directory('sdv_sim')

    sick_launch = os.path.join(
        get_package_share_directory('sick_scan_xd'),
        'launch',
        'sick_nav_350.launch.py'
    )

    # ===========================
    # TF
    # ===========================
    cloud_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        output='screen',
        arguments=[
            "0.25", "0", "0",
            "0", "0", "0",
            "base_link",
            "cloud"
        ]
    )

    footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_footprint',
        output='screen',
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "base_link",
            "base_footprint"
        ]
    )

    # ===========================
    # ROBOT DESCRIPTION
    # ===========================
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
        output="screen"
    )
    
    # ===========================
    # DRIVERS
    # ===========================
    sick_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sick_launch)
    )

    serial_node = Node(
        package="sdv_serial",
        executable="sdv_serial_node",
        name="sdv_serial_node",
        output="screen"
    )

    controller_node = Node(
        package="sdv_controller",
        executable="sdv_controller_node",
        name="sdv_controller_node",
        output="screen"
    )

    tracking_node = Node(
        package="sdv_tracking",
        executable="pure_pursuit",
        name="sdv_controller_node",
        output="screen"
    )

    planner_node = Node(
        package="sdv_planner",
        executable="a_star_planner",
        name="sdv_planner",
        output="screen"
    )     
    
    # ===========================
    # NUEVO NODO: traslate (sdv_pruebas)
    # Usa la acción ComputePathToPose y luego FollowPath
    # ===========================
    traslate_node = Node(
        package="sdv_pruebas",
        executable="sdv_translate",
        name="sdv_translate",
        output="screen"
    )

    # ===============================================
    # NAV2 LIFECYCLE NODES
    # ===============================================

    # ===========================
    # MAP SERVER
    # ===========================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml_path},
            {'autostart': False}
        ]
    )

    # ===========================
    # AMCL
    # ===========================
#    amcl = Node(
#        package='nav2_amcl',
#        executable='amcl',
#        name='amcl',
#        output='screen',
#        parameters=[{
#            'use_sim_time': use_sim_time,
#            'autostart': False,
#            'base_frame_id': 'base_link',
#            'odom_frame_id': 'odom',
#            'global_frame_id': 'map',
#            'laser_frame_id': 'cloud',
#            'scan_topic': 'scan',
#        }],
#        remappings=[
#            ('scan', '/scan'),
#            ('odom', '/odom'),
#        ]
#    )

    # ===========================
    # HECTOR SLAM
    # ===========================
    hector_slam = Node(
        package='hector_mapping',
        executable='hector_mapping',
        name='hector_mapping',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'pub_map_scanmatch_transform': True,
            'tf_map_scanmatch_transform_frame_name': 'map',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'map_frame': 'map'
        }],
        remappings=[
            ('scan', '/scan'),
            ('map', '/map_dinamico'),              
            ('map_metadata', '/map_dinamico_metadata')
        ]
    )

    # ===========================
    # NAV2: PLANNER SERVER
    # ===========================
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': False,
            'expected_planner_frequency': 5.0,
            'planner_plugins': ['GridBased'],
            'GridBased': {
                'plugin': 'nav2_navfn_planner/NavfnPlanner',
                'tolerance': 0.5,
                'use_astar': False,
                'allow_unknown': True
            }
        }]
    )

    # ===========================
    # NAV2: CONTROLLER SERVER (FIXED)
    # Parámetros CRÍTICOS añadidos para evitar el Segmentation Fault
    # ===========================
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': False,
            
            # Frecuencia del controlador
            'controller_frequency': 20.0,
            
            # 1. Definición y configuración del Goal Checker
            'goal_checker_plugins': ['general_goal_checker'],
            'general_goal_checker': {
                'plugin': 'nav2_controller::SimpleGoalChecker',
                'xy_goal_tolerance': 0.25,
                'yaw_goal_tolerance': 0.25,
            },
            
            # 2. FIX: Parámetros del Goal Checker por defecto (elimina el WARN)
            'current_goal_checker': 'general_goal_checker',

            # Lista de plugins
            'controller_plugins': ['FollowPath'],
            
            # Parámetros generales del controlador
            'odom_topic': '/odom',
            'base_frame_id': 'base_link',
            
            # 3. FIX: Umbrales de velocidad (CRÍTICO para DWB/evitar SegFault)
            'min_x_velocity_threshold': 0.001,
            'min_theta_velocity_threshold': 0.001,
            
            # Plugin DWB como FollowPath
            'FollowPath': {
                'plugin': 'dwb_core::DWBLocalPlanner',
                
                # Velocidades
                'min_vel_x': 0.0,
                'max_vel_x': 0.5,
                'min_vel_theta': -1.0,
                'max_vel_theta': 1.0,
                
                # Muestras
                'vx_samples': 10,
                'vtheta_samples': 20,
                
                # Aceleraciones
                'acc_lim_x': 1.0,
                'acc_lim_theta': 2.0,
                
                # Huella
                'footprint_model': {
                    'type': 'circle',
                    'radius': 0.22
                },
                
                # ❗ Critic plugins (Restauramos críticos geométricos para estabilidad)
                'critics': [
                    'PathAlign',
                    'GoalAlign',
                    'PathDist',
                    'GoalDist'
                ]
            }
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('odom', '/odom'),
            # REMOVED: ('plan', '/path') --> Ya no es necesario si usas la acción FollowPath
        ]
    )

    # ===========================
    # NAV2: LIFECYCLE MANAGER (Orquestador)
    # ===========================
    nav2_nodes_to_manage = [
        'map_server', 
        #'amcl', 
        'planner_server', 
        'controller_server'
    ]
    
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': nav2_nodes_to_manage}
        ]
    )


    # ===========================
    # RETURN
    # ===========================
    return LaunchDescription([
        use_sim_time_arg,
        
        # Sistemas de bajo nivel
        cloud_tf,
        footprint_tf,
        robot_state_pub,
        joint_state_pub,
        sick_node,
        serial_node,
        controller_node,
        traslate_node,
        tracking_node,
        planner_node,
        hector_slam,

        # Nodos Nav2
        map_server,
        #amcl,
        planner_server,
        controller_server,
        
        # Orquestador Nav2
        lifecycle_manager
    ])