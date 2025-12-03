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
            "amcl_fixed.yaml"  # Archivo con configuración corregida
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
                {'yaml_filename': map_yaml_path}
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
            parameters=[amcl_params_file],
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
                PathJoinSubstitution([
                    get_package_share_directory('sdv_nav'),
                    'config',
                    'controller_params.yaml'
                ])
            ],
            remappings=[
                ('cmd_vel', '/cmd_vel'),
                ('global_plan', '/a_star/path'),  # ← IMPORTANTE
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

            # =======================
            # CONTROLLER SERVER (RPP)
            # =======================
            TimerAction(
                period=7.0,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', '/controller_server', 'configure'],
                        output='screen'
                    )
                ]
            ),
            TimerAction(
                period=8.0,
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

            # Nav2 Localization
            map_server,
            amcl,
            
            # Lifecycle commands
            *lifecycle_commands
        ])