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
        name="sdv_tracking_node",
        output="screen"
    )

    planner_node = Node(
        package="sdv_planner",
        executable="a_star_planner",
        name="sdv_planner",
        output="screen"
    )     

    # ===========================
    # NODO: translate
    # ===========================
    traslate_node = Node(
        package="sdv_pruebas",
        executable="sdv_translate",
        name="sdv_translate",
        output="screen"
    )

    # ===========================
    # LASER ODOMETRY (Hector remplacé)
    # ===========================
    laser_scan_matcher = Node(
        package="sdv_scan_matcher",
        executable="sdv_scan_matcher",
        name="sdv_scan_matcher",
        output="screen"
    )

    # ===========================
    # RETURN
    # ===========================
    return LaunchDescription([
        use_sim_time_arg,

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

        # Odometry via Laser Scan Matching
        laser_scan_matcher
    ])
