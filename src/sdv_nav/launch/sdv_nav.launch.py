from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

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
    # TF STATIC ─ base_link → cloud (LIDAR)
    # =======================
    cloud_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser_base',
        output='screen',
        arguments=[
            '0.25', '0.0', '0.0',   # x y z del LIDAR respecto a base_link
            '0', '0', '0',          # roll pitch yaw
            'base_link',            # parent frame
            'cloud'                 # child frame (coincide con scanner_frame del SICK)
        ]
    )

    # =======================
    # URDF / XACRO
    # =======================
    robot_description = {
        'robot_description': Command([
            'xacro ',
            PathJoinSubstitution([
                sdv_description_pkg,
                'xacro',
                'sdv_description.xacro'
            ])
        ])
    }

    # =======================
    # ROBOT STATE PUBLISHER
    # =======================
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # =======================
    # JOINT STATE PUBLISHER
    # =======================
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # =======================
    # SICK LIDAR
    # =======================
    sick_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sick_launch)
    )

    # =======================
    # SERIAL NODE (publica odom → base_link)
    # =======================
    serial_node = Node(
        package='sdv_serial',
        executable='sdv_serial_node',
        name='sdv_serial_node',
        output='screen'
    )

    # =======================
    # CONTROLLER NODE
    # =======================
    controller_node = Node(
        package='sdv_controller',
        executable='sdv_controller_node',
        name='sdv_controller_node',
        output='screen'
    )

    # =======================
    # MAP SERVER (solo para ver el mapa en RViz)
    # =======================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_path}]
    )

    # =======================
    # RETURN
    # =======================
    return LaunchDescription([
        robot_state_pub,
        joint_state_pub,
        cloud_tf,
        sick_node,
        serial_node,
        controller_node,
        map_server
    ])
