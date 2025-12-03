from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    # ---------------------------
    # PARAMETROS DE MAP SERVER
    # ---------------------------
    map_server_params = {
        "yaml_filename": "/home/ros/maps/map.yaml",
        "use_sim_time": False
    }

    # ---------------------------
    # PARAMETROS DE AMCL
    # ---------------------------
    amcl_params = {
        "use_sim_time": False,
        "alpha1": 0.2,
        "alpha2": 0.2,
        "alpha3": 0.2,
        "alpha4": 0.2,
        "alpha5": 0.2,
        "base_frame_id": "base_link",
        "odom_frame_id": "odom",
        "global_frame_id": "map",
        "scan_topic": "/scan",
        "min_particles": 300,
        "max_particles": 2000,
        "update_min_d": 0.1,
        "update_min_a": 0.1
    }

    # ---------------------------
    # PARAMETROS DEL CONTROLLER SERVER (RPP + DWB)
    # ---------------------------
    controller_params = {
        "use_sim_time": False,
        "controller_frequency": 10.0,
        "FollowPath": {
            "plugin": "dwb_core::DWBLocalPlanner",

            # LISTA DE CRITICS (OBLIGATORIA)
            "critics": [
                "RotateToGoal",
                "ObstacleFootprint",
                "GoalDist",
                "PathAlign",
                "PathDist",
                "Twirling"
            ],

            # VELOCIDADES QUE YA CONTROLAS PERO NAV2 TAMBIÉN DEBE SABER
            "max_vel_x": 0.3,
            "min_vel_x": -0.3,
            "max_vel_theta": 1.0,
            "min_vel_theta": -1.0,
            "acc_lim_x": 1.0,
            "acc_lim_theta": 2.0,

            # OTROS
            "xy_goal_tolerance": 0.20,
            "yaw_goal_tolerance": 0.15,
        },

        "local_costmap": {
            "local_costmap": {
                "ros__parameters": {
                    "use_sim_time": False,
                    "global_frame": "map",
                    "robot_base_frame": "base_link",
                    "update_frequency": 10.0,
                    "publish_frequency": 10.0,
                    "rolling_window": True,
                    "width": 5.0,
                    "height": 5.0,
                    "resolution": 0.05,
                    "plugins": ["static_layer", "obstacle_layer", "inflation_layer"],
                    "static_layer": {"map_topic": "/map"},
                    "obstacle_layer": {"observation_sources": "scan", "scan": {"topic": "/scan", "clearing": True, "marking": True}},
                }
            }
        }
    }

    # ---------------------------
    # NODOS
    # ---------------------------

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[map_server_params]
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_params]
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        remappings=[
            ("/cmd_vel", "/cmd_vel"),
            ("/global_plan", "/a_star/path"),
            ("/odom", "/odom")
        ],
        parameters=[controller_params]
    )

    # ---------------------------
    # LIFECYCLE MANAGER
    # ---------------------------
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": False,
            "node_names": ["map_server", "amcl", "controller_server"]
        }]
    )

    # ---------------------------
    # LIFECYCLE COMMANDS
    # ---------------------------
    lifecycle_cmds = [
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="ros2cli",
                    executable="ros2",
                    arguments=["lifecycle", "set", "/map_server", "configure"],
                    output="screen"
                )
            ]
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="ros2cli",
                    executable="ros2",
                    arguments=["lifecycle", "set", "/map_server", "activate"],
                    output="screen"
                )
            ]
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="ros2cli",
                    executable="ros2",
                    arguments=["lifecycle", "set", "/amcl", "configure"],
                    output="screen"
                )
            ]
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="ros2cli",
                    executable="ros2",
                    arguments=["lifecycle", "set", "/amcl", "activate"],
                    output="screen"
                )
            ]
        ),
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package="ros2cli",
                    executable="ros2",
                    arguments=["lifecycle", "set", "/controller_server", "configure"],
                    output="screen"
                )
            ]
        ),
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package="ros2cli",
                    executable="ros2",
                    arguments=["lifecycle", "set", "/controller_server", "activate"],
                    output="screen"
                )
            ]
        )
    ]

    return LaunchDescription([
        map_server,
        amcl,
        controller_server,
        lifecycle_manager,
        *lifecycle_cmds
    ])
