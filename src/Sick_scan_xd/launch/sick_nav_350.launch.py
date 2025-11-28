import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ld = LaunchDescription()
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    launchfile = os.path.basename(__file__)[:-3] # convert "<lidar_name>.launch.py" to "<lidar_name>.launch"
    launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/' + launchfile) # 'launch/sick_nav_350.launch')
    node_arguments=[launch_file_path]
    
    # append optional commandline arguments in name:=value syntax
    for arg in sys.argv:
        if len(arg.split(":=")) == 2:
            node_arguments.append(arg)
    
    ROS_DISTRO = os.environ.get('ROS_DISTRO') # i.e. 'eloquent', 'foxy', etc.
    if ROS_DISTRO[0] <= "e": # ROS versions eloquent and earlier require "node_executable", ROS foxy and later use "executable"
        node = Node(
            package='sick_scan_xd',
            node_executable='sick_generic_caller',
            output='log',
            remappings=[ ('/sick_nav_350/scan', '/scan'), ],
            arguments=node_arguments,
                        parameters=[{
                "use_odom": False,
                "publish_odom_tf": False,
                "publish_odom": False,
                "output_odom_topic": "",
            }]
        )
    else: 
        node = Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            output='log',
            remappings=[ ('/sick_nav_350/scan', '/scan'), ],
            arguments=node_arguments,
            parameters=[{
                "use_odom": False,
                "publish_odom_tf": False,
                "publish_odom": False,
                "output_odom_topic": "",
                
                # Deshabilitar si publica una TF de cloud (a veces es cloud_transform)
                "cloud_transform": False, # <--- CAMBIO CRÍTICO (Desactivar si es posible)

                # Forzar la coherencia: el scanner_frame será el frame_id real
                "scanner_frame": "cloud",  # <--- CORREGIDO: Usar 'cloud'
                
                # A veces, este es el parámetro que realmente controla el frame_id en el header:
                "frame_id": "cloud", # <--- Añadir este por seguridad
                
                "publish_laserscan": True,
                "use_published_timestamp": False
            }]
        )
    
    ld.add_action(node)
    return ld