import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()
    
    # ⚠️ Importante: Ya no se calcula 'node_arguments' con el launch file interno.
    # Los argumentos se pasan vacíos o solo con aquellos que necesites añadir.
    node_arguments = [] 
    
    # Se mantiene el código para agregar argumentos opcionales desde la línea de comandos
    for arg in sys.argv:
        if len(arg.split(":=")) == 2:
            node_arguments.append(arg)

    # El ROS_DISTRO ya no es necesario si asumimos ROS 2 Humble
    # Vamos directamente al bloque de configuración para Humble
    
    node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='log',
        # Remapeo CRÍTICO: Remapea el tópico por defecto del SICK al estándar /scan
        remappings=[ ('/sick_nav_350/scan', '/scan'), ],
        arguments=node_arguments,
        parameters=[{
            
            # --- Parámetros de Conexión (Asegúrate de que sean correctos para tu sensor) ---
            "scanner_type": "sick_nav_350", # <--- ESENCIAL para que el caller sepa qué sensor es
            "min_scanner_frequency": 10.0,
            "max_scanner_frequency": 10.0,
            # Añade aquí IP y puerto si no son por defecto:
            # "hostname": "192.168.0.1", 
            # "port": "2112", 

            # --- Parámetros Críticos de TF para Integración ---
            # Deshabilita la publicación de TF del driver COMPLETAMENTE
            "use_tf": False, 
            "cloud_transform": False, 
            "publish_odom_tf": False,
            "publish_odom": False,

            # Frame ID para el header del mensaje /scan
            "scanner_frame": "cloud",
            "frame_id": "cloud", 
            
            # --- Otros Parámetros ---
            "publish_laserscan": True,
            "use_published_timestamp": False
        }]
    )
    
    ld.add_action(node)
    return ld