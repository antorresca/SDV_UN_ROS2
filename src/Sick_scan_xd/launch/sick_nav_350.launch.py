import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ld = LaunchDescription()
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    
    # --- 1. Calcular el camino al launch file interno (Necesario para la configuración del sensor) ---
    # Esto se resuelve a algo como 'launch/sick_nav_350.launch'
    launchfile = os.path.basename(__file__)[:-3] 
    launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/' + launchfile) 
    
    # --- 2. Crear argumentos, incluyendo el launch file interno ---
    node_arguments=[launch_file_path] 
    
    # Añadir argumentos opcionales
    for arg in sys.argv:
        if len(arg.split(":=")) == 2:
            node_arguments.append(arg)
    
    # --- 3. Definición del Nodo (Bloque de Humble) ---
    # Asumimos Humble (ROS_DISTRO > "e")
    node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        output='log',
        remappings=[ ('/sick_nav_350/scan', '/scan'), ],
        arguments=node_arguments, # Se pasa el launch file interno
        parameters=[{
                
                # ... (Parámetros de Odometría)
                "use_odom": False,
                "publish_odom_tf": False,
                "publish_odom": False,
                "output_odom_topic": "",
                
                # --- Parámetros Críticos de TF para Desactivación (NUEVO) ---
                
                # CRÍTICO: Sobreescribe el argumento tf_publish_rate del launch interno a 0.0 Hz.
                # Esto desactiva la publicación de la TF map -> cloud.
                "tf_publish_rate": 0.0, # <--- ¡CLAVE!

                # Desactivación forzada (por si acaso el driver no respeta el tf_publish_rate=0.0)
                "cloud_transform": False, 
                "use_tf": False,          
                
                # Frame ID para el mensaje /scan (Debe ser 'cloud')
                "scanner_frame": "cloud",
                "frame_id": "cloud", 

                # Otros parámetros (mantener si es necesario)
                "publish_laserscan": True,
                "use_published_timestamp": False
            }]
    )
    
    ld.add_action(node)
    return ld