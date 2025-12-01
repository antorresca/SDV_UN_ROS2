import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ld = LaunchDescription()
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    
    # Obtiene el nombre del archivo launch para pasarlo como argumento al caller
    # Esto se resuelve a algo como 'sick_nav_350.launch'
    launchfile = os.path.basename(__file__)[:-3] 
    launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/' + launchfile) 
    node_arguments=[launch_file_path]
    
    # Permite añadir argumentos adicionales desde la línea de comandos
    for arg in sys.argv:
        if len(arg.split(":=")) == 2:
            node_arguments.append(arg)
    
    # --- Parámetros específicos para ROS 2 Humble (Foxy y posteriores) ---
    node = Node(
        package='sick_scan_xd',
        # 'executable' es lo correcto para ROS Foxy/Humble y posteriores
        executable='sick_generic_caller',
        output='log',
        # Remapeo CRÍTICO: remapea el tópico por defecto del SICK al estándar /scan
        remappings=[ ('/sick_nav_350/scan', '/scan'), ],
        arguments=node_arguments,
        parameters=[{
                # --- Parámetros de Odometría (Deshabilitados) ---
                "use_odom": False,
                "publish_odom_tf": False,
                "publish_odom": False,
                "output_odom_topic": "",
                
                # --- Parámetros Críticos de TF para Integración (ACTUALIZADO) ---
                
                # CRÍTICO: Deshabilita cualquier publicación de TF por parte del driver.
                "use_tf": False, # <--- ¡NUEVO Y CRÍTICO!

                # Deshabilita la transformación de la nube de puntos.
                "cloud_transform": False, 

                # Este NO es el frame que conecta con base_link, es el frame donde irán los datos del sensor.
                "scanner_frame": "cloud", 
                "frame_id": "cloud", 
                
                # Parámetro de seguridad, aunque 'use_tf: False' debería bastar.
                "tf_frame_id": "", # <--- ¡NUEVO! Deja vacío o sin sentido.
                
                # --- Otros Parámetros ---
                "publish_laserscan": True,
                "use_published_timestamp": False
            }]
    )
    
    ld.add_action(node)
    return ld