import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Usaremos un mundo por defecto de stage_ros2 o uno simple
    # Stage necesita un archivo .world
    stage_pkg_share = get_package_share_directory('stage_ros2')
    
    # El paquete stage_ros2 suele traer ejemplos en share/stage_ros2/world
    # Vamos a usar 'cave.world' o 'willow-erratic.world' como base, 
    # pero para RL es mejor un mapa vacío o con obstaculos simples.
    
    # SI NO TIENES UN ARCHIVO .world PROPIO:
    # Usa uno de los que vienen por defecto.
    world_path = os.path.join(stage_pkg_share, 'world', 'cave.world')

    return LaunchDescription([
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            output='screen',
            parameters=[{'world_file': world_path}],
            # Remapeos si son necesarios (Stage suele usar /base_scan)
            remappings=[('/base_scan', '/scan')] # Si haces esto, NO cambies el código en environment.py
        ),

        # AÑADIR ESTE NODO: El Wrapper de Reset
        Node(
            package='dqn_robot_nav',
            executable='reset_stage',
            name='odom_reset_wrapper',
            output='screen'
        ),
        
        # Opcional: Publicador de TF estático si Stage no une odom->base_link correctamente
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        # )
    ])