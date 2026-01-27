import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Configuración de rutas
    stage_pkg_share = get_package_share_directory('stage_ros2')
    # Usamos el mapa por defecto 'cave.world'
    world_path = os.path.join(stage_pkg_share, 'world', 'cave.world')

    return LaunchDescription([
        # ---------------------------------------------------------
        # NODO 1: SIMULADOR STAGE
        # ---------------------------------------------------------
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            output='screen',
            parameters=[{'world_file': world_path}],
            # IMPORTANTE: Remapeo para que el robot lea /scan en lugar de /base_scan
            remappings=[('/base_scan', '/scan')] 
        ),

        # ---------------------------------------------------------
        # NODO 2: WRAPPER DE RESET (NECESARIO)
        # ---------------------------------------------------------
        # Este nodo es vital para que env.reset() funcione sin romper la odometría
        Node(
            package='dqn_robot_nav',
            executable='reset_stage',
            name='odom_reset_wrapper',
            output='screen'
        ),
    ])