import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Obtener rutas
    stage_pkg_share = get_package_share_directory('stage_ros2')
    world_path = os.path.join(stage_pkg_share, 'world', 'cave.world')

    return LaunchDescription([
        # 1. EL SIMULADOR (El cuerpo)
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            output='screen',
            parameters=[{'world_file': world_path}],
            remappings=[('/base_scan', '/scan')] 
        ),

        # 2. EL RESET (El teletransportador)
        Node(
            package='dqn_robot_nav',
            executable='reset_stage',
            name='odom_reset_wrapper',
            output='screen'
        ),
        
        # 3. TU SCRIPT DE HISTORIA (El cerebro que analiza)
        # Asegúrate de que el ejecutable coincida con como lo llamaste en setup.py
        # Si no lo pusiste en setup.py, este nodo fallará y deberás usar python3
        Node(
            package='dqn_robot_nav',
            executable='reconstruct_history', 
            name='history_reconstructor',
            output='screen'
        ),
    ])