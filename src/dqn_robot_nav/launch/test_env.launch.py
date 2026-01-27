import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Usaremos un mundo por defecto de stage_ros2
    stage_pkg_share = get_package_share_directory('stage_ros2')
    
    # Ruta al mundo (cave.world suele venir por defecto)
    world_path = os.path.join(stage_pkg_share, 'world', 'cave.world')

    return LaunchDescription([
        # 1. NODO DEL SIMULADOR (STAGE)
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            output='screen',
            parameters=[{'world_file': world_path}],
            # Remapeo vital para que tu código lea /scan
            remappings=[('/base_scan', '/scan')] 
        ),

        # 2. NODO WRAPPER DE RESET (Para evitar problemas de odometría)
        Node(
            package='dqn_robot_nav',
            executable='reset_stage',
            name='odom_reset_wrapper',
            output='screen'
        ),
        
        # 3. NODO DE TEST (EVALUACIÓN)
        # Este nodo cargará el modelo 'dqn_model_500.pkl' y correrá la prueba
        Node(
            package='dqn_robot_nav',
            executable='test_node',
            name='dqn_tester',
            output='screen'
        ),
    ])
