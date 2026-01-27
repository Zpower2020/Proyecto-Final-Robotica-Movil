from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dqn_robot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ESTO ES CRUCIAL: Instalar los launch files para que Stage funcione
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mateozc',
    maintainer_email='mateozc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Solo ponemos los Nodos ejecutables aquí.
            # Los scripts de ayuda (agente, procesador) se importan, no se ejecutan solos.
            
            'train_node = dqn_robot_nav.train_node:main',
            'test_node = dqn_robot_nav.test_node:main',
            'reset_stage = dqn_robot_nav.reset_stage:main',
            'reconstruct_history = dqn_robot_nav.reconstruct_history:main',
            
        ],
    },
)