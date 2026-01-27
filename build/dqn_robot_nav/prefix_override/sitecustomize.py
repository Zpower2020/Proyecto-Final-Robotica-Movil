import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mateozc/Diplomado/Lab2/install/dqn_robot_nav'
