import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tyashu/Swarm-System-Gazebo-simulation-main/Antbots/install/antbot_navigation'
