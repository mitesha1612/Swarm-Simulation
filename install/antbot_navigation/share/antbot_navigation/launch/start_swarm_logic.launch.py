from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    namespaces = ['bot1', 'bot2', 'bot3']

    for ns in namespaces:
        node = Node(
            package='antbot_navigation',
            executable='swarm_logic',
            name='swarm_logic',
            namespace=ns,
            parameters=[{'namespace': ns}],
            output='screen'
        )
        ld.add_action(node)

    return ld
