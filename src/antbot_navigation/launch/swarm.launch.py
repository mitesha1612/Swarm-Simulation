import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_antbot_description = get_package_share_directory('antbot_description')
    pkg_antbot_gazebo = get_package_share_directory('antbot_gazebo')
    pkg_antbot_navigation = get_package_share_directory('antbot_navigation')

    world_path = os.path.join(pkg_antbot_gazebo, 'worlds', 'arena.world')
    urdf_file = os.path.join(pkg_antbot_description, 'urdf', 'antbot.urdf.xacro')

    # 1. Gazebo Server & Client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # ── Spawn positions inside the 2×2 m arena ──
    robots = [
        {'name': 'bot1', 'x': '-0.75', 'y': '-0.75', 'z': '0.1'},
        {'name': 'bot2', 'x': '-0.75', 'y': '-0.45', 'z': '0.1'},
        {'name': 'bot3', 'x': '-0.45', 'y': '-0.75', 'z': '0.1'},
    ]

    delayed_actions = []

    robot_description_param = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # 2. Spawning each bot
    for robot in robots:
        namespace = robot['name']

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'robot_description': robot_description_param,
                'frame_prefix': namespace + '/'
            }]
        )

        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', namespace,
                '-topic', f'/{namespace}/robot_description',
                '-x', robot['x'],
                '-y', robot['y'],
                '-z', robot['z'],
                '-robot_namespace', namespace
            ],
            output='screen'
        )

        delayed_actions.append(robot_state_publisher)
        delayed_actions.append(spawn_entity)

        # 3. Spawn logic node for each bot
        swarm_logic_node = Node(
            package='antbot_navigation',
            executable='swarm_logic',
            name='swarm_logic',
            namespace=namespace,
            parameters=[{'namespace': namespace}],
            output='screen'
        )
        delayed_actions.append(swarm_logic_node)

    # 4. Swarm GUI server
    gui_server_node = Node(
        package='antbot_navigation',
        executable='swarm_gui_server',
        name='swarm_gui_server',
        output='screen'
    )
    delayed_actions.append(gui_server_node)

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Delay all spawning, navigation, and GUI server by 5 seconds to let gazebo start
    delay_all = TimerAction(period=5.0, actions=delayed_actions)
    ld.add_action(delay_all)

    return ld
