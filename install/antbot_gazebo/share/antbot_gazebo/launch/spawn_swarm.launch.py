import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_antbot_description = get_package_share_directory('antbot_description')
    pkg_antbot_gazebo = get_package_share_directory('antbot_gazebo')

    world_path = os.path.join(pkg_antbot_gazebo, 'worlds', 'arena.world')
    urdf_file = os.path.join(pkg_antbot_description, 'urdf', 'antbot.urdf.xacro')

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

    # ── Spawn positions inside the 2×2 m arena (interior: ±1.0 m) ──────────
    # Bots start spread in the bottom-left quadrant so they don't overlap
    robots = [
        {'name': 'bot1', 'x': '-0.55', 'y': '-0.55', 'z': '0.1'},
        {'name': 'bot2', 'x':  '0.55', 'y':  '0.0',  'z': '0.1'},
        {'name': 'bot3', 'x':  '0.0',  'y':  '0.0',  'z': '0.1'},
    ]

    spawn_actions = []

    robot_description_param = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

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

        spawn_actions.append(robot_state_publisher)
        spawn_actions.append(spawn_entity)

    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    delay_spawn = TimerAction(period=5.0, actions=spawn_actions)
    ld.add_action(delay_spawn)

    return ld
