import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


from launch_ros.actions import Node


def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    publish_stamped_twist = launch.substitutions.LaunchConfiguration('publish_stamped_twist')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_vel', default_value='cmd_vel_teleop'),
        launch.actions.DeclareLaunchArgument('joy_config', default_value='xbox'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='0'),
        launch.actions.DeclareLaunchArgument('publish_stamped_twist', default_value='false'),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('msiclaw_teleop_twist_joy'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),
        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_filepath, {'publish_stamped_twist': publish_stamped_twist}],
            remappings={
                ('/cmd_vel', "input/cmd_vel_teleop_msiclaw_joy"),
                ("joy", "msiclaw_joy")
                },
            ),
    ])