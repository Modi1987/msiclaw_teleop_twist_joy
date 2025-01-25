import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_filepath = os.path.join(
        get_package_share_directory('msiclaw_teleop_twist_joy'),
        'config',
        'xbox.config.yaml'
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            config_filepath,
            {'publish_stamped_twist': False}
        ],
        remappings=[
            ('/cmd_vel', 'input/cmd_vel_teleop_msiclaw_joy'),
            ('joy', 'handheld_joy')
        ],
    )

    msi_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("handheld_teleop_twist_joy"),
                "launch/joy.launch.py",
            )
        )
    )

    return launch.LaunchDescription([
        teleop_twist_joy_node,
        msi_joy_launch
    ])
