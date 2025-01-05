import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('msiclaw_teleop_twist_joy'), 
        'config', 
        'msiclaw_joy_config.yaml')
    msiclaw_joy_node = Node(
        package='msiclaw_teleop_twist_joy',
        executable='msiclaw_joy',
        output='screen',
        )
    ld = LaunchDescription()
    ld.add_action(msiclaw_joy_node)
    return ld
