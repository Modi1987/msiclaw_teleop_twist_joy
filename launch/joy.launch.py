import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_dir = get_package_share_directory('msiclaw_teleop_twist_joy')
    config_file = os.path.join(package_share_dir, 'config', 'msiclaw_joy_config.yaml')
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Configuration file not found: {config_file}")

    msiclaw_joy_node = Node(
        package='msiclaw_teleop_twist_joy',
        executable='msiclaw_joy',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('msiclaw_joy', 'handheld_joy') # published by the node
        ],
    )
    ld = LaunchDescription()
    ld.add_action(msiclaw_joy_node)
    
    return ld
