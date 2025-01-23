import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    msiclaw_joy_teleop_manager_node = Node(
        package='msiclaw_teleop_twist_joy',
        executable='msiclaw_joy_teleop_manager',
        output='screen',
    )
    ld = LaunchDescription()
    ld.add_action(msiclaw_joy_teleop_manager_node)
    
    return ld
