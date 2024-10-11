import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, actions
from launch.actions import DeclareLaunchArgument

# Launch stage control in mcp mode
# Remember to launch bridge server in another terminal
# Remember to run keypress node (trajcontrol package) in another terminal

def generate_launch_description():

    # Use smart template node
    robot = Node(
        package="smart_template",
        executable="template",
    )  

    return LaunchDescription([
        robot
    ])
