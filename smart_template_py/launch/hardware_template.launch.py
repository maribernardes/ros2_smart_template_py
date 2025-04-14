from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        description='URDF description of the robot'
    )

    # Use smart template node
    robot = Node(
        package="smart_template_py",
        executable="template",
        parameters=[{
            "robot_description": LaunchConfiguration("robot_description")
        }]
    )  

    return LaunchDescription([
        robot_description_arg,
        robot
    ])
