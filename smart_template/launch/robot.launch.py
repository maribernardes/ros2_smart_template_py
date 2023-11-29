import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, conditions
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Launch robot (hardware or virtual version)

def generate_launch_description():

    ld = LaunchDescription()

    arg_sim_level = DeclareLaunchArgument(
        "sim_level",
        default_value = "2",
        description = "virtual = 1, real = 2"
    )  

    launch_directory = os.path.join(get_package_share_directory('smart_template'), 'launch')
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_directory, 'hardware_template.launch.py')),
        condition=conditions.IfCondition(
            PythonExpression([LaunchConfiguration('sim_level'), " == 2"])
        )
    )
    virtual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_directory, 'virtual_template.launch.py')),
        condition=conditions.IfCondition(
            PythonExpression([LaunchConfiguration('sim_level'), " == 1"])
        )
    )

    # Include launch arguments
    ld.add_action(arg_sim_level)
    
    ld.add_action(hardware_launch)
    ld.add_action(virtual_launch)
    
    return ld
