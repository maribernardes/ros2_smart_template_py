import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, conditions
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import PythonExpression, Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch.actions import TimerAction


# Launch robot (hardware or virtual version)

def generate_launch_description():

    ld = LaunchDescription()

    arg_needle_type = DeclareLaunchArgument(
        'needle_type',
        default_value = 'default',
        description = 'default / stylet'
    )  

    arg_sim_level = DeclareLaunchArgument(
        'sim_level',
        default_value = '2',
        description = 'virtual = 1, real hardware = 2'
    )  

    arg_rviz = DeclareLaunchArgument(
        'rviz', 
        default_value = 'false', 
        choices = ['true', 'false'],
        description = 'Start RViz automatically'
    )

    arg_gui = DeclareLaunchArgument(
        'gui', 
        default_value = 'true', 
        choices = ['true', 'false'],
        description = 'Start SmartTemplate GUI plugin automatically'
    )

    arg_description_package = DeclareLaunchArgument(
        'description_package',
        default_value = 'smart_template_description',
        description = 'Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.',
    )

    arg_description_file = DeclareLaunchArgument(
        'description_file',
        default_value = 'smart_template.urdf.xacro',
        description = 'URDF/XACRO description file with the robot'
    )
    
    arg_name = DeclareLaunchArgument(
        'name',
        default_value = 'smart_template',
        description = 'Name of the robot system'
    )
    
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    rviz_file = PathJoinSubstitution([FindPackageShare(description_package), 'rviz', 'urdf.rviz'])
    rate = LaunchConfiguration('rate', default=50.0)  # Hz, default is 10 so we're increasing that a bit. 
    # Funny enough joint and robot state publishers don't have the same name for that parameter :-(
    
    # Select hardware or virtual robot
    launch_directory = os.path.join(get_package_share_directory('smart_template'), 'launch')
    robot_real_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_directory, 'hardware_template.launch.py')),
        condition=conditions.IfCondition(
            PythonExpression([LaunchConfiguration('sim_level'), " == 2"])
        )
    )
    robot_virtual_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_directory, 'virtual_template.launch.py')),
        condition=conditions.IfCondition(
            PythonExpression([LaunchConfiguration('sim_level'), " == 1"])
        )
    )

    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), 'urdf', description_file]),
        " ",
        "name:=", LaunchConfiguration('name'),
        " ",
        "needle_type:=", LaunchConfiguration('needle_type'),
    ])
    robot_description = {
        "robot_description": robot_description_content, 'publish_frequency': rate}

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        arguments=[description_file]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments = ['-d', rviz_file],
        condition = IfCondition(LaunchConfiguration('rviz'))
    )

    # # RQt GUI plugin conditionally
    # gui_plugin = ExecuteProcess(
    #     condition=IfCondition(LaunchConfiguration('gui')),
    #     cmd=['rqt', '--standalone', 'smart_template_gui'],
    #     output='screen'
    # )

    # Event handler to launch the GUI plugin after the robot_state_publisher is started
    gui_plugin_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[
                ExecuteProcess(
                    condition=IfCondition(LaunchConfiguration('gui')),
                    cmd=['rqt', '--standalone', 'smart_template_gui'],
                    output='screen'
                )
            ]
        )
    )

    # # Event handler to launch the GUI plugin after the robot_state_publisher is started
    # gui_plugin_event_handler = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=robot_state_publisher_node,
    #         on_start=[
    #             TimerAction(
    #                 period=10.0,  # Delay in seconds
    #                 actions=[
    #                     ExecuteProcess(
    #                         condition=IfCondition(LaunchConfiguration('gui')),
    #                         cmd=['rqt', '--standalone', 'smart_template_gui'],
    #                         output='screen'
    #                     )
    #                 ]
    #             )
    #         ]
    #     )
    # )


    # Include launch arguments
    ld.add_action(arg_needle_type)
    ld.add_action(arg_sim_level)
    ld.add_action(arg_rviz)
    ld.add_action(arg_gui)
    ld.add_action(arg_description_package)
    ld.add_action(arg_description_file)
    ld.add_action(arg_name)
    
    #Nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(robot_real_hardware_launch)
    ld.add_action(robot_virtual_launch)
    ld.add_action(rviz_node)
    ld.add_action(gui_plugin_event_handler)

    return ld
