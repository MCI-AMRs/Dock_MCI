from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument(
        'description',
        default_value='false',
        description='Launch turtlebot4 description'
    ),
    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    ),
]

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        Node(
            package='dock_action',
            namespace='turtlesim1',
            executable='dock_action_node',
            name='sim'
        )
    ])