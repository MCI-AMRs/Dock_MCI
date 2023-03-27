from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription();

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value='',
        description="Namespace for the dock_action_node"
    )
    ld.add_action(namespace_arg)

    set_env_var = SetEnvironmentVariable(
        "ROS_NAMESPACE",
        LaunchConfiguration("namespace")
    )
    ld.add_action(set_env_var)


    dock_action_node = Node(
            package='dock_action',
            namespace=LaunchConfiguration("namespace"),
            executable='dock_action_server'
        )
    ld.add_action(dock_action_node)

    return ld