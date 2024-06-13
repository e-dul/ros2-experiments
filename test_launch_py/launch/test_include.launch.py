from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
 
def generate_launch_description():
    ld = LaunchDescription()
    node_name = LaunchConfiguration('node_name')
    node_name_launch_arg = DeclareLaunchArgument('node_name',
                                                 default_value='turtle_launch_arg')
    ld.add_action(node_name_launch_arg)

    turtle = Node(package="turtlesim", executable="turtlesim_node", name=node_name)

    ld.add_action(turtle)

    return ld