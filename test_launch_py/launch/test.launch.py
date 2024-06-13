from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
 
def generate_launch_description():
    ld = LaunchDescription()

    topic_name = LaunchConfiguration('topic_name')
    topic_name_launch_arg = DeclareLaunchArgument('topic_name',
                                                 default_value='talker_test')
    ld.add_action(topic_name_launch_arg)

    # default topic is chatter
    talker = Node(package="demo_nodes_cpp", executable="talker", remappings=[("chatter", topic_name)])

    included_launch = PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('test_launch_py'),
                    'launch',
                    'test_include.launch.py'
                ])
            ]) 
    included_launch_description = included_launch.try_get_launch_description_without_context()
    included_launch_params = included_launch_description.get_launch_arguments_with_include_launch_description_actions()
    for v in included_launch_params:
        print(v[0].name)


    inc = IncludeLaunchDescription(
            included_launch,
            launch_arguments={
                # with arg FWD like this node name provided from terminal doesn't work
                #"node_name": topic_name
            }.items()
        )

    ld.add_action(talker)
    ld.add_action(inc)

    return ld