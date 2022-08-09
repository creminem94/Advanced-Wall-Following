import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import launch_ros
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
# Need master or Galactic branch for this feature
from launch_ros.descriptions import ParameterValue
import xacro


def generate_launch_description():
    package_name = 'advanced_wall_following'

    package_dir = get_package_share_directory(package_name)
    path_to_urdf = get_package_share_path(
        'turtlebot3_description') / 'urdf' / 'turtlebot3_burger.urdf'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(package_dir, 'rviz', 'AWF.rviz')],
    )
    robot_state_cmd = LaunchConfiguration('robot_state_cmd')

    ld = LaunchDescription([

        DeclareLaunchArgument(
            'robot_state_cmd',
            default_value='True',
            description='If robot state publisher is in NAV2 launch'),

        Node(
            condition=IfCondition(robot_state_cmd),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', str(path_to_urdf)]), value_type=str
                )
            }]
        ),
        # Node(package="tf2_ros",
        #      executable="static_transform_publisher",
        #      arguments=["0", "0", "0", "0", "0", "0", "0", "base_link", "map"]),
        # Node(package="tf2_ros",
        #      executable="static_transform_publisher",
        #      arguments=["-0.032", "0", "0.172", "0", "0", "0", "1", "base_link", "base_scan"]),
    ])

    ld.add_action(rviz_node)

    return ld
