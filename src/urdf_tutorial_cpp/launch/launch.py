import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf = os.path.join(get_package_share_directory('urdf_tutorial_cpp'), 'urdf', 'r2d2.urdf.xml')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
             output='screen',
             parameters=[{'user_sim_time': use_sim_time, 'robot_description': robot_desc}],
             arguments=[urdf]),
        Node(
            package='urdf_tutorial_cpp',
            executable='urdf_tutorial_cpp',
            name='urdf_tutorial_cpp',
            output='screen'
        )
    ])