import os
from launch import LaunchDescription, launch_description_sources
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, Command, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    sim_pkg_share = get_package_share_directory('simulation')

    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_pkg_share, 'launch', 'sjtu_drone_bringup.launch.py')
            )
        ),

        Node(
            package='navigation',
            executable='nav',
            name='nav',
            output='log',
        ),

        Node(
            package='simulation',
            executable='target_selection',
            name='target_selection',
            output='log',
        ),


    ])
