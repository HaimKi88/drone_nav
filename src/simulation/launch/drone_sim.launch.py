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
    default_rviz_config_path = os.path.join(sim_pkg_share, 'rviz/urdf_config.rviz')

    gz_server_launch = launch_description_sources.PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')])
    
    gz_client_launch = launch_description_sources.PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')])
    

    return LaunchDescription([
        DeclareLaunchArgument(name='gazebo', default_value='false'),
        DeclareLaunchArgument(name='rviz', default_value='false'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                              description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='respawns', default_value='True'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        
        # IncludeLaunchDescription(gz_server_launch, 
        #                          launch_arguments={'world': os.path.join(get_package_share_directory('simulation'), 'world', 'empty-world.sdf')}.items()),
        IncludeLaunchDescription(gz_client_launch,
                                 condition=IfCondition(LaunchConfiguration('gazebo'))),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            prefix='bash -c "sleep 1; $0 $@"',
            respawn=LaunchConfiguration('respawns'),
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
            # arguments=['-entity', 'nakai_robot', '-topic',
            #         'robot_description', 
            #         '-x', '0', '-y', '0', '-z', '0',
            #             '-R', '0', '-P', '0', '-Y', '0'],
        #     output='log',
        # ),

        Node(
            package='navigation',
            executable='nav',
            name='nav',
            output='log',
        ),

    ])
