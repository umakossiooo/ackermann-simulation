"""Launch file for single robot DRL training in Bari city."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for DRL training."""
    
    # Get package directories
    pkg_saye_bringup = get_package_share_directory('saye_bringup')
    pkg_saye_description = get_package_share_directory('saye_description')
    pkg_ackermann_drl = get_package_share_directory('ackermann_drl')
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='bari_world.sdf',
        description='World file name'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Launch Gazebo GUI'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    # Robot spawn position (default from OSM spawn_point_173)
    robot_x_arg = DeclareLaunchArgument(
        'robot_x',
        default_value='5.55',
        description='Robot X position (east, meters)'
    )
    robot_y_arg = DeclareLaunchArgument(
        'robot_y',
        default_value='-94.69',
        description='Robot Y position (north, meters)'
    )
    robot_z_arg = DeclareLaunchArgument(
        'robot_z',
        default_value='0.35',
        description='Robot Z position (height, meters)'
    )
    robot_Y_arg = DeclareLaunchArgument(
        'robot_Y',
        default_value='-1.5064',
        description='Robot yaw (radians)'
    )
    
    # Include saye_spawn launch to spawn the robot
    saye_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_saye_bringup, 'launch', 'saye_spawn.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'robot_x': LaunchConfiguration('robot_x'),
            'robot_y': LaunchConfiguration('robot_y'),
            'robot_z': LaunchConfiguration('robot_z'),
            'robot_Y': LaunchConfiguration('robot_Y'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )
    
    return LaunchDescription([
        world_arg,
        gui_arg,
        rviz_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,
        robot_Y_arg,
        saye_spawn,
    ])

