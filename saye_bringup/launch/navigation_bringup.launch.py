import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_saye_bringup = get_package_share_directory('saye_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')
    map_yaml = LaunchConfiguration('map', default=os.path.join(pkg_saye_bringup, 'maps', 'map.yaml'))

    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_saye_bringup, 'maps', 'map.yaml'),
        description='Full path to a Nav2 map YAML file'
    )

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': map_yaml,
            'params_file': os.path.join(pkg_saye_bringup, 'config', 'nav2_params.yaml'),
            'package_path': pkg_saye_bringup, 
        }.items()
    )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d' + os.path.join(
                pkg_saye_bringup,
                'rviz',
                'navigation.rviz'
            )
        ]
    )
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(pkg_saye_bringup, 'config', 'amcl.yaml')],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml}],
    )




    ld = LaunchDescription()

    ld.add_action(declare_map_arg)
    ld.add_action(nav2_launch_cmd)
    ld.add_action(rviz_launch_cmd)
    ld.add_action(amcl_node)
    ld.add_action(map_server_node)
    # Do not publish a static map->odom. AMCL provides map->odom during navigation.

    return ld
