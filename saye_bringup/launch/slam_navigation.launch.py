import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('saye_bringup')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically transition lifecycle nodes'
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo Harmonic GUI'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='bari_world.sdf',
        description='World name under saye_description/worlds'
    )

    declare_nav2_params = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(bringup_pkg, 'config', 'nav2_params.yaml'),
        description='Nav2 parameter file'
    )

    declare_slam_params = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(bringup_pkg, 'config', 'slam.yaml'),
        description='slam_toolbox synchronous mapping parameters'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_pkg, 'rviz', 'navigation.rviz'),
        description='RViz configuration'
    )

    spawn_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'saye_spawn.launch.py')
        ),
        launch_arguments={
            'world': world,
            'gui': gui,
            'rviz': 'false'
        }.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, 'launch', 'online_sync_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
            'autostart': autostart,
            'use_lifecycle_manager': 'false'
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file,
            'slam': 'False',
            'use_localization': 'False',
            'map': ''
        }.items()
    )

    map_saver = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'save_map_timeout': 5.0,
            'free_thresh_default': 0.25,
            'occupied_thresh_default': 0.65,
            'map_subscribe_transient_local': True
        }]
    )

    map_saver_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_saver']}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config],
        output='screen'
    )

    # TF chain: map -> odom (from SLAM) -> saye/base_link (from odometry)
    # The odometry publisher should publish odom->saye/base_link, but if it doesn't,
    # we need to extract it from the odometry message
    # Try to use the installed script first, fallback to source
    installed_script = os.path.join(bringup_pkg, '..', '..', 'lib', 'saye_bringup', 'odom_to_tf.py')
    source_script = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(bringup_pkg))),
        'src', 'ackermann-vehicle-gzsim-ros2', 'saye_bringup', 'scripts', 'odom_to_tf.py'
    )
    # Use source script if installed doesn't exist
    odom_script = installed_script if os.path.exists(installed_script) else source_script
    odom_to_tf_node = ExecuteProcess(
        cmd=['python3', odom_script,
             '--ros-args',
             '-p', 'base_frame:=saye',  # Publish odom -> saye (robot_state_publisher handles saye -> saye/base_link)
             '-p', 'odom_frame:=odom',
             '-p', 'use_sim_time:=true'],  # Always true for simulation
        name='odom_to_tf',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_gui)
    ld.add_action(declare_world)
    ld.add_action(declare_nav2_params)
    ld.add_action(declare_slam_params)
    ld.add_action(declare_rviz)
    ld.add_action(spawn_sim)
    ld.add_action(slam_launch)
    ld.add_action(odom_to_tf_node)  # Publish odom->saye/base_link from odometry messages
    ld.add_action(nav2_launch)
    ld.add_action(map_saver)
    ld.add_action(map_saver_lifecycle)
    ld.add_action(rviz_node)

    return ld
