import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler, IncludeLaunchDescription, ExecuteProcess)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    bringup_pkg = get_package_share_directory('saye_bringup')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    rviz_config = LaunchConfiguration('rviz_config')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')

    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

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

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(bringup_pkg, 'config', 'slam.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_rviz = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_pkg, 'rviz', 'navigation.rviz'),
        description='RViz configuration'
    )

    # Spawn simulation and robot
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

    # SLAM launch using online_sync_launch.py
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

    # Map saver
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

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config],
        output='screen'
    )

    # TF chain: map -> odom (from SLAM) -> saye/base_link (from odometry)
    # Publish odom -> saye/base_link transform from odometry messages
    installed_script = os.path.join(bringup_pkg, '..', '..', 'lib', 'saye_bringup', 'odom_to_tf.py')
    source_script = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(bringup_pkg))),
        'src', 'ackermann-vehicle-gzsim-ros2', 'saye_bringup', 'scripts', 'odom_to_tf.py'
    )
    odom_script = installed_script if os.path.exists(installed_script) else source_script
    odom_to_tf_node = ExecuteProcess(
        cmd=['python3', odom_script,
             '--ros-args',
             '-p', 'base_frame:=saye/base_link',
             '-p', 'odom_frame:=odom',
             '-p', 'use_sim_time:=true'],
        name='odom_to_tf',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_gui)
    ld.add_action(declare_world)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_rviz)
    ld.add_action(spawn_sim)  # Spawn simulation and robot
    ld.add_action(slam_launch)  # Launch SLAM
    ld.add_action(odom_to_tf_node)  # Publish odom->saye/base_link transform
    ld.add_action(map_saver)
    ld.add_action(map_saver_lifecycle)
    ld.add_action(rviz_node)

    return ld
