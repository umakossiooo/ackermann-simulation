import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('saye_bringup')
    pkg_project_localization = get_package_share_directory('saye_localization')
    pkg_project_description = get_package_share_directory('saye_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    

    # Path to the SDF file in the description package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'saye', 'model.sdf')
    urdf_file = os.path.join(pkg_project_description, 'urdf', 'saye.urdf')

    with open(urdf_file, 'r') as urdf_handle:
        robot_description_content = urdf_handle.read()

    # World selection (default: bari_world.sdf)
    world_arg = DeclareLaunchArgument(
        'world', default_value='bari_world.sdf',
        description='World file name under saye_description/worlds'
    )

    # Advanced: override full gz sim argument (absolute world path)
    gz_args_arg = DeclareLaunchArgument(
        'gz_args',
        default_value=PathJoinSubstitution([
            pkg_project_description,
            'worlds',
            LaunchConfiguration('world')
        ]),
        description='Full argument passed to gz_sim.launch.py (e.g., absolute world path)'
    )

    # Whether to launch the Gazebo GUI; disable for headless stability in containers
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Launch Gazebo GUI (false for headless)'
    )

    # Robot initial pose (map frame). Override at launch time as needed.
    robot_x_arg = DeclareLaunchArgument('robot_x', default_value='0.0', description='Robot X in meters')
    # Default spawn nudged sideways so we start clear of nearby buildings.
    robot_y_arg = DeclareLaunchArgument('robot_y', default_value='-6.0', description='Robot Y in meters')
    robot_z_arg = DeclareLaunchArgument('robot_z', default_value='0.35', description='Robot Z in meters')
    robot_R_arg = DeclareLaunchArgument('robot_R', default_value='0.0', description='Robot roll in radians')
    robot_P_arg = DeclareLaunchArgument('robot_P', default_value='0.0', description='Robot pitch in radians')
    robot_Y_arg = DeclareLaunchArgument('robot_Y', default_value='0.0', description='Robot yaw in radians')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': LaunchConfiguration('gz_args'),
            'gui': LaunchConfiguration('gui')
        }.items(),
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'rviz', 'saye.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    bridge = Node(
    # Bridge ROS topics and Gazebo messages for establishing communication
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_content},
            {'frame_prefix': 'saye/'}
        ],
        output='screen'
    )
    # Spawn directly from the SDF file to avoid SDF->URDF conversion issues
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', sdf_file,
            '-name', 'saye',
            '-x', LaunchConfiguration('robot_x'),
            '-y', LaunchConfiguration('robot_y'),
            '-z', LaunchConfiguration('robot_z'),
            '-R', LaunchConfiguration('robot_R'),
            '-P', LaunchConfiguration('robot_P'),
            '-Y', LaunchConfiguration('robot_Y')
        ]
    )
    return LaunchDescription([
        world_arg,
        gz_args_arg,
        gui_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,
        robot_R_arg,
        robot_P_arg,
        robot_Y_arg,
        gz_sim,
        robot_state_publisher,
        gz_spawn_entity,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        bridge,
        rviz
    ])
