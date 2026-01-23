import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Setup project paths
    pkg_project_bringup = get_package_share_directory('saye_bringup')
    # pkg_project_localization = get_package_share_directory('saye_localization')
    pkg_project_description = get_package_share_directory('saye_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    resources_path = PathJoinSubstitution([pkg_project_description, 'worlds', 'models'])
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            resources_path,
            TextSubstitution(text=':'),
            EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')
        ]
    )


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
        ])
    )

    # Append -r to run simulation immediately
    gz_args_running = [LaunchConfiguration('gz_args'), ' -r']

    # Whether to launch the Gazebo GUI; disable for headless stability in containers
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Launch Gazebo GUI (false for headless)'
    )

    # Robot initial pose (map frame). Override at launch time as needed.
    # Default spawn position is on a road, calculated from map2gazebo road geometry.
    # Coordinates are verified to be on a road (distance to road = 0.0m).
    # To override spawn position at launch: robot_x:=<x> robot_y:=<y> robot_Y:=<yaw>
    robot_x_arg = DeclareLaunchArgument('robot_x', default_value='5.55', description='Robot X in meters (east coordinate)')
    robot_y_arg = DeclareLaunchArgument('robot_y', default_value='-94.69', description='Robot Y in meters (north coordinate)')
    robot_z_arg = DeclareLaunchArgument('robot_z', default_value='0.35', description='Robot Z in meters (height above ground)')
    robot_R_arg = DeclareLaunchArgument('robot_R', default_value='0.0', description='Robot roll in radians')
    robot_P_arg = DeclareLaunchArgument('robot_P', default_value='0.0', description='Robot pitch in radians')
    robot_Y_arg = DeclareLaunchArgument('robot_Y', default_value='-1.5064', description='Robot yaw in radians (orientation, aligned with road, rotated 180°)')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': gz_args_running,
            'gui': LaunchConfiguration('gui')
        }.items(),
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'rviz', 'saye.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz')),
       parameters=[{'use_sim_time': True}]
    )

    bridge = Node(
    # Bridge ROS topics and Gazebo messages for establishing communication
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': True
        }],
        output='screen'
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description_content},
            {'frame_prefix': 'saye/'},
            {'use_sim_time': True}
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

    # Publish odom -> saye transform from odometry messages
    # This is needed for RViz to visualize the robot when fixed frame is set to "odom"
    # robot_state_publisher handles saye -> saye/base_link, so we need odom -> saye
    installed_script = os.path.join(pkg_project_bringup, '..', '..', 'lib', 'saye_bringup', 'odom_to_tf.py')
    source_script = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(pkg_project_bringup))),
        'src', 'ackermann-vehicle-gzsim-ros2', 'saye_bringup', 'scripts', 'odom_to_tf.py'
    )
    odom_script = installed_script if os.path.exists(installed_script) else source_script
    odom_to_tf_node = ExecuteProcess(
        cmd=['python3', odom_script,
             '--ros-args',
             '-p', 'base_frame:=saye/base_link',  # Publish odom -> saye/base_link (matches robot_state_publisher prefix)
             '-p', 'odom_frame:=odom',
             '-p', 'use_sim_time:=true'],
        name='odom_to_tf',
        output='screen'
    )

    # Follow camera node - updates Gazebo camera to follow the robot
    installed_follow_script = os.path.join(pkg_project_bringup, '..', '..', 'lib', 'saye_bringup', 'follow_camera.py')
    source_follow_script = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(pkg_project_bringup))),
        'src', 'ackermann-vehicle-gzsim-ros2', 'saye_bringup', 'scripts', 'follow_camera.py'
    )
    follow_script = installed_follow_script if os.path.exists(installed_follow_script) else source_follow_script
    
    # Initial camera setup (static position at spawn, then follow camera takes over)
    initial_camera_setup = TimerAction(
        period=4.0,  # Wait 4 seconds for Gazebo and robot to spawn
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/gui/move_to/pose',
                    '--reqtype', 'gz.msgs.GUICamera',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '2000',
                    '--req', 'pose: {position: {x: 5.23, y: -89.70, z: 3.35}, orientation: {x: 0.1359, y: 0.1449, z: -0.6703, w: 0.7150}}'
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('gui'))
            )
        ]
    )
    
    # Follow camera node - continuously updates camera to follow robot
    follow_camera_node = ExecuteProcess(
        cmd=['python3', follow_script,
             '--ros-args',
             '-p', 'use_sim_time:=true'],
        name='follow_camera',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    return LaunchDescription([
        world_arg,
        gz_args_arg,
        gui_arg,
        set_gz_resource_path,
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
        odom_to_tf_node,  # Publish odom -> saye transform for RViz
        rviz,
        initial_camera_setup,  # Initial camera position
        follow_camera_node  # Follow camera (updates continuously)
    ])
