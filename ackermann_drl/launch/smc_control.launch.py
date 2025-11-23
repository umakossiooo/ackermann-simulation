"""Launch file for Sliding Mode Control (SMC) path following.

Replaces DRL training with SMC controller for road following.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for SMC control."""
    
    # Launch arguments
    lambda_param_arg = DeclareLaunchArgument(
        'lambda_param',
        default_value='1.0',
        description='Weight for heading error in sliding surface'
    )
    
    K_smc_arg = DeclareLaunchArgument(
        'K_smc',
        default_value='2.0',
        description='SMC control gain'
    )
    
    boundary_layer_arg = DeclareLaunchArgument(
        'boundary_layer',
        default_value='0.1',
        description='Boundary layer thickness to reduce chattering'
    )
    
    desired_velocity_arg = DeclareLaunchArgument(
        'desired_velocity',
        default_value='2.0',
        description='Desired forward velocity (m/s)'
    )
    
    max_steering_arg = DeclareLaunchArgument(
        'max_steering',
        default_value='1.0',
        description='Maximum steering command (rad/s)'
    )
    
    # SMC Control Node
    smc_node = Node(
        package='ackermann_drl',
        executable='smc_control_node.py',
        name='smc_control_node',
        output='screen',
        parameters=[{
            'lambda_param': LaunchConfiguration('lambda_param'),
            'K_smc': LaunchConfiguration('K_smc'),
            'boundary_layer': LaunchConfiguration('boundary_layer'),
            'desired_velocity': LaunchConfiguration('desired_velocity'),
            'max_steering': LaunchConfiguration('max_steering'),
        }]
    )
    
    return LaunchDescription([
        lambda_param_arg,
        K_smc_arg,
        boundary_layer_arg,
        desired_velocity_arg,
        max_steering_arg,
        smc_node,
    ])

