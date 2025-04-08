import rclpy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare the frequency parameter
        DeclareLaunchArgument(
            'frequency',
            default_value='0.5',
            description='Frequency for service calls in Hz'
        ),

        # Node for AssemblerClient with frequency parameter and topic remapping
        Node(
            package='pointcloud2_assembler',  # Replace with your package name
            executable='assembler_client',  # The name of the executable in your package
            name='assembler_client',
            output='screen',
            parameters=[{
                'frequency': LaunchConfiguration('frequency')
            }],
            remappings=[
                ('/assembled_cloud', '/assembled_cloud')]
        )
    ])
