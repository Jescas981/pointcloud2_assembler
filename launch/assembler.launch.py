from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud2_assembler',
            executable='assembler',
            name='assembler',
            output='screen',
            remappings=[
                ('/cloud_in', '/velodyne_new_points'),
                ('/cloud_out', '/filtered_points')
            ],
            parameters=[
                {
                    'fixed_frame': 'map',
                    'scan_buffer_size': 1000
                }
            ]
        )
    ])
