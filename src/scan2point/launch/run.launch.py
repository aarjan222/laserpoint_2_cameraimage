import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    distance_at_diff_angles = Node(package='scan2point',
                                   executable='distance_at_diff_angles.py', output='screen')

    laser2pc = Node(package='scan2point',
                    executable='laser2point.py', output='screen')

    return LaunchDescription([
        # distance_at_diff_angles,
        laser2pc,
    ])
