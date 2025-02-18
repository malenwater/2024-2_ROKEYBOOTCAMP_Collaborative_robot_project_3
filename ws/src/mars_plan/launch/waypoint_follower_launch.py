import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


'''
사용 목적: 이미 저장된 웨이포인트를 따라가는 기능

'''
def generate_launch_description():
    return LaunchDescription([
        # Waypoint Follower 노드 실행
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            # 임의로 추가해놓음,
            parameters=[{
                'use_sim_time': True,
                'waypoints': [
                    {'x': 2.0, 'y': 2.0, 'yaw': 0.0},
                    {'x': 4.0, 'y': 4.0, 'yaw': 0.0},
                ]
            }]
        ),
    ])
