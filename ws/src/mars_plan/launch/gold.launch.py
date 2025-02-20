import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 경로 가져오기
    pkg_share = get_package_share_directory('mars_plan')
    sdf_file = os.path.join(pkg_share, 'urdf', 'gold.sdf')  # SDF 파일 경로

    # robot 딕셔너리 정의 (예시로 좌표 설정)
    robot = {
        'name': 'gold',
        'x_pose': '2.0',
        'y_pose': '2.0',
        'z_pose': '0.0'
    }
    
    # Gazebo에서 SDF 엔티티 소환
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', sdf_file,
            '-entity', robot['name'],
            '-robot_namespace', robot['name'],  # namespace로 설정
            '-x', robot['x_pose'], '-y', robot['y_pose'], '-z', robot['z_pose'], '-Y', '0.0',
            '-unpause'
        ],
        output='screen',
    )

    # 5초 후에 SDF 모델 소환
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )    

    return LaunchDescription([
        delayed_spawn
    ])
