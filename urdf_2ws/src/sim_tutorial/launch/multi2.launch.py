import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "sim_tutorial"

    # 1️⃣ 로봇 3 & 4 로드
    robot3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "robot_3.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    robot4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "robot_4.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # 2️⃣ Gazebo 환경 설정 (custom_world 사용)
    world_file = os.path.join(
        get_package_share_directory(package_name),
        "worlds",
        "custom_world.world"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
        launch_arguments={"world": world_file, "use_sim_time": "true"}.items(),
    )

    # 3️⃣ Gazebo 내 로봇 배치 (spawn)
    spawn_robot3 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "robot3", "-x", "0", "-y", "0", "-z", "0"],
        output="screen",
    )

    spawn_robot4 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "robot4", "-x", "2", "-y", "0", "-z", "0"],
        output="screen",
    )

    # 4️⃣ LIDAR 런치파일 포함 (기존 lidar_node 제거)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "lidar.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    return LaunchDescription([
        gazebo,           # Gazebo 실행
        robot3_launch,    # robot_3 실행
        robot4_launch,    # robot_4 실행
        spawn_robot3,     # robot_3 Gazebo 배치
        #spawn_robot4,     # robot_4 Gazebo 배치
        #lidar_launch      # lidar.launch.py 실행
    ])
