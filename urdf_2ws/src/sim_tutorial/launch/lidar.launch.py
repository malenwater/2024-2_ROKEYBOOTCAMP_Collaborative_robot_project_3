import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 패키지 정보, 파일 경로 설정
    package_name = "sim_tutorial"

    # 2. 로봇 관련 런치파일 실행
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # 로봇 urdf모델 및 센서 설정
            [os.path.join(get_package_share_directory(package_name), "launch", "robot_4.launch.py")]
        ),
        # 시뮬레이션 시간 사용
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # 3.사용할 gazebo 월드 설정
    world = os.path.join(
        get_package_share_directory(package_name),
        'worlds', # worlds.
        'custom_world.world' # gazebo 환경 사용
    )


    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
        launch_arguments={'world': world}.items()
    )


    # gazebo.launch.py 실행해서, gazebo시뮬레이션을 시작한다.
    # spawn_entity.py을 실행하여 gazebo에 로봇을 배치한다.
    # gazebo.launch.py/ spawn.entity.py 은 ros2패키지 기본 실행 파일이다.
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot", "-x", "1", "-y", "1", "-z", "1"],
        output="screen",
    )

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
        ]
    )
