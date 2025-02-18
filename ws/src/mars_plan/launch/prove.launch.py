import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

'''
prove.launch.py는 로봇의 URDF 모델을 Gazebo에서 사용할 수 있도록 설정하는 역할

1. Xacro 파일(turtlebot3_waffle.xacro)을 URDF로 변환
2. 변환된 URDF 데이터를 /robot_description에 저장
3. robot_state_publisher를 실행하여 로봇 상태를 방송
4. ROS 2 시스템이 Gazebo에서 로봇을 제대로 불러올 수 있도록 함
'''

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    # 패키지 경로
    #pkg_path = get_package_share_directory("turtlebot3_description")
    pkg_path = get_package_share_directory("mars_plan")
    # TurtleBot3 Waffle URDF 경로
    xacro_file = os.path.join(pkg_path, "urdf", "turtlebot3_waffle.xacro") # <- 이거 파일 코드 어디?
    # Xacro 파일 처리
    robot_description = xacro.process_file(xacro_file)
    # 파라미터 설정
    params = {
        #robot_description을 ROS 2의 파라미터 서버에 저장하여 다른 노드에서 사용할 수 있도록 설정.
        "robot_description": robot_description.toxml(), 
        "use_sim_time": use_sim_time,
        "namespace": "robot4"
    }

    # LaunchDescription 반환
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="use sim time"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],
            ),
        ]
    )
