import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    # 패키지 경로
    pkg_path = get_package_share_directory("turtlebot3_description")
    # TurtleBot3 Waffle URDF 경로
    xacro_file = os.path.join(pkg_path, "urdf", "turtlebot3_waffle.xacro")
    # Xacro 파일 처리
    robot_description = xacro.process_file(xacro_file)
    # 파라미터 설정
    params = {
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
