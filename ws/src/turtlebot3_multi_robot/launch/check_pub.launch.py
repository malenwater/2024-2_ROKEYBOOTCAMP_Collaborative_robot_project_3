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
    pkg_path = os.path.join(get_package_share_directory("turtlebot3_multi_robot"))

    TURTLEBOT3_MODEL = "waffle"
    urdf = os.path.join(
        pkg_path, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )
    with open(urdf, 'r') as file:
        robot_description = file.read()
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
        # Create state publisher node for that instance
    turtlebot_state_publisher = Node(
        package='robot_state_publisher',
        # namespace="test_1",
        executable='robot_state_publisher',
        output='screen',
        parameters=[{   "robot_description": robot_description,
                        'use_sim_time': use_sim_time,
                        'publish_frequency': 10.0}],
        # remappings=remappings,
        # arguments=[urdf],
    )


    # LaunchDescription 반환
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="use sim time"
            ),
            turtlebot_state_publisher,
        ]
    )
