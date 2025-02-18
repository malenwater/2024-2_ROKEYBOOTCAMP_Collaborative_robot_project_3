import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

def generate_launch_description():
    pkg_share = get_package_share_directory("car_simulation")

    xacro_file = os.path.join(pkg_share, "urdf", "car.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        )
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_desc}]
    ) 

    
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot"],
        output="screen"
    )

    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        delayed_spawn
    ])
