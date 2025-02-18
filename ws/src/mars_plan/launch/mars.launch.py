import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "mars_plan"

    prove_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "prove.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )


    world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'custom_world.world'
    )


    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
        launch_arguments={'world': world}.items()
    )

    # Launch them all!
    return LaunchDescription(
        [
            gazebo,
            prove_spawn,
        ]
    )
