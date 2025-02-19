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


    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )

    # Launch them all!
    return LaunchDescription(
        [
            gzserver_cmd,
            gzclient_cmd,
            prove_spawn,
        ]
    )
