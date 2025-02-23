import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import launch.logging
import xacro

'''
worldmap 가져오고
gazebo실행


'''


def generate_launch_description():
    ld = LaunchDescription()

    # ==================gazebo map server띄우기 ==========================
    # world가져옴,
    world = os.path.join(
    get_package_share_directory('car_track'),
    'worlds', 'parking_world.world')

    # 가제보 서버 띄움
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(), # <- 서버 띄울떄, world박아넣음, 
    )

    # 가제보 클라이언트 , 
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )

    ld.add_action(gzclient_cmd)
    ld.add_action(gzserver_cmd)

    # ==========================
    # 2. 로봇 띄우기
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_PUB = DeclareLaunchArgument(
                'use_sim_time',
                default_value='True',
                description='Use sim time if true')
    
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('car_track'))
    xacro_file = os.path.join(pkg_path,'urdf','robot_4.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot",
                    '-x', "1.0", '-y',  "1.0",
                     '-z', "10.0" , '-Y', '0.0',
                   ],
        output="screen",
    )

    ld.add_action(use_sim_time_PUB)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_entity)


    return ld

