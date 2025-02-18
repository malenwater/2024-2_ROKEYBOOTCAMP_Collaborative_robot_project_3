import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

'''
Gazebo 시뮬레이션을 실행하고, 
특정 환경(custom_world.world)을 로드한 후, 
TurtleBot3 로봇을 특정 위치에 스폰하는 ROS 2 런치 파일
'''

def generate_launch_description():
    package_name = "mars_plan"

    # rsp = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(get_package_share_directory(package_name), "launch", "prove.launch.py")]
    #     ),
    #     launch_arguments={"use_sim_time": "true"}.items(),
    # )


    # gazebo시뮬레이션 을 위해서 mars.world을 불러온다.
    world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'custom_world.world'
    )


    # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo_ros패키지의 gazebo.launch.py을 실행하여 gazebo을 실행한다.
    # gazebo.launch,py -> gazebo_ros 패키지에서 제공하는 Gazebo 시뮬레이터 실행을 위한 기본 런치 파일
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
        # launch_arguments={'world': world}.items() 옵션을 주어서 특정 월드 파일(예: custom_world.world)을 로드하는 방식
        launch_arguments={'world': world}.items() # <- custom_world.world 환경을 로드한다.
    )


    # 로봇 스폰
    # spawn_entity.py 를 실행하여 로봇을 스폰한다.
    '''

    **
    mars.launch.py는 Gazebo 시뮬레이터를 실행하고,
    특정 환경(custom_world.world)을 로드한 후,
    로봇을 Gazebo에 스폰하는 역할

    1.Gazebo 실행 (gazebo.launch.py 실행)
    2.사용할 환경(custom_world.world)을 로드
    3.TurtleBot3 로봇을 특정 좌표에 생성 (spawn_entity.py 사용)

    =>     spawn_entity.py 에서 urdf를 gazebo에 전달하는 과정,
    spawn_entity.py는 ROS 2의 gazebo_ros 패키지에 포함된 실행 파일로,
      로봇의 URDF(또는 SDF) 모델을 Gazebo에 전달하여 스폰(생성) 하는 역할
    
    1.robot_state_publisher가 robot_description에 URDF를 로드함.
    2.spawn_entity.py가 -topic robot_description을 통해 URDF 데이터를 가져옴.
    3.Gazebo에서 turtlebot3_waffle이라는 이름으로 로봇이 생성됨.


    '''
    # 
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "turtlebot3_waffle", "-x", "1", "-y", "1", "-z", "1"],
        output="screen",
    )

    # Launch them all!
    return LaunchDescription(
        [
            #rsp,
            gazebo,
            spawn_entity,
        ]
    )
