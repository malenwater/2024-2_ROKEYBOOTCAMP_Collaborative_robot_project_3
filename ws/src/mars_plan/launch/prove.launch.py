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

def generate_launch_description():
    ld = LaunchDescription()
    robots = [
        {'name': 'tb1', 'x_pose': '-1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
        {'name': 'tb2', 'x_pose': '-1.5', 'y_pose': '0.5', 'z_pose': 0.01},
        #{'name': 'tb3', 'x_pose': '1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
        #{'name': 'tb4', 'x_pose': '1.5', 'y_pose': '0.5', 'z_pose': 0.01},
        # ...
        # ...
        ]
    params_file = LaunchConfiguration('nav_params_file')
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    turtlebot3_multi_robot = get_package_share_directory('mars_plan')
    package_dir = get_package_share_directory('mars_plan')
    TURTLEBOT3_MODEL = 'waffle'
    nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')
    urdf = os.path.join(
        turtlebot3_multi_robot, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    )
    last_action = None
    # Spawn turtlebot3 instances in gazebo
    for robot in robots:

        namespace = [ '/' + robot['name'] ]

        # Create state publisher node for that instance
        turtlebot_state_publisher = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                            'publish_frequency': 10.0}],
            remappings=remappings,
            arguments=[urdf],
        )
        ld.add_action(turtlebot_state_publisher)

    #     # Create spawn call
        spawn_turtlebot3_burger = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', os.path.join(turtlebot3_multi_robot,'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-x', robot['x_pose'], '-y', robot['y_pose'],
                '-z', '0.01', '-Y', '0.0',
                '-unpause',
            ],
            output='screen',
        )
        ld.add_action(spawn_turtlebot3_burger)

    #     bringup_cmd = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(nav_launch_dir, 'bringup_launch.py')),
    #                 launch_arguments={  
    #                                 'slam': 'False',
    #                                 'namespace': namespace,
    #                                 'use_namespace': 'True',
    #                                 'map': '',
    #                                 'map_server': 'False',
    #                                 'params_file': params_file,
    #                                 'default_bt_xml_filename': os.path.join(
    #                                     get_package_share_directory('nav2_bt_navigator'),
    #                                     'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
    #                                 'autostart': 'true',
    #                                 'use_sim_time': use_sim_time, 'log_level': 'warn'}.items()
    #                                 )

    #     if last_action is None:
    #         # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
    #         ld.add_action(turtlebot_state_publisher)
    #         ld.add_action(spawn_turtlebot3_burger)
    #         ld.add_action(bringup_cmd)

    #     else:
    #         # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
    #         # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
    #         spawn_turtlebot3_event = RegisterEventHandler(
    #             event_handler=OnProcessExit(
    #                 target_action=last_action,
    #                 on_exit=[spawn_turtlebot3_burger,
    #                         turtlebot_state_publisher,
    #                         bringup_cmd],
    #             )
    #         )

    #         ld.add_action(spawn_turtlebot3_event)

    #     # Save last instance for next RegisterEventHandler
    #     last_action = spawn_turtlebot3_burger
        
    return ld