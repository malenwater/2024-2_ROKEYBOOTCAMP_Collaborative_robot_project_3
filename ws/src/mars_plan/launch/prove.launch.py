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
        {'name': 'tb1', 'x_pose': '2.0', 'y_pose': '5.0', 'z_pose': '0.01'},
        {'name': 'tb2', 'x_pose': '2.5', 'y_pose': '5.0', 'z_pose': '0.01'},
        #{'name': 'tb3', 'x_pose': '1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
        #{'name': 'tb4', 'x_pose': '1.5', 'y_pose': '0.5', 'z_pose': 0.01},
        # ...
        # ...
        ]
    package_dir = get_package_share_directory('mars_plan')
    
    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive', default_value=enable_drive, description='Enable robot drive node'
    )
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )
    
    # print("hihi")
    params_file = LaunchConfiguration('nav_params_file')
    # declare_params_file_cmd = DeclareLaunchArgument(
    #     'nav_params_file',
    #     default_value=os.path.join(package_dir, 'params', 'nav2_params.yaml'),
    #     description='Full path to the ROS2 parameters file to use for all launched nodes')
    # print("hihi")
    
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    turtlebot3_multi_robot = get_package_share_directory('mars_plan')
    TURTLEBOT3_MODEL = 'waffle'
    nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')
    urdf = os.path.join(
        turtlebot3_multi_robot, 'urdf', 'turtlebot3_' + TURTLEBOT3_MODEL + '_pi.urdf'
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

        # Create spawn call
        spawn_turtlebot3_burger = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', os.path.join(turtlebot3_multi_robot,'models', 'turtlebot3_' + TURTLEBOT3_MODEL +'_pi', 'model.sdf'),
                '-entity', robot['name'],
                '-robot_namespace', namespace,
                '-x', robot['x_pose'], '-y', robot['y_pose'],
                '-z', robot['z_pose'], '-Y', '-1.57',
                '-unpause',
            ],
            output='screen',
        )
        ld.add_action(spawn_turtlebot3_burger)
        bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_launch_dir, 'bringup_launch.py')),
                    launch_arguments={  
                                    'slam': 'False',
                                    'namespace': namespace,
                                    'use_namespace': 'True',
                                    'map': '',
                                    'map_server': 'False',
                                    'params_file': params_file,
                                    'default_bt_xml_filename': os.path.join(
                                        get_package_share_directory('nav2_bt_navigator'),
                                        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                                    'autostart': 'true',
                                    'use_sim_time': use_sim_time, 'log_level': 'warn'}.items()
                                    )
        ld.add_action(bringup_cmd)
        
    # for robot in robots:

    #     namespace = [ '/' + robot['name'] ]

    #     # Create a initial pose topic publish call
    #     message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
    #         robot['x_pose'] + ', y: ' + robot['y_pose'] + \
    #         ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

    #     initial_pose_cmd = ExecuteProcess(
    #         cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + ['/initialpose'],
    #             'geometry_msgs/PoseWithCovarianceStamped', message],
    #         output='screen'
    #     )

    #     rviz_cmd = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(nav_launch_dir, 'rviz_launch.py')),
    #             launch_arguments={'use_sim_time': use_sim_time, 
    #                               'namespace': namespace,
    #                               'use_namespace': 'True',
    #                               'rviz_config': rviz_config_file, 'log_level': 'warn'}.items(),
    #                                condition=IfCondition(enable_rviz)
    #                                 )

    #     drive_turtlebot3_burger = Node(
    #         package='turtlebot3_gazebo', executable='turtlebot3_drive',
    #         namespace=namespace, output='screen',
    #         condition=IfCondition(enable_drive),
    #     )

    #     # Use RegisterEventHandler to ensure next robot rviz launch happens 
    #     # only after all robots are spawned
    #     post_spawn_event = RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=last_action,
    #             on_exit=[initial_pose_cmd, rviz_cmd, drive_turtlebot3_burger],
    #         )
    #     )

    #     # Perform next rviz and other node instantiation after the previous intialpose request done
    #     last_action = initial_pose_cmd

    #     ld.add_action(post_spawn_event)
    #     # ld.add_action(declare_params_file_cmd)
        
    return ld