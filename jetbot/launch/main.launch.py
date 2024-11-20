from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, LogInfo, GroupAction
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'jetbot'
    config_folder = os.path.join(get_package_share_directory(package_name), 'config')
    mapper_params_path = os.path.join(config_folder, 'mapper_params_online_async.yaml')

      # Encoder Publisher Node with error handling
    # encoder_node = Node(
    #     package='jetbot',
    #     executable='encoder_publisher.py',
    #     name='encoder_publisher',
    #     output='screen',
    #     on_exit=[
    #         LogInfo(msg="Encoder node exited. Checking status...")
    #     ]
    # )
 
    # Odometry Node conditional on Encoder Node success
    # odometry_node = Node(
    #     package='jetbot',
    #     executable='odometry_publisher.py',
    #     name='odometry_publisher',
    #     output='screen',
    #     condition=IfCondition('${encoder_node_success}')
    # )
 
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'base_link',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )
 
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[mapper_params_path],
        output='screen'
    )
 
    check_rplidar_connection = ExecuteProcess(
        cmd=['ros2', 'topic', 'list'],
        output='screen',
        on_exit=[
            LogInfo(msg='Checking RPLidar connection...'),
            TimerAction(
                period=5.0,
                actions=[
                    LogInfo(msg='RPLidar connection successful. Launching SLAM Toolbox.'),
                    slam_toolbox_node
                ]
            )
        ]
    )
 
    nodes = [
        # encoder_node,
        # odometry_node,
        rplidar_node,
        check_rplidar_connection,
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen'
        # )
    ]


    return LaunchDescription(nodes)