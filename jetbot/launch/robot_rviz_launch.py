# launch/robot_launch.py
# load the urdf file in rviz with fake data of joint_state_publisher
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    # Path to the URDF file
    urdf_file_path = os.path.join(get_package_share_path('jetbot'), 'urdf', 'robot.urdf.xacro')
    
    # Path to rviz config to see the tfs and robotmodel
    rviz_config_path = os.path.join(get_package_share_path('jetbot'),'config', 'rviz_config.rviz')

    robot_description = ParameterValue(Command(['xacro ', urdf_file_path]), value_type=str)

    robot_state_publisher_node =  Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description , 'use_sim_time': True}]
        )
    
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])
