"""Start the SPARCBot stack: driver + diff_drive + odometry + wheel_controller.

    ros2 launch sparcbot_bringup robot.launch.py

Then drive it from a second terminal:

    ros2 run sparcbot_teleop keyboard_teleop

All parameters come from one file: config/sparcbot.yaml.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = str(
        Path(get_package_share_directory('sparcbot_bringup')) / 'config' / 'sparcbot.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='SPARCBot parameter file'),

        Node(
            package='sparcbot_nodes',
            executable='sparcbot_driver_node',
            name='sparcbot_driver',
            parameters=[params_file],
            output='screen',
            emulate_tty=True),

        Node(
            package='sparcbot_nodes',
            executable='diff_drive_node',
            name='diff_drive',
            parameters=[params_file],
            output='screen',
            emulate_tty=True),

        Node(
            package='sparcbot_nodes',
            executable='odometry_node',
            name='odometry',
            parameters=[params_file],
            output='screen',
            emulate_tty=True),

        Node(
            package='sparcbot_control',
            executable='wheel_controller',
            name='wheel_controller',
            parameters=[params_file],
            output='screen',
            emulate_tty=True),
    ])
