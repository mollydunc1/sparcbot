"""Start the circle tracker (full state feedback round a 1 m circle).

    ros2 launch sparcbot_bringup robot.launch.py            # terminal 1
    ros2 launch sparcbot_bringup circle_tracker.launch.py   # terminal 2
    ros2 service call /circle_tracker/start std_srvs/srv/Trigger   # terminal 3

The tracker waits for ~/start, so the robot does not move until you ask it to.
All its settings (radius, speed, laps, gains, limits) are at the top of
sparcbot_control/circle_tracker.py; it does not read sparcbot.yaml.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sparcbot_control',
            executable='circle_tracker',
            name='circle_tracker',
            output='screen',
            emulate_tty=True),
    ])
