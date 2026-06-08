#!/usr/bin/env python3
"""Launch para motor_gateway con joystick. Drive manual sin odometria.

Nodos:
  - navigation/joystick.py  (MQTT joystick -> /cmd_vel_joy)
  - motor/gateway.py        (cmd_vel_joy -> motores)

Uso:
  ros2 launch buey_robot motor_gateway.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('buey_robot')
    nav_yaml = os.path.join(pkg_share, 'config', 'navigation.yaml')
    motor_yaml = os.path.join(pkg_share, 'config', 'motor.yaml')

    joystick_node = Node(
        package='buey_robot',
        executable='joystick_controller',
        name='joystick_controller',
        output='screen',
        parameters=[nav_yaml],
    )

    motor_gateway_node = Node(
        package='buey_robot',
        executable='motor_gateway',
        name='motor_gateway',
        output='screen',
        parameters=[motor_yaml],
    )

    return LaunchDescription([
        joystick_node,
        motor_gateway_node,
    ])
