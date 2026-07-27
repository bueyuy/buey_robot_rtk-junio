#!/usr/bin/env python3
"""Drive manual: joystick MQTT + gateway a motores, sin odometria.

  ros2 launch buey_robot motor_gateway.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('buey_robot')
    joystick_yaml = os.path.join(pkg_share, 'config', 'navigation', 'joystick.yaml')
    gateway_yaml = os.path.join(pkg_share, 'config', 'motor', 'gateway.yaml')

    joystick_node = Node(
        package='buey_robot',
        executable='joystick_controller',
        name='joystick_controller',
        output='screen',
        parameters=[joystick_yaml],
    )

    motor_gateway_node = Node(
        package='buey_robot',
        executable='motor_gateway',
        name='motor_gateway',
        output='screen',
        parameters=[gateway_yaml],
    )

    return LaunchDescription([
        joystick_node,
        motor_gateway_node,
    ])
