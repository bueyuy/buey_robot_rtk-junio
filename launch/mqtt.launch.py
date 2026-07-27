#!/usr/bin/env python3
"""Puentes MQTT: estado ROS -> broker, comandos broker -> ROS y /rosout -> broker."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    telemetry = Node(
        package='buey_robot', executable='telemetry_bridge', name='telemetry_bridge',
        output='screen',
    )
    command = Node(
        package='buey_robot', executable='command_bridge', name='command_bridge',
        output='screen',
    )
    logs = Node(
        package='buey_robot', executable='log_bridge', name='log_bridge',
        output='screen',
    )

    return LaunchDescription([telemetry, command, logs])
