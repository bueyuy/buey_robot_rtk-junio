#!/usr/bin/env python3
"""Launch de telemetria: solo pose_publisher.

Util para debug de telemetria sin lanzar navegacion ni motores.
Requiere que /odom_filtered y /heading/* esten publicando (ej: ZED activo).

Nodos:
  - adapters/mqtt/outputs/pose.py             (bueyuy/telemetry/json)

Uso:
  ros2 launch buey_robot telemetry.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pose_node = Node(
        package='buey_robot',
        executable='pose_publisher',
        name='pose_publisher',
        output='screen',
    )

    return LaunchDescription([
        pose_node
    ])
