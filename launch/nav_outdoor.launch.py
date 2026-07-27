#!/usr/bin/env python3
"""Navegacion outdoor (GPS RTK): sensores, fusion, odometria, MQTT y navegacion. La ruta
de waypoints llega en vivo por MQTT (lat/lon); el datum lo fija odometry_gps con el primer
fix confiable. El motor_gateway (joystick + motores) se levanta aparte, en su terminal:

  Terminal 1:  ros2 launch buey_robot motor_gateway.launch.py
  Terminal 2:  ros2 launch buey_robot nav_outdoor.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('buey_robot')
    launch_dir = os.path.join(pkg_share, 'launch')
    robot_yaml = os.path.join(pkg_share, 'config', 'robot.yaml')
    fusion_yaml = os.path.join(pkg_share, 'config', 'fusion', 'heading.yaml')
    gps_odom_yaml = os.path.join(pkg_share, 'config', 'odometry', 'gps.yaml')
    controller_yaml = os.path.join(pkg_share, 'config', 'navigation', 'controller.yaml')
    initializer_yaml = os.path.join(pkg_share, 'config', 'navigation', 'initializer.yaml')

    fusion = Node(
        package='buey_robot', executable='fusion_heading', name='fusion_heading',
        output='screen', parameters=[fusion_yaml],
    )
    odometry = Node(  # robot.yaml aporta el lever-arm de la antena
        package='buey_robot', executable='odometry_gps', name='odometry_gps',
        output='screen', parameters=[gps_odom_yaml, robot_yaml],
    )
    controller = Node(
        package='buey_robot', executable='navigation_controller', name='navigation_controller',
        output='screen', parameters=[controller_yaml],
    )
    initializer = Node(
        package='buey_robot', executable='navigation_initializer', name='navigation_initializer',
        output='screen', parameters=[initializer_yaml],
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sensors.launch.py')))
    mqtt = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mqtt.launch.py')))

    return LaunchDescription([
        sensors,
        mqtt,
        fusion,
        odometry,
        controller,
        initializer,
    ])
