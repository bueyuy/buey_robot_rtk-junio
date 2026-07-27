#!/usr/bin/env python3
"""Sensores: agente micro-ROS de la IMU + drivers GPS e IMU."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('buey_robot')
    gps_yaml = os.path.join(pkg_share, 'config', 'drivers', 'gps_nmea.yaml')
    imu_yaml = os.path.join(pkg_share, 'config', 'drivers', 'imu_mpu6050.yaml')

    # La IMU corre sobre firmware micro-ROS: el agente cruza el serial a DDS (/mpu6050/imu/data).
    imu_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
             'serial', '--dev', '/dev/ttyUSB0', '-b', '115200', '-v6'],
        output='log',
    )

    # El GPS es serial NMEA directo: el driver abre el puerto sin agente.
    gps = Node(
        package='buey_robot', executable='gps_nmea', name='gps_nmea',
        output='screen', parameters=[gps_yaml],
    )
    imu = Node(
        package='buey_robot', executable='imu_mpu6050', name='imu_mpu6050',
        output='screen', parameters=[imu_yaml],
    )

    return LaunchDescription([imu_agent, gps, imu])
