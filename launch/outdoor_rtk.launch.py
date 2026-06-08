#!/usr/bin/env python3
"""Launch outdoor con GPS RTK + IMU: sensores, odometria y telemetria.

NO incluye el trajectory_controller (ver trajectory_controller.launch.py).
Asi se puede reportar RTK / IMU / telemetria MQTT y fijar BASE/START en el
dashboard sin que el robot intente navegar.

Nodos:
  - mapper/gps_nmea.py        (serial NMEA -> /gps/fix + rtk/location/json)
  - odometry/rtk.py           (GPS+IMU -> /odom_filtered)
  - adapters/mqtt/outputs/pose.py  (telemetry MQTT, incluye heading_gps)

La IMU la publica microROS directamente en /imu/data (sensor_msgs/Imu); su agente
corre en terminal aparte y no se lanza desde aca (igual que motor_gateway).

motor_gateway NO se incluye — corre en terminal aparte con motor_gateway.launch.py.
Para navegacion autonoma, lanzar ademas trajectory_controller.launch.py.

Uso:
  ros2 launch buey_robot outdoor_rtk.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('buey_robot')

    nav_yaml = os.path.join(pkg_share, 'config', 'navigation.yaml')
    nav_outdoor_yaml = os.path.join(pkg_share, 'config', 'navigation_outdoor.yaml')
    sensors_yaml = os.path.join(pkg_share, 'config', 'sensors.yaml')
    imu_yaml = os.path.join(pkg_share, 'config', 'imu.yaml')

    # Driver GPS NMEA (serial)
    gps_node = Node(
        package='buey_robot',
        executable='gps_nmea_driver',
        name='gps_nmea_driver',
        output='screen',
        parameters=[sensors_yaml],
    )

    # Brujula calibrada: /imu/mag (crudo) -> /imu/heading_calibrated
    imu_compass_node = Node(
        package='buey_robot',
        executable='imu_compass',
        name='imu_compass',
        output='screen',
        parameters=[imu_yaml],
    )

    # Bridge MQTT de IMU/heading/odom para el panel de telemetria
    imu_bridge_node = Node(
        package='buey_robot',
        executable='imu_bridge',
        name='imu_bridge',
        output='screen',
    )

    # Odometria RTK: recibe GPS params via ROS2 desde navigation.yaml + sensors.yaml
    rtk_odom_node = Node(
        package='buey_robot',
        executable='rtk_odometry',
        name='rtk_odometry',
        output='screen',
        parameters=[
            sensors_yaml,
            nav_yaml,
            nav_outdoor_yaml,
        ],
    )

    # Telemetria MQTT consolidada
    pose_node = Node(
        package='buey_robot',
        executable='pose_publisher',
        name='pose_publisher',
        output='screen',
    )

    # Sensores arrancan primero, odometria y telemetria con delay de 2s
    delayed_nodes = TimerAction(
        period=2.0,
        actions=[
            rtk_odom_node,
            pose_node,
            imu_bridge_node,
        ]
    )

    return LaunchDescription([
        gps_node,
        imu_compass_node,
        delayed_nodes,
    ])
