#!/usr/bin/env python3
"""Launch UNICO de navegacion outdoor (GPS RTK). Levanta TODO el stack:

  - motor_gateway (joystick_controller + motor_gateway)   [drive manual + salida a motores]
  - micro_ros_agent                                        [agente serial de la IMU]
  - mapper/gps_nmea.py        (serial NMEA -> /gps/fix + rtk/location/json)   [SENSORES]
  - mapper/mpu6050_gyro.py    (/mpu6050/imu/data -> /heading/fused)
  - odometry/rtk.py           (GPS + heading -> /odom_filtered)
  - adapters/mqtt/outputs/pose.py + imu_bridge             [TELEMETRIA MQTT]
  - navigation/controller.py                               [NAV_CONTROLLER]

Navegacion FULL RTK con geolocalizacion DINAMICA: la ruta de waypoints llega en vivo
por MQTT (bueyuy/waypoints, lat/lon) desde la telemetria. NO hay waypoints fijos ni
carga por archivo. El origen del frame lo fija rtk con el primer fix (auto).

Orden: motores + sensores primero; odometria/telemetria a +2s; el controller a +6s
(igual espera internamente odom + origen + calibracion del gyro + waypoints).

Uso:
  ros2 launch buey_robot nav_outdoor.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    TimerAction, ExecuteProcess, IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('buey_robot')

    nav_yaml = os.path.join(pkg_share, 'config', 'navigation.yaml')
    sensors_yaml = os.path.join(pkg_share, 'config', 'sensors.yaml')
    robot_yaml = os.path.join(pkg_share, 'config', 'robot.yaml')
    imu_yaml = os.path.join(pkg_share, 'config', 'imu.yaml')
    motor_yaml = os.path.join(pkg_share, 'config', 'motor.yaml')

    # --- motor_gateway (joystick + salida a motores) ---
    motor_gateway = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'motor_gateway.launch.py')))

    # --- Agente microROS de la IMU (serial) ---
    micro_ros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
             'serial', '--dev', '/dev/ttyUSB0', '-b', '115200', '-v6'],
        output='log',
    )

    # --- SENSORES ---
    gps_node = Node(  # robot.yaml aporta el lever-arm de la antena
        package='buey_robot', executable='gps_nmea_driver', name='gps_nmea_driver',
        output='screen', parameters=[sensors_yaml, robot_yaml],
    )
    mpu6050_gyro_node = Node(
        package='buey_robot', executable='mpu6050_gyro', name='mpu6050_gyro',
        output='screen', parameters=[imu_yaml],
    )

    # --- ODOMETRIA + TELEMETRIA (delay para que los sensores arranquen) ---
    rtk_odom_node = Node(
        package='buey_robot', executable='rtk_odometry', name='rtk_odometry',
        output='screen', parameters=[sensors_yaml, nav_yaml],
    )
    pose_node = Node(
        package='buey_robot', executable='pose_publisher', name='pose_publisher',
        output='screen',
    )
    imu_bridge_node = Node(
        package='buey_robot', executable='imu_bridge', name='imu_bridge',
        output='screen',
    )
    odom_telemetry = TimerAction(
        period=2.0, actions=[rtk_odom_node, pose_node, imu_bridge_node])

    # --- NAV_CONTROLLER (delay para que /odom_filtered ya publique) ---
    controller_node = Node(
        package='buey_robot', executable='trajectory_controller',
        name='trajectory_controller', output='screen',
        parameters=[nav_yaml, motor_yaml],
    )
    controller = TimerAction(period=6.0, actions=[controller_node])

    return LaunchDescription([
        motor_gateway,
        micro_ros_agent,
        gps_node,
        mpu6050_gyro_node,
        odom_telemetry,
        controller,
    ])
