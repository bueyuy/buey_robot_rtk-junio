#!/usr/bin/env python3
"""Launch para navegacion indoor con ZED Visual Odometry.

Nodos:
  - buey_robot/robot_state_publisher_launch (ZED wrapper externo)
  - odometry/zed.py           (ZED odom -> /odom_filtered)
  - navigation/controller.py  (ARC/FINAL_APPROACH -> /cmd_vel)
  - adapters/mqtt/outputs/pose.py             (telemetry MQTT)

motor_gateway NO se incluye — corre en terminal aparte con motor_gateway.launch.py.
Workflow tipico: motor_gateway en T1 (siempre); este launch en T2 (arranca/se mata
por iteracion). motor_gateway escucha /cmd_vel de este controller, o /cmd_vel_joy
del joystick con prioridad.

Uso:
  ros2 launch buey_robot indoor_zed.launch.py
  ros2 launch buey_robot indoor_zed.launch.py waypoints_file:=/ruta/a/waypoints.yaml
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('buey_robot')
    buey_robot_share = get_package_share_directory('buey_robot')

    default_waypoints = os.path.join(pkg_share, 'waypoints', 'rectangulo_local.yaml')

    nav_yaml = os.path.join(pkg_share, 'config', 'navigation.yaml')
    nav_indoor_yaml = os.path.join(pkg_share, 'config', 'navigation_indoor.yaml')
    motor_yaml = os.path.join(pkg_share, 'config', 'motor.yaml')
    sensors_yaml = os.path.join(pkg_share, 'config', 'sensors.yaml')

    waypoints_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints,
        description='Ruta al archivo YAML de waypoints (coordenadas locales x,y)'
    )

    # ZED wrapper + robot_state_publisher (paquete externo buey_robot)
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(buey_robot_share, 'launch', 'robot_state_publisher_launch.py')
        ),
        launch_arguments={'use_zed_localization': 'true'}.items()
    )

    # Odometria ZED: filtra y re-publica en /odom_filtered
    zed_odom_node = Node(
        package='buey_robot',
        executable='zed_odometry',
        name='zed_odometry',
        output='screen',
        parameters=[
            sensors_yaml,
            nav_yaml,
            nav_indoor_yaml,
        ],
    )

    # Controlador de trayectoria
    controller_node = Node(
        package='buey_robot',
        executable='trajectory_controller',
        name='trajectory_controller',
        output='screen',
        parameters=[
            nav_yaml,
            nav_indoor_yaml,
            motor_yaml,
            {'auto_load_waypoints': LaunchConfiguration('waypoints_file')},
        ]
    )

    # Telemetria MQTT (pose.py)
    pose_node = Node(
        package='buey_robot',
        executable='pose_publisher',
        name='pose_publisher',
        output='screen',
    )

    # Nodos de control con delay de 6s para esperar inicializacion ZED
    # motor_gateway NO se levanta aca — corre en otra terminal con motor_gateway.launch.py
    delayed_nodes = TimerAction(
        period=6.0,
        actions=[
            zed_odom_node,
            controller_node,
            pose_node,
        ]
    )

    return LaunchDescription([
        waypoints_arg,
        zed_launch,
        delayed_nodes,
    ])
