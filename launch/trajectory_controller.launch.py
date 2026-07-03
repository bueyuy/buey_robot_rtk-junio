#!/usr/bin/env python3
"""Launch del trajectory_controller (navegacion autonoma) para outdoor RTK.

Separado de outdoor_rtk.launch.py: ese levanta sensores + odometria + telemetria;
este levanta solo el controller. Asi se puede correr la telemetria sin navegar, y
reiniciar/tunear el controller sin reiniciar la odometria (que re-esperaria la BASE
y reiniciaria /odom_filtered).

Requiere que /odom_filtered ya este publicando (outdoor_rtk.launch.py corriendo).
Con goal_source: mqtt_positions (default outdoor) navega BASE->START.

Nodos:
  - navigation/controller.py  (ARC/FINAL_APPROACH -> /cmd_vel)

motor_gateway NO se incluye — corre en terminal aparte con motor_gateway.launch.py.

Uso:
  ros2 launch buey_robot trajectory_controller.launch.py
  ros2 launch buey_robot trajectory_controller.launch.py waypoints_file:=/ruta/a/waypoints.yaml
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('buey_robot')
    default_waypoints = os.path.join(pkg_share, 'waypoints', 'rectangulo_local.yaml')

    nav_yaml = os.path.join(pkg_share, 'config', 'navigation.yaml')
    nav_outdoor_yaml = os.path.join(pkg_share, 'config', 'navigation_outdoor.yaml')
    motor_yaml = os.path.join(pkg_share, 'config', 'motor.yaml')

    waypoints_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints,
        description='Ruta al YAML de waypoints x,y locales (solo si goal_source=waypoints_file)'
    )

    # navigation_outdoor.yaml fija goal_source=mqtt_waypoints: la ruta lat/lon llega
    # por MQTT (bueyuy/waypoints) y se fija al origen BASE. Para navegar un YAML local
    # x,y relativo al arranque:
    #   ros2 launch ... goal_source:=waypoints_file waypoints_file:=/ruta/local.yaml
    goal_source_arg = DeclareLaunchArgument(
        'goal_source',
        default_value='mqtt_waypoints',
        description='mqtt_waypoints (ruta lat/lon por MQTT, origen=BASE) | waypoints_file (x,y local)'
    )

    controller_node = Node(
        package='buey_robot',
        executable='trajectory_controller',
        name='trajectory_controller',
        output='screen',
        parameters=[
            nav_yaml,
            nav_outdoor_yaml,
            motor_yaml,
            {
                'auto_load_waypoints': LaunchConfiguration('waypoints_file'),
                'goal_source': LaunchConfiguration('goal_source'),
            },
        ]
    )

    return LaunchDescription([
        waypoints_arg,
        goal_source_arg,
        controller_node,
    ])
