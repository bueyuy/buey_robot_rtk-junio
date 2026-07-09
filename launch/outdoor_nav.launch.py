#!/usr/bin/env python3
"""Launch UNICO para pruebas de navegacion outdoor (RTK).

Compone en un solo comando lo que normalmente se corre en 3 terminales:
  - motor_gateway.launch.py        (joystick_controller + motor_gateway)
  - outdoor_rtk.launch.py          (agente microROS + GPS + gyro/heading + odom + TELEMETRIA)
  - trajectory_controller.launch.py (navegacion; goal_source=mqtt_waypoints por default)

Reusa los launches existentes (no duplica nodos). El motor_gateway se incluye UNA
sola vez, asi que NO hay que correr tambien motor_gateway.launch.py aparte (eso
daria dos gateways). Usar ESTE launch EN VEZ de los tres por separado.

El controller arranca con un delay para que la odometria (/odom_filtered) ya este
publicando. Igual el controller espera internamente odom + BASE + calibracion del
gyro + waypoints, asi que el orden exacto no es critico.

Uso:
  ros2 launch buey_robot outdoor_nav.launch.py
  ros2 launch buey_robot outdoor_nav.launch.py goal_source:=waypoints_file \
       waypoints_file:=/ruta/local.yaml
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('buey_robot')
    launch_dir = os.path.join(pkg_share, 'launch')
    default_waypoints = os.path.join(pkg_share, 'waypoints', 'rectangulo_local.yaml')

    # Reenviados al trajectory_controller (mismos defaults que ese launch).
    goal_source_arg = DeclareLaunchArgument(
        'goal_source', default_value='mqtt_waypoints',
        description='mqtt_waypoints (ruta lat/lon por MQTT, origen=BASE) | waypoints_file (x,y local)')
    waypoints_arg = DeclareLaunchArgument(
        'waypoints_file', default_value=default_waypoints,
        description='Ruta al YAML de waypoints x,y locales (solo si goal_source=waypoints_file)')

    def include(name, launch_arguments=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=launch_arguments.items() if launch_arguments else None,
        )

    # 1) Drive manual (joystick) + salida a motores. Siempre disponible.
    motor_gateway = include('motor_gateway.launch.py')

    # 2) Sensores + odometria RTK + heading fused + TELEMETRIA (pose + imu_bridge).
    outdoor_rtk = include('outdoor_rtk.launch.py')

    # 3) Navegacion. Delay para que /odom_filtered ya este arriba.
    ctrl_args = {
        'goal_source': LaunchConfiguration('goal_source'),
        'waypoints_file': LaunchConfiguration('waypoints_file'),
    }
    controller = TimerAction(
        period=6.0,
        actions=[include('trajectory_controller.launch.py', ctrl_args)],
    )

    return LaunchDescription([
        goal_source_arg,
        waypoints_arg,
        motor_gateway,
        outdoor_rtk,
        controller,
    ])
