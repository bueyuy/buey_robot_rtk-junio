#!/usr/bin/env python3
"""
Script para enviar waypoints al controlador de trayectoria GPS.
"""

import rclpy
from rclpy.node import Node
import sys


class WaypointSender(Node):
    def __init__(self, waypoint_file):
        super().__init__('waypoint_sender')
        self.waypoint_file = waypoint_file

    def send_waypoints(self):
        """Envía waypoints al controlador."""
        # Importar aquí para evitar errores si el paquete no está compilado
        from buey_robot.gps_trajectory_controller import GPSTrajectoryController

        self.get_logger().info(f'Enviando waypoints desde: {self.waypoint_file}')

        # Por ahora, simplemente informamos al usuario
        # En una implementación completa, usaríamos un servicio ROS2 o topic
        self.get_logger().info('Para cargar waypoints:')
        self.get_logger().info(f'  1. Asegúrate que gps_trajectory_controller está corriendo')
        self.get_logger().info(f'  2. El controlador cargará waypoints del archivo especificado')
        self.get_logger().info(f'  3. Archivo: {self.waypoint_file}')


def main(args=None):
    if len(sys.argv) < 2:
        print('Uso: ros2 run buey_robot send_waypoints <archivo_waypoints.yaml>')
        print('Ejemplo: ros2 run buey_robot send_waypoints waypoints/ejemplo_trayectoria.yaml')
        return

    waypoint_file = sys.argv[1]

    rclpy.init(args=args)
    node = WaypointSender(waypoint_file)

    node.send_waypoints()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
