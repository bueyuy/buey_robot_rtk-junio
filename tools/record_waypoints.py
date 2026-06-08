#!/usr/bin/env python3
"""
Script para grabar waypoints GPS mientras caminas/conduces el campo.
Presiona Enter para grabar un waypoint, 'q' para terminar y guardar.
"""

import rclpy
from rclpy.node import Node
from gps_msgs.msg import GPSFix
import yaml
import sys
import select


class WaypointRecorder(Node):
    def __init__(self, output_file):
        super().__init__('waypoint_recorder')

        self.output_file = output_file
        self.waypoints = []
        self.current_gps = None

        # Subscriber GPS
        self.gps_sub = self.create_subscription(
            GPSFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

        self.get_logger().info('Waypoint Recorder iniciado')
        self.get_logger().info('Presiona ENTER para grabar waypoint actual')
        self.get_logger().info('Presiona "q" + ENTER para terminar y guardar')

    def gps_callback(self, msg: GPSFix):
        """Callback GPS."""
        self.current_gps = msg

    def record_waypoint(self):
        """Graba el waypoint actual."""
        if self.current_gps is None:
            self.get_logger().warn('No hay fix GPS disponible')
            return

        waypoint = {
            'latitude': self.current_gps.latitude,
            'longitude': self.current_gps.longitude,
            'altitude': self.current_gps.altitude
        }

        self.waypoints.append(waypoint)

        self.get_logger().info(
            f'Waypoint {len(self.waypoints)} grabado: '
            f'lat={waypoint["latitude"]:.6f}, lon={waypoint["longitude"]:.6f}'
        )

    def save_waypoints(self):
        """Guarda waypoints a archivo YAML."""
        if not self.waypoints:
            self.get_logger().warn('No hay waypoints para guardar')
            return

        # Simplificar waypoints (solo lat/lon)
        simplified_waypoints = []
        for wp in self.waypoints:
            simplified_waypoints.append({
                'latitude': wp['latitude'],
                'longitude': wp['longitude']
            })

        data = {'waypoints': simplified_waypoints}

        try:
            with open(self.output_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)

            self.get_logger().info(f'Guardados {len(self.waypoints)} waypoints en: {self.output_file}')
        except Exception as e:
            self.get_logger().error(f'Error guardando waypoints: {str(e)}')


def main(args=None):
    output_file = 'recorded_waypoints.yaml'

    if len(sys.argv) >= 2:
        output_file = sys.argv[1]

    rclpy.init(args=args)
    node = WaypointRecorder(output_file)

    print('\n=== Grabador de Waypoints GPS ===')
    print(f'Archivo de salida: {output_file}')
    print('Esperando fix GPS...\n')

    try:
        while rclpy.ok():
            # Procesar callbacks ROS2
            rclpy.spin_once(node, timeout_sec=0.1)

            # Verificar input del usuario (non-blocking)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip()

                if line.lower() == 'q':
                    # Guardar y salir
                    node.save_waypoints()
                    break
                else:
                    # Grabar waypoint
                    node.record_waypoint()

    except KeyboardInterrupt:
        print('\nInterrumpido por usuario')
        node.save_waypoints()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
