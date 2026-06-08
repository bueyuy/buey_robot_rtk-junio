"""PosePublisher: nodo ROS2 que agrega odometria y publica bueyuy/telemetry/json.

EXCEPCION: este es un NODO, no una clase reutilizable, porque necesita
suscribirse a multiples topics ROS2 y agregar los datos antes de publicar.

Publica por evento: cada vez que llega un mensaje en cualquiera de los
topics suscritos se republica el estado consolidado al broker MQTT.

Suscribe:
  - /odom_filtered (Odometry)
  - /heading/zed   (Float64, grados)
  - /heading/imu   (Float64, grados)
  - /heading/gps   (Float64, grados) -- solo en outdoor_rtk

Publica MQTT:
  - bueyuy/telemetry/json
"""

import json
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.utils.config import load_config, require_key


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        mqtt_cfg = load_config('mqtt.yaml')

        # Cliente MQTT compartido (singleton)
        self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())

        self.topic_telemetry = require_key(mqtt_cfg, 'topics', 'telemetry_json')
        self._qos = require_key(mqtt_cfg, 'qos', 'telemetry')

        # Estado agregado
        self._x = 0.0
        self._y = 0.0
        self._heading_zed = 0.0
        self._heading_imu = 0.0
        self._heading_gps = 0.0
        self._odom_received = False
        self._messages_sent = 0

        # Suscriptores: fuente de odometria unificada
        self.create_subscription(Odometry, '/odom_filtered', self._odom_callback, 10)
        self.create_subscription(Float64, '/heading/zed', self._heading_zed_callback, 10)
        self.create_subscription(Float64, '/heading/imu', self._heading_imu_callback, 10)
        self.create_subscription(Float64, '/heading/gps', self._heading_gps_callback, 10)

        self.get_logger().info(f'Pose Publisher iniciado -> {self.topic_telemetry} (por evento)')

    def _odom_callback(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        if not self._odom_received:
            self.get_logger().info('Primer /odom_filtered recibido')
            self._odom_received = True
        self._publish_telemetry()

    def _heading_zed_callback(self, msg: Float64):
        self._heading_zed = msg.data
        self._publish_telemetry()

    def _heading_imu_callback(self, msg: Float64):
        self._heading_imu = msg.data
        self._publish_telemetry()

    def _heading_gps_callback(self, msg: Float64):
        self._heading_gps = msg.data
        self._publish_telemetry()

    def _publish_telemetry(self):
        """Publica JSON consolidado con posicion y headings (por evento)."""
        payload = json.dumps({
            'x': round(self._x, 3),
            'y': round(self._y, 3),
            'heading_zed': round(self._heading_zed, 1),
            'heading_imu': round(self._heading_imu, 1),
            'heading_gps': round(self._heading_gps, 1),
            'odom_active': self._odom_received,
            'timestamp': time.time(),
        })
        self._mqtt.publish(self.topic_telemetry, payload, qos=self._qos)
        self._messages_sent += 1
        if self._messages_sent == 1 or self._messages_sent % 50 == 0:
            self.get_logger().info(
                f'Telemetry: x={self._x:.2f}, y={self._y:.2f}, '
                f'h_gps={self._heading_gps:.1f}, odom={self._odom_received} '
                f'[{self._messages_sent} msgs]'
            )

    def destroy_node(self):
        # El cliente es compartido — no llamar shutdown() aqui.
        # El proceso que lo creo (main) es responsable de cerrarlo.
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
