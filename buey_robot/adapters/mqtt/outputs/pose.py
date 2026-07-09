"""PosePublisher: nodo ROS2 que agrega odometria y publica bueyuy/telemetry/json.

EXCEPCION: este es un NODO, no una clase reutilizable, porque necesita
suscribirse a multiples topics ROS2 y agregar los datos antes de publicar.

Publica por evento: cada vez que llega un mensaje en cualquiera de los
topics suscritos se republica el estado consolidado al broker MQTT.

Suscribe:
  - /odom_filtered (Odometry) -- posicion y heading (yaw del quaternion)

El heading sale del quaternion de /odom_filtered, que ya viene fusionado por la
fuente de odometria (rtk.py). Asi pose.py no depende de los topics
/heading/* (rtk ya no los publica: solo emite /odom_filtered). Para comparar los
headings (gyro crudo vs fused vs GPS), ver imu_bridge (bueyuy/heading/*).

Publica MQTT:
  - bueyuy/telemetry/json
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.utils.config import load_config, require_key
from buey_robot.utils.math import quaternion_to_yaw


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
        self._heading = 0.0
        self._odom_received = False
        self._messages_sent = 0

        # Suscriptor unico: la odometria fusionada trae posicion y heading
        self.create_subscription(Odometry, '/odom_filtered', self._odom_callback, 10)

        self.get_logger().info(f'Pose Publisher iniciado -> {self.topic_telemetry} (por evento)')

    def _odom_callback(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._heading = math.degrees(quaternion_to_yaw(q.x, q.y, q.z, q.w))
        if not self._odom_received:
            self.get_logger().info('Primer /odom_filtered recibido')
            self._odom_received = True
        self._publish_telemetry()

    def _publish_telemetry(self):
        """Publica JSON consolidado con posicion y heading (por evento)."""
        payload = json.dumps({
            'x': round(self._x, 3),
            'y': round(self._y, 3),
            'heading': round(self._heading, 1),
            'odom_active': self._odom_received,
            'timestamp': time.time(),
        })
        self._mqtt.publish(self.topic_telemetry, payload, qos=self._qos)
        self._messages_sent += 1
        if self._messages_sent == 1 or self._messages_sent % 50 == 0:
            self.get_logger().info(
                f'Telemetry: x={self._x:.2f}, y={self._y:.2f}, '
                f'heading={self._heading:.1f}, odom={self._odom_received} '
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
