"""LogBridge: reenvia los logs de ROS (/rosout) al broker MQTT para la web."""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from rcl_interfaces.msg import Log

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.utils.config import load_config, require_key

_LEVEL = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
_MIN_LEVEL = 20   # INFO en adelante


class LogBridge(Node):
    def __init__(self):
        super().__init__('log_bridge')
        # Salida: broker MQTT
        mqtt_cfg = load_config('mqtt.yaml')
        self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())
        self._qos = require_key(mqtt_cfg, 'qos', 'telemetry')
        self._topic = require_key(mqtt_cfg, 'topics', 'logs')

        # Entrada: /rosout
        # QoS de /rosout (transient_local, depth 1000) para capturar tambien los logs de arranque.
        rosout_qos = QoSProfile(depth=1000, history=HistoryPolicy.KEEP_LAST,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Log, '/rosout', self._on_log, rosout_qos)
        self.get_logger().info('log_bridge iniciado (/rosout -> MQTT)')

    def _on_log(self, msg):
        if msg.level < _MIN_LEVEL or msg.name == self.get_name():   # no reenviar los propios (loop)
            return
        self._mqtt.publish(self._topic, json.dumps({
            'level': _LEVEL.get(msg.level, msg.level),
            'node': msg.name,
            'msg': msg.msg,
            'stamp': msg.stamp.sec + msg.stamp.nanosec * 1e-9,
        }), qos=self._qos)


def main(args=None):
    rclpy.init(args=args)
    node = LogBridge()
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
