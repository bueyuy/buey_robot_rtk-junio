"""TelemetryBridge: reenvia los topics de estado del stack ROS al broker MQTT
(bueyuy/<topic-sin-slash>), throttleado a la tasa de telemetria (mqtt.yaml)."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.utils.config import load_config, require_key
from buey_robot.contracts import NAV_STATUS, GEO_POSITION, GPS_STATUS, IMU_STATUS, IMU_YAW

BRIDGED = [
    (NAV_STATUS, String),
    (GEO_POSITION, String),
    (GPS_STATUS, String),
    (IMU_STATUS, String),
    (IMU_YAW, Float32),
]


class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')
        # Salida: broker MQTT
        mqtt_cfg = load_config('mqtt.yaml')
        self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())
        self._qos = require_key(mqtt_cfg, 'qos', 'telemetry')
        self._min_period = 1.0 / require_key(mqtt_cfg, 'telemetry_hz')
        self._last_pub = {}

        # Entradas: topics ROS de estado
        for ros_topic, msg_type in BRIDGED:
            mqtt_topic = 'bueyuy/' + ros_topic.lstrip('/')
            self.create_subscription(msg_type, ros_topic, self._make_cb(mqtt_topic), 10)
        self.get_logger().info(f'telemetry_bridge iniciado (ROS -> MQTT, {1.0 / self._min_period:.0f}Hz)')

    def _make_cb(self, mqtt_topic):
        def cb(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            if (now - self._last_pub.get(mqtt_topic, 0.0)) < self._min_period:
                return
            self._last_pub[mqtt_topic] = now
            payload = msg.data if isinstance(msg, String) else json.dumps(round(msg.data, 2))
            self._mqtt.publish(mqtt_topic, payload, qos=self._qos)
        return cb


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryBridge()
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
