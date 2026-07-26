"""TelemetryBridge: espeja el estado del stack ROS al broker MQTT para la web.
Publica la pose consolidada (bueyuy/telemetry/json) y reenvia varios topics como JSON."""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.utils.config import load_config, require_key
from buey_robot.utils.math import quaternion_to_yaw
from buey_robot.contracts import ODOM, HEADING_FUSED, IMU_YAW, IMU_STATUS, GPS_STATUS

# (topic ROS, tipo, periodo minimo de publicacion en seg, topic MQTT | None).
# Si el topic MQTT es None, se usa bueyuy/<topic-ros-sin-slash>.
BRIDGED = [
    (HEADING_FUSED, Float32, 0.0, 'bueyuy/heading/fused'),
    (IMU_YAW, Float32, 0.0, 'bueyuy/heading/gyro_raw'),
    ('/mpu6050/imu/data', Imu, 0.1, None),
    (ODOM, Odometry, 0.1, None),
    (IMU_STATUS, String, 0.0, None),
    (GPS_STATUS, String, 0.0, None),
]


class TelemetryBridge(Node):
    def __init__(self):
        super().__init__('telemetry_bridge')
        mqtt_cfg = load_config('mqtt.yaml')
        self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())
        self._qos = require_key(mqtt_cfg, 'qos', 'telemetry')
        self._telemetry_topic = require_key(mqtt_cfg, 'topics', 'telemetry_json')
        self._last_pub = {}

        self.create_subscription(Odometry, ODOM, self._on_odom, 10)
        for ros_topic, msg_type, min_period, override in BRIDGED:
            mqtt_topic = override or ('bueyuy/' + ros_topic.lstrip('/'))
            self.create_subscription(msg_type, ros_topic, self._make_cb(mqtt_topic, min_period), 10)
        self.get_logger().info('telemetry_bridge iniciado (ROS -> MQTT para la web)')

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self._mqtt.publish(self._telemetry_topic, json.dumps({
            'x': round(msg.pose.pose.position.x, 3),
            'y': round(msg.pose.pose.position.y, 3),
            'heading': round(math.degrees(quaternion_to_yaw(q.x, q.y, q.z, q.w)), 1),
            'timestamp': time.time(),
        }), qos=self._qos)

    def _make_cb(self, mqtt_topic, min_period):
        def cb(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            if min_period > 0.0 and (now - self._last_pub.get(mqtt_topic, 0.0)) < min_period:
                return
            self._last_pub[mqtt_topic] = now
            self._mqtt.publish(mqtt_topic, json.dumps(message_to_ordereddict(msg)), qos=self._qos)
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
