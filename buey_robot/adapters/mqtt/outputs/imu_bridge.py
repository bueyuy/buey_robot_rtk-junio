"""imu_bridge: republica topics ROS de heading/odom al broker MQTT.

Para que el panel de telemetria (app web) pueda comparar headings, este nodo
reenvia varios topics ROS como JSON al broker, bajo el prefijo bueyuy/.

Mapeo:  topic ROS  ->  bueyuy/<topic-sin-slash-inicial>

  /heading/gyro           -> bueyuy/heading/gyro            (yaw integrado del gyro CRUDO, nodo mpu6050_gyro)
  /heading/fused          -> bueyuy/heading/fused           (gyro + offset COG GPS: absoluto ENU; el que usa rtk/odom)
  /mpu6050/imu/data       -> bueyuy/mpu6050/imu/data        (Imu crudo MPU6050: accel+GYRO)
  /heading/imu            -> bueyuy/heading/imu             (IMU en ENU, salida de rtk.py)
  /heading/gps            -> bueyuy/heading/gps             (GPS en ENU)
  /odom_filtered          -> bueyuy/odom_filtered           (pose + velocidad)

Es un NODO en adapters/mqtt/outputs/ (capa transport), asi los nodos de
sensores/odometria NO importan paho: el MQTT vive solo en esta capa.
"""

import json

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.utils.config import load_config, require_key

# (topic ROS, tipo de mensaje, periodo minimo de publicacion en seg)
BRIDGED = [
    ('/heading/gyro', Float32, 0.0),
    ('/heading/fused', Float32, 0.0),
    ('/mpu6050/imu/data', Imu, 0.1),     # throttle a ~10 Hz: MPU6050 crudo (gyro)
    ('/heading/imu', Float64, 0.0),
    ('/heading/gps', Float64, 0.0),
    ('/odom_filtered', Odometry, 0.1),   # throttle a ~10 Hz max
]


class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')

        mqtt_cfg = load_config('mqtt.yaml')
        self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())
        self._qos = require_key(mqtt_cfg, 'qos', 'telemetry')

        self._last_pub = {}
        self._counts = {}
        for ros_topic, msg_type, min_period in BRIDGED:
            mqtt_topic = 'bueyuy/' + ros_topic.lstrip('/')
            self._counts[ros_topic] = 0
            self.create_subscription(
                msg_type, ros_topic,
                self._make_cb(ros_topic, mqtt_topic, min_period), 10)
            self.get_logger().info(f'bridge: {ros_topic} -> {mqtt_topic}')

    def _make_cb(self, ros_topic, mqtt_topic, min_period):
        def cb(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            last = self._last_pub.get(ros_topic, 0.0)
            if min_period > 0.0 and (now - last) < min_period:
                return
            self._last_pub[ros_topic] = now
            payload = json.dumps(message_to_ordereddict(msg))
            self._mqtt.publish(mqtt_topic, payload, qos=self._qos)
            self._counts[ros_topic] += 1
            if self._counts[ros_topic] == 1:
                self.get_logger().info(f'primer mensaje en {mqtt_topic}')
        return cb

    def destroy_node(self):
        # Cliente MQTT compartido: no cerrarlo aqui.
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuBridge()
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
