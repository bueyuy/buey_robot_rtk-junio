"""imu_bridge: republica topics ROS de heading/odom/status al broker MQTT.

Para que el panel de telemetria (app web) y el debug puedan comparar señales, este
nodo reenvia varios topics ROS como JSON al broker, bajo el prefijo bueyuy/.

Mapeo:  topic ROS  ->  bueyuy/<topic>  (por defecto bueyuy/<topic-sin-slash>,
salvo override explicito en BRIDGED).

  /heading/fused   -> bueyuy/heading/fused y bueyuy/heading/gyro  (heading absoluto ENU, el que usa odom)
  /imu/heading     -> bueyuy/heading/gyro_raw                     (gyro CRUDO ENU, cero arbitrario; debug)
  /mpu6050/imu/data-> bueyuy/mpu6050/imu/data                     (Imu crudo MPU6050: accel+GYRO)
  /odom_filtered   -> bueyuy/odom_filtered                        (pose + velocidad)
  /imu/status      -> bueyuy/imu/status      (debug: bias, yaw_rate, calibrated)
  /gps/status      -> bueyuy/gps/status      (debug: quality, sats, accuracy, course)
  /fusion/status   -> bueyuy/fusion/status   (debug: imu_heading vs gps_course, offset, residual)

Es un NODO en adapters/mqtt/outputs/ (capa transport), asi los nodos de
sensores/odometria NO importan paho: el MQTT vive solo en esta capa.
"""

import json

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.utils.config import load_config, require_key
from buey_robot.contracts import (
    HEADING_FUSED, IMU_HEADING, IMU_STATUS, GPS_STATUS, FUSION_STATUS)

# (topic ROS, tipo, periodo minimo de publicacion en seg, topic MQTT | None).
# Si el topic MQTT es None, se usa bueyuy/<topic-ros-sin-slash>. Un mismo topic ROS
# puede ir a varios MQTT (una fila por destino) -> el estado se keyea por topic MQTT.
BRIDGED = [
    # bueyuy/heading/gyro (flecha del dashboard) = el fused (ya alineado al COG).
    (HEADING_FUSED, Float32, 0.0, 'bueyuy/heading/gyro'),
    (HEADING_FUSED, Float32, 0.0, 'bueyuy/heading/fused'),
    (IMU_HEADING, Float32, 0.0, 'bueyuy/heading/gyro_raw'),
    ('/mpu6050/imu/data', Imu, 0.1, None),   # throttle a ~10 Hz: MPU6050 crudo (gyro)
    ('/odom_filtered', Odometry, 0.1, None), # throttle a ~10 Hz max
    (IMU_STATUS, String, 0.0, None),         # debug estructurado del IMU
    (GPS_STATUS, String, 0.0, None),         # debug estructurado del GPS
    (FUSION_STATUS, String, 0.0, None),      # debug estructurado de la fusion
]


class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')

        mqtt_cfg = load_config('mqtt.yaml')
        self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())
        self._qos = require_key(mqtt_cfg, 'qos', 'telemetry')

        self._last_pub = {}
        self._counts = {}
        for ros_topic, msg_type, min_period, mqtt_override in BRIDGED:
            mqtt_topic = mqtt_override or ('bueyuy/' + ros_topic.lstrip('/'))
            self._counts[mqtt_topic] = 0
            self.create_subscription(
                msg_type, ros_topic,
                self._make_cb(mqtt_topic, min_period), 10)
            self.get_logger().info(f'bridge: {ros_topic} -> {mqtt_topic}')

    def _make_cb(self, mqtt_topic, min_period):
        def cb(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            last = self._last_pub.get(mqtt_topic, 0.0)
            if min_period > 0.0 and (now - last) < min_period:
                return
            self._last_pub[mqtt_topic] = now
            payload = json.dumps(message_to_ordereddict(msg))
            self._mqtt.publish(mqtt_topic, payload, qos=self._qos)
            self._counts[mqtt_topic] += 1
            if self._counts[mqtt_topic] == 1:
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
