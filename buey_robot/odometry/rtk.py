"""Odometria RTK: convierte /gps/fix -> /odom_filtered usando GPSConverter.

El heading sale del nodo heading_fusion (/heading/fused = gyro + COG GPS, ENU); con
use_imu_heading=false cae al COG del GPS. Produce /odom_filtered, que consume
navigation/controller.py sin saber como se genero.
"""

import rclpy
from rclpy.node import Node
from gps_msgs.msg import GPSFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.adapters.mqtt.outputs.origin import MqttOriginOutput
from buey_robot.utils.config import load_config, require_key
from buey_robot.utils.math import angle_normalize
from buey_robot.utils.filters import MovingAverageFilter, ExponentialFilter
from buey_robot.utils.gps_converter import GPSConverter
from buey_robot.contracts import GPS_FIX
from buey_robot.utils.params import load_params

# Accuracy horizontal (m) por debajo de la cual un fix se considera RTK Fixed (cm)
# en vez de Float (decimetro). Alineado con la covarianza del driver gps_nmea:
# Fixed -> 0.02, Float -> 0.05. Solo se usa si gps.origin.allow_float=false.
_RTK_FIXED_MAX_ACCURACY_M = 0.035

PARAMS = {
    'origin_allow_float': ('gps.origin.allow_float', bool),
    'filter_enabled': ('gps.filter.enabled', bool),
    'filter_window': ('gps.filter.window_size', int),
    'min_movement': ('gps.heading.min_movement_threshold', float),
    'heading_alpha': ('gps.heading.filter_alpha', float),
    'use_imu_heading': ('gps.imu.use_imu_heading', bool),
    'imu_heading_topic': ('gps.imu.heading_topic', str),
    'frame_id': ('gps.frame_id', str),
}


class RTKOdometry(Node):
    def __init__(self):
        super().__init__('rtk_odometry')

        load_params(self, PARAMS)

        # Conversor GPS y filtros
        self.gps_converter = GPSConverter()
        self.x_filter = MovingAverageFilter(self.filter_window) if self.filter_enabled else None
        self.y_filter = MovingAverageFilter(self.filter_window) if self.filter_enabled else None
        self.heading_filter = ExponentialFilter(self.heading_alpha)

        # Estado
        self.current_x = None
        self.current_y = None
        self.current_heading = None
        self.current_speed = 0.0
        self.gps_received = False
        self.imu_heading_received = False

        # Publishers
        self.odom_filtered_pub = self.create_publisher(Odometry, '/odom_filtered', 10)
        #self.heading_pub = self.create_publisher(Float64, '/heading/gps', 10)
        #self.heading_imu_pub = self.create_publisher(Float64, '/heading/imu', 10)

        # Subscribers
        self.create_subscription(GPSFix, GPS_FIX, self.gps_callback, 10)
        # Heading fusionado (gyro + COG GPS) desde el nodo heading_fusion, ya en yaw
        # ENU (std_msgs/Float32, GRADOS). El topic es configurable
        # (gps.imu.heading_topic -> /heading/fused). Con use_imu_heading se adopta
        # directo como current_heading; sin el, el heading sale del COG del GPS.
        self.create_subscription(Float32, self.imu_heading_topic, self.imu_callback, 10)
        if self.use_imu_heading:
            self.get_logger().info(
                f'RTK Odometry: heading desde {self.imu_heading_topic} (gyro+GPS, ENU)')
        else:
            self.get_logger().info('RTK Odometry: heading solo-GPS (COG)')

        # Salida MQTT (capa transport; rtk no importa paho): ORIGEN del frame ENU
        # (primer fix, donde arranca el robot) -> el controller lo consume para
        # convertir los waypoints al mismo frame. La posicion del robot (centro, ya
        # con lever-arm de antena) la publica el driver GPS en rtk/location/json:
        # una unica ubicacion, no hay cruda + corregida.
        mqtt_cfg = load_config('mqtt.yaml')
        mqtt = get_client(mqtt_cfg, logger=self.get_logger())
        self._origin_output = MqttOriginOutput(mqtt, mqtt_cfg)

        self.get_logger().info('RTK Odometry iniciado')
        self.get_logger().info('  Origen: primer fix GPS (auto), publicado a la nav')

    def _origin_fix_ok(self, msg: GPSFix) -> bool:
        """True si el fix sirve para fijar el origen. allow_float=true acepta cualquier
        fix (ya paso el gate RTK); false exige RTK Fixed (accuracy de cm por covarianza)."""
        if self.origin_allow_float:
            return True
        acc = math.sqrt(max(0.0, msg.position_covariance[0]))
        return acc <= _RTK_FIXED_MAX_ACCURACY_M

    def gps_callback(self, msg: GPSFix):
        """Callback GPS: convierte a ENU, calcula heading, publica /odom_filtered.
        La calidad del fix ya la gatea el driver GPS antes de /gps/fix."""
        # Origen del frame ENU: primer fix GPS (donde arranca el robot). Se publica
        # a MQTT para que el controller convierta los waypoints al mismo frame. Con
        # allow_float=false, espera un RTK Fixed; con true, sirve tambien Float.
        if not self.gps_converter.origin_set:
            if self._origin_fix_ok(msg):
                self.gps_converter.set_origin(msg.latitude, msg.longitude)
                self._origin_output.send(msg.latitude, msg.longitude)
                acc = math.sqrt(max(0.0, msg.position_covariance[0]))
                kind = 'FIXED' if acc <= _RTK_FIXED_MAX_ACCURACY_M else 'FLOAT'
                self.get_logger().info(
                    f'Origen GPS (primer fix, {kind}, ~{acc:.2f}m): '
                    f'lat={msg.latitude:.8f}, lon={msg.longitude:.8f}')
                self.gps_received = True
            else:
                self.get_logger().warn(
                    'Esperando RTK Fixed para el origen (allow_float=false)...',
                    throttle_duration_sec=5.0)

        if not self.gps_converter.origin_set:
            return

        # msg.latitude/longitude YA es el centro del robot: el driver GPS aplica el
        # lever-arm de la antena antes de publicar /gps/fix (unica ubicacion).
        x, y = self.gps_converter.gps_to_local(msg.latitude, msg.longitude)

        if self.filter_enabled and self.x_filter and self.y_filter:
            x = self.x_filter.update(x)
            y = self.y_filter.update(y)

        self.current_x = x
        self.current_y = y

        self._update_heading(msg.speed, msg.track)
        self._publish_odometry(msg)

    def imu_callback(self, msg: Float32):
        """Heading fusionado (gyro + COG GPS) desde heading_fusion (/heading/fused),
        ya en yaw ENU. Se adopta directo como current_heading cuando use_imu_heading;
        rtk NO re-transforma ni re-fusiona (la fusion ya la hizo heading_fusion)."""
        if not self.use_imu_heading:
            return
        self.current_heading = angle_normalize(math.radians(msg.data))
        self.imu_heading_received = True

    def _update_heading(self, speed: float, track: float):
        """Actualiza el heading. Con use_imu_heading lo pone imu_callback
        (/heading/fused, ENU). Sin el, se deriva del COG del GPS.

        track viene en convencion brujula (0=Norte, horario) -> yaw ENU (0=Este,
        antihorario), consistente con el resto del sistema.
        """
        self.current_speed = speed

        if self.use_imu_heading:
            return  # el heading lo provee /heading/fused via imu_callback

        if self.current_speed > self.min_movement:
            # COG brujula -> yaw ENU: yaw = 90 - track
            self.current_heading = angle_normalize(math.radians(90.0 - track))

    def _publish_odometry(self, gps_msg: GPSFix):
        """Publica nav_msgs/Odometry a /odom_filtered."""
        if self.current_x is None:
            return
        if self.current_heading is None:
            # Sin heading todavia, usar 0
            heading = 0.0
        else:
            heading = self.current_heading

        msg = Odometry()
        msg.header.stamp = gps_msg.header.stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = self.frame_id

        msg.pose.pose.position.x = self.current_x
        msg.pose.pose.position.y = self.current_y
        msg.pose.pose.position.z = gps_msg.altitude if gps_msg.altitude else 0.0

        # Orientacion desde heading (yaw)
        msg.pose.pose.orientation.z = math.sin(heading / 2.0)
        msg.pose.pose.orientation.w = math.cos(heading / 2.0)

        # Velocidad aproximada
        msg.twist.twist.linear.x = self.current_speed

        self.odom_filtered_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RTKOdometry()
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
