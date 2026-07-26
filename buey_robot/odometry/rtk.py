"""Odometria RTK: /gps/fix -> /odom_filtered (frame ENU local). Aplica el lever-arm
(traslada la posicion de la antena GPS al centro del robot), adopta el heading de
/heading/fused, y publica odom SOLO cuando el fix es confiable; si no, /odom/status
con el motivo.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from gps_msgs.msg import GPSFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String

from buey_robot.utils.math import angle_normalize
from buey_robot.utils.filters import MovingAverageFilter
from buey_robot.utils.gps_converter import GPSConverter
from buey_robot.utils.params import load_params
from buey_robot.contracts import GPS_FIX, HEADING_FUSED, ODOM_FILTERED, ODOM_ORIGIN, ODOM_STATUS

_M_PER_DEG = 111320.0              # metros por grado de latitud (para el lever-arm)
_RTK_FIXED_MAX_ACCURACY_M = 0.035  # techo de accuracy para RTK Fixed (cm)
_RTK_FLOAT_MAX_ACCURACY_M = 0.10   # techo de accuracy para RTK Float (dm)

PARAMS = {
    'allow_float': ('gps.allow_float', bool),
    'filter_enabled': ('gps.filter.enabled', bool),
    'filter_window': ('gps.filter.window_size', int),
    'antenna_offset': ('robot.antenna.offset_x_m', float),
    'frame_id': ('gps.frame_id', str),
}


class RTKOdometry(Node):
    def __init__(self):
        super().__init__('rtk_odometry')

        load_params(self, PARAMS)

        self._converter = GPSConverter()
        self._x_filter = MovingAverageFilter(self.filter_window) if self.filter_enabled else None
        self._y_filter = MovingAverageFilter(self.filter_window) if self.filter_enabled else None
        self._heading = None       # yaw ENU (rad), de /heading/fused
        self._last_status = None

        # Origen del frame ENU: la lat/lon del primer fix es el (0,0) del frame local. Se
        # emite una sola vez, reteniendo el ultimo valor para que un subscriber que
        # conecte despues igual lo reciba.
        retained = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._odom_pub = self.create_publisher(Odometry, ODOM_FILTERED, 10)
        self._origin_pub = self.create_publisher(NavSatFix, ODOM_ORIGIN, retained)
        self._status_pub = self.create_publisher(String, ODOM_STATUS, 10)
        self.create_subscription(GPSFix, GPS_FIX, self._on_fix, 10)
        self.create_subscription(Float32, HEADING_FUSED, self._on_heading, 10)

        modo = 'RTK Float' if self.allow_float else 'RTK Fixed'
        self.get_logger().info(
            f'RTK Odometry iniciada. Minimo para confiar: {modo} '
            f'(accuracy <= {self._max_accuracy():.2f}m). Peor que eso no publica odometria.')

    def _max_accuracy(self) -> float:
        """Accuracy horizontal (m) maxima aceptada segun el modo (Float o Fixed)."""
        return _RTK_FLOAT_MAX_ACCURACY_M if self.allow_float else _RTK_FIXED_MAX_ACCURACY_M

    def _on_heading(self, msg: Float32):
        self._heading = angle_normalize(math.radians(msg.data))

    def _center(self, lat, lon):
        """Traslada la antena (offset adelante en X) al centro del robot con el heading.
        Sin heading todavia, devuelve la posicion cruda."""
        if self.antenna_offset == 0.0 or self._heading is None:
            return lat, lon
        d_east = -self.antenna_offset * math.cos(self._heading)
        d_north = -self.antenna_offset * math.sin(self._heading)
        return lat + d_north / _M_PER_DEG, lon + d_east / (_M_PER_DEG * math.cos(math.radians(lat)))

    def _on_fix(self, msg: GPSFix):
        # Accuracy horizontal (m) del fix vs el umbral del modo: peor que eso (DGPS, GPS
        # single o sin fix) = no confiable -> no se publica odometria.
        acc = math.sqrt(max(0.0, msg.position_covariance[0]))
        if acc > self._max_accuracy():
            modo = 'RTK Float' if self.allow_float else 'RTK Fixed'
            self._set_status(
                f'GPS poco preciso: accuracy {acc:.2f}m > {self._max_accuracy():.2f}m '
                f'(minimo {modo}). No publico odometria.')
            return

        lat, lon = self._center(msg.latitude, msg.longitude)
        if not self._converter.origin_set:
            self._converter.set_origin(lat, lon)
            origin = NavSatFix()
            origin.latitude, origin.longitude = lat, lon
            self._origin_pub.publish(origin)
            self.get_logger().info(
                f'Origen del frame local fijado (primer fix): lat={lat:.8f}, lon={lon:.8f}')

        x, y = self._converter.gps_to_local(lat, lon)
        if self._x_filter and self._y_filter:
            x, y = self._x_filter.update(x), self._y_filter.update(y)

        heading = self._heading if self._heading is not None else 0.0
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = self.frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = msg.altitude or 0.0
        odom.pose.pose.orientation.z = math.sin(heading / 2.0)
        odom.pose.pose.orientation.w = math.cos(heading / 2.0)
        odom.twist.twist.linear.x = msg.speed
        self._odom_pub.publish(odom)
        self._set_status('ok')

    def _set_status(self, status: str):
        if status != self._last_status:   # loguear solo las transiciones, no cada fix
            if status == 'ok':
                self.get_logger().info('Odometria confiable: publicando /odom_filtered')
            else:
                self.get_logger().warn(status)
            self._last_status = status
        self._status_pub.publish(String(data=status))


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
