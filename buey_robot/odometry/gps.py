"""Odometria GPS: /gps/fix -> /odom (frame local x/y). Fija el datum internamente
(primer fix confiable), traslada la antena al centro del robot (lever-arm en x/y),
adopta el heading de /heading/fused y publica solo cuando el fix es confiable."""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from gps_msgs.msg import GPSFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

from buey_robot.utils.gps_converter import GPSConverter
from buey_robot.utils.filters import MovingAverageFilter
from buey_robot.utils.params import load_params
from buey_robot.contracts import GPS_FIX, HEADING_FUSED, ODOM, GEO_ROUTE, LOCAL_ROUTE

PARAMS = {
    'allow_float': ('gps.allow_float', bool),
    'fixed_max_accuracy': ('gps.rtk.fixed_max_accuracy_m', float),
    'float_max_accuracy': ('gps.rtk.float_max_accuracy_m', float),
    'filter_enabled': ('gps.filter.enabled', bool),
    'filter_window': ('gps.filter.window_size', int),
    'antenna_offset': ('robot.antenna.offset_x_m', float),
    'frame_id': ('gps.frame_id', str),
}


class OdometryGps(Node):
    def __init__(self):
        super().__init__('odometry_gps')
        load_params(self, PARAMS)
        self._conv = GPSConverter()
        self._xf = MovingAverageFilter(self.filter_window) if self.filter_enabled else None
        self._yf = MovingAverageFilter(self.filter_window) if self.filter_enabled else None
        self._heading = None         # rad, yaw ENU de /heading/fused
        self._pending_route = None   # ruta geo que llego antes de fijar el datum
        self._reliable = None        # estado del gate de accuracy (para loguear transiciones)
        self._odom_pub = self.create_publisher(Odometry, ODOM, 10)
        retained = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._route_pub = self.create_publisher(String, LOCAL_ROUTE, retained)
        self.create_subscription(GPSFix, GPS_FIX, self._on_fix, 10)
        self.create_subscription(Float32, HEADING_FUSED, self._on_heading, 10)
        self.create_subscription(String, GEO_ROUTE, self._on_route, 10)
        self.get_logger().info(
            f'odometry_gps iniciado: minimo {"RTK Float" if self.allow_float else "RTK Fixed"} '
            f'(accuracy <= {self._max_accuracy():.2f}m). Peor que eso no publica /odom.')

    def _max_accuracy(self):
        return self.float_max_accuracy if self.allow_float else self.fixed_max_accuracy

    def _on_heading(self, msg):
        self._heading = math.radians(msg.data)

    def _on_fix(self, msg):
        acc = math.sqrt(max(0.0, msg.position_covariance[0]))
        if acc > self._max_accuracy():        # fix poco preciso -> no publico odometria
            if self._reliable is not False:
                self._reliable = False
                self.get_logger().warn(
                    f'GPS poco preciso: accuracy {acc:.2f}m > {self._max_accuracy():.2f}m '
                    f'({"RTK Float" if self.allow_float else "RTK Fixed"}). No publico /odom.')
            return
        if self._reliable is not True:
            self._reliable = True
            self.get_logger().info(f'GPS confiable (accuracy {acc:.2f}m) -> publicando /odom')
        if not self._conv.origin_set:          # datum = primer fix confiable
            self._conv.set_origin(msg.latitude, msg.longitude)
            self.get_logger().info(
                f'datum fijado (primer fix): lat={msg.latitude:.7f} lon={msg.longitude:.7f}')
            if self._pending_route is not None:
                self._publish_route(self._pending_route)
                self._pending_route = None
        x, y = self._conv.gps_to_local(msg.latitude, msg.longitude)
        x, y = self._to_center(x, y)
        if self._xf:
            x, y = self._xf.update(x), self._yf.update(y)
        h = self._heading if self._heading is not None else 0.0
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = self.frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = msg.altitude or 0.0
        odom.pose.pose.orientation.z = math.sin(h / 2.0)
        odom.pose.pose.orientation.w = math.cos(h / 2.0)
        odom.twist.twist.linear.x = msg.speed
        self._odom_pub.publish(odom)

    def _to_center(self, x, y):
        # Lever-arm en x/y: la antena esta adelante del centro (offset en X), rotar por el heading.
        if self.antenna_offset == 0.0 or self._heading is None:
            return x, y
        return (x - self.antenna_offset * math.cos(self._heading),
                y - self.antenna_offset * math.sin(self._heading))

    def _on_route(self, msg):
        # Convierte la ruta geografica a x/y con el datum. Sin datum aun -> la guarda.
        try:
            route = json.loads(msg.data)
        except Exception:
            return
        if not self._conv.origin_set:
            self._pending_route = route
            return
        self._publish_route(route)

    def _publish_route(self, route):
        xy = [dict(zip(('x', 'y'), self._conv.gps_to_local(wp['lat'], wp['lon'])))
              for wp in route.get('waypoints', [])]
        self._route_pub.publish(String(data=json.dumps(
            {'waypoints': xy, 'loop': bool(route.get('loop', False))})))
        self.get_logger().info(f'ruta convertida a x/y: {len(xy)} waypoints')


def main(args=None):
    rclpy.init(args=args)
    node = OdometryGps()
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
