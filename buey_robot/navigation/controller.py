"""Controlador de navegacion: sigue una ruta de waypoints (x/y) con /odom en modo
stop_and_turn; genera /nav/cmd_vel. Frena si /odom no esta fresco."""

import json
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String

from buey_robot.navigation.waypoint_manager import WaypointManager
from buey_robot.navigation.control.stop_and_turn import StopAndTurnControl, StopAndTurnParams
from buey_robot.utils.params import load_params
from buey_robot.utils.log import TransitionLogger
from buey_robot.utils.math import angle_diff
from buey_robot.contracts import ODOM, LOCAL_ROUTE, NAV_START, NAV_STATUS, NAV_CMD_VEL

PARAMS = {
    'frequency': ('frequency_hz', float),
    'odom_timeout': ('odom_timeout_s', float),
    'goal_tolerance': ('goal.position_tolerance_m', float),
    'alignment_tolerance_deg': ('goal.alignment_tolerance_deg', float),
    'cruise_linear': ('velocity.cruise_linear_m_s', float),
    'min_linear': ('velocity.min_linear_m_s', float),
    'cruise_angular': ('velocity.cruise_angular_rad_s', float),
    'max_angular': ('velocity.max_angular_rad_s', float),
    'angular_gain': ('angular.proportional_gain', float),
    'decel_distance': ('deceleration.distance_m', float),
}


class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        load_params(self, PARAMS)

        # Estado
        self._x = self._y = self._heading = None
        self._last_odom = None
        self._route_loaded = False
        self._loop_route = False
        self._navigating = False
        self._nav_log = TransitionLogger(self.get_logger())   # estado del loop, solo en transiciones
        self._wp = WaypointManager()
        self._control = StopAndTurnControl(StopAndTurnParams(
            cruise_linear=self.cruise_linear, cruise_angular=self.cruise_angular,
            angular_gain=self.angular_gain, max_angular=self.max_angular,
            min_linear=self.min_linear, goal_tolerance=self.goal_tolerance,
            alignment_tolerance=math.radians(self.alignment_tolerance_deg),
            decel_distance=self.decel_distance))

        # Entradas
        self.create_subscription(Odometry, ODOM, self._on_odom, 10)
        self.create_subscription(String, LOCAL_ROUTE, self._on_route, 10)
        self.create_subscription(Empty, NAV_START, self._on_start, 10)

        # Salidas
        self._cmd_pub = self.create_publisher(Twist, NAV_CMD_VEL, 10)
        self._status_pub = self.create_publisher(String, NAV_STATUS, 10)

        self.create_timer(1.0 / self.frequency, self._loop)
        self.get_logger().info(
            f'navigation_controller iniciado (freq={self.frequency}Hz, odom_timeout={self.odom_timeout}s)')

    def _loop(self):
        if not self._odom_fresh():
            self._nav_log.info('sin odometria fresca -> detenido')
            self._publish_status('no_odom')
            self._stop()
            return
        if not self._navigating:
            self._nav_log.info('idle (esperando GO)')
            self._publish_status('idle')
            self._stop()
            return
        if self._wp.is_complete():
            if self._loop_route:
                self._wp.restart()
                self._control.reset()
                self.get_logger().info('ruta completa -> reiniciando (loop)')
            else:
                self._navigating = False
                self._publish_status('completed')
                self._stop()
                self.get_logger().info('ruta completada')
                return
        pose = (self._x, self._y, self._heading)
        linear, angular, reached, status = self._control.compute(pose, self._wp.current_goal())
        if reached:
            self._wp.advance()
            if not self._wp.is_complete():
                self._control.reset()
            self._stop()
            self.get_logger().info(f'WP alcanzado ({self._wp.progress_string()})')
            return
        self._nav_log.reset()                                     # al volver a idle/stale re-loguea
        self.get_logger().info(status, throttle_duration_sec=2.0)  # progreso (dist) cada 2s
        self._publish_status('navigating', linear, angular)
        self._publish(linear, angular)

    def _publish_status(self, state, v=0.0, w=0.0):
        goal = self._wp.current_goal()
        dist = herr = None
        if goal is not None and self._x is not None:
            dist = round(math.hypot(goal[0] - self._x, goal[1] - self._y), 2)
            herr = round(math.degrees(angle_diff(math.atan2(goal[1] - self._y, goal[0] - self._x),
                                                 self._heading)), 1)
        self._status_pub.publish(String(data=json.dumps({
            'state': state, 'wp': self._wp.index + 1, 'wp_total': self._wp.total,
            'distance_m': dist, 'heading_error_deg': herr,
            'x': round(self._x, 3) if self._x is not None else None,
            'y': round(self._y, 3) if self._y is not None else None,
            'heading_deg': round(math.degrees(self._heading), 1) if self._heading is not None else None,
            'v': round(v, 3), 'w': round(w, 3),
        })))

    def _on_odom(self, msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._heading = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)   # yaw del quaternion
        self._last_odom = self.get_clock().now()

    def _odom_fresh(self):
        if self._last_odom is None:
            return False
        age = (self.get_clock().now() - self._last_odom).nanoseconds * 1e-9
        return age <= self.odom_timeout

    def _on_route(self, msg):
        try:
            route = json.loads(msg.data)
        except Exception:
            return
        wps = route.get('waypoints', [])
        if not wps:                                # ruta vacia = descartar y quedar en idle
            self._route_loaded = False
            self._navigating = False
            self._stop()
            self.get_logger().info('ruta vacia -> descartada, idle')
            return
        self._wp.set_waypoints([(w['x'], w['y']) for w in wps])
        self._loop_route = bool(route.get('loop', False))
        self._route_loaded = True
        self._navigating = False                   # ruta lista pero NO arranca: espera el GO
        self._control.reset()
        self.get_logger().info(f'ruta cargada: {len(wps)} waypoints (loop={self._loop_route}), esperando GO')

    def _on_start(self, msg):
        if not self._route_loaded:
            self.get_logger().warn('GO ignorado: no hay ruta cargada')
            return
        self._wp.restart()
        self._control.reset()
        self._navigating = True
        self.get_logger().info('GO -> navegando')

    def _publish(self, linear, angular):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self._cmd_pub.publish(t)

    def _stop(self):
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = NavigationController()
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
