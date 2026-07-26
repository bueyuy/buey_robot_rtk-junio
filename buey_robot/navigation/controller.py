"""Controlador de navegacion: lleva el robot por una ruta de waypoints (x/y del frame
local) usando /odom, en modo stop_and_turn (frena, gira, avanza) waypoint por waypoint.
Genera /nav/cmd_vel. Frena si /odom deja de estar fresco."""

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
from buey_robot.contracts import ODOM, LOCAL_ROUTE, NAV_START, NAV_CMD_VEL

PARAMS = {
    'frequency': ('controller.frequency_hz', float),
    'odom_timeout': ('controller.odom_timeout_s', float),
    'goal_tolerance': ('controller.goal.position_tolerance_m', float),
    'alignment_tolerance_deg': ('controller.goal.alignment_tolerance_deg', float),
    'cruise_linear': ('controller.velocity.cruise_linear_m_s', float),
    'min_linear': ('controller.velocity.min_linear_m_s', float),
    'cruise_angular': ('controller.velocity.cruise_angular_rad_s', float),
    'max_angular': ('controller.velocity.max_angular_rad_s', float),
    'angular_gain': ('controller.angular.proportional_gain', float),
    'decel_distance': ('controller.deceleration.distance_m', float),
}


class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        load_params(self, PARAMS)
        self._x = self._y = self._heading = None
        self._last_odom = None
        self._route_loaded = False
        self._loop_route = False
        self._navigating = False
        self._wp = WaypointManager()
        self._control = StopAndTurnControl(StopAndTurnParams(
            cruise_linear=self.cruise_linear, cruise_angular=self.cruise_angular,
            angular_gain=self.angular_gain, max_angular=self.max_angular,
            min_linear=self.min_linear, goal_tolerance=self.goal_tolerance,
            alignment_tolerance=math.radians(self.alignment_tolerance_deg),
            decel_distance=self.decel_distance))
        self.create_subscription(Odometry, ODOM, self._on_odom, 10)
        self.create_subscription(String, LOCAL_ROUTE, self._on_route, 10)
        self.create_subscription(Empty, NAV_START, self._on_start, 10)
        self._cmd_pub = self.create_publisher(Twist, NAV_CMD_VEL, 10)
        self.create_timer(1.0 / self.frequency, self._loop)

    def _loop(self):
        if not self._odom_fresh() or not self._navigating:
            self._stop()
            return
        if self._wp.is_complete():
            if self._loop_route:
                self._wp.restart()
                self._control.reset()
            else:
                self._navigating = False
                self._stop()
                return
        pose = (self._x, self._y, self._heading)
        linear, angular, reached, _ = self._control.compute(pose, self._wp.current_goal())
        if reached:
            self._wp.advance()
            if not self._wp.is_complete():
                self._control.reset()
            self._stop()
            return
        self._publish(linear, angular)

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
            return
        self._wp.set_waypoints([(w['x'], w['y']) for w in wps])
        self._loop_route = bool(route.get('loop', False))
        self._route_loaded = True
        self._navigating = False                   # ruta lista pero NO arranca: espera el GO
        self._control.reset()

    def _on_start(self, msg):
        if not self._route_loaded:
            return
        self._wp.restart()
        self._control.reset()
        self._navigating = True

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
