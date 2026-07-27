"""Initializer: maniobra de arranque open-loop. En cada GO recalibra el gyro (robot
quieto) y avanza recto para generar COG hasta que el heading converge; ahi deja de
publicar /init/cmd_vel."""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Float32

from buey_robot.utils.params import load_params
from buey_robot.contracts import NAV_START, ODOM, IMU_YAW, HEADING_FUSED, INIT_CMD_VEL, IMU_CALIBRATE

PARAMS = {
    'rate': ('rate_hz', float),
    'straight_speed': ('straight_speed_m_s', float),
    'min_distance': ('min_distance_m', float),
    'max_distance': ('max_distance_m', float),
    'calib_settle': ('calib_settle_s', float),
}

_IDLE = 'idle'
_CALIBRATING = 'calibrating'
_DRIVING_STRAIGHT = 'driving_straight'


class NavigationInitializer(Node):
    def __init__(self):
        super().__init__('navigation_initializer')
        load_params(self, PARAMS)

        # Estado
        self._state = _IDLE
        self._x = self._y = None
        self._start_xy = None
        self._converged = False
        self._gyro_ready = False
        self._calib_sent_at = 0.0

        # Salidas
        self._cmd_pub = self.create_publisher(Twist, INIT_CMD_VEL, 10)
        self._calib_pub = self.create_publisher(Empty, IMU_CALIBRATE, 10)

        # Entradas
        self.create_subscription(Empty, NAV_START, self._on_start, 10)
        self.create_subscription(Odometry, ODOM, self._on_odom, 10)
        self.create_subscription(Float32, IMU_YAW, self._on_yaw, 10)
        self.create_subscription(Float32, HEADING_FUSED, self._on_fused, 10)

        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f'navigation_initializer iniciado (recalib gyro + recto {self.straight_speed}m/s '
            f'hasta converger, min {self.min_distance}m / max {self.max_distance}m)')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_start(self, msg):
        # Cada GO reinicia la secuencia: recalibrar gyro + avanzar recto.
        self._state = _CALIBRATING
        self._converged = False
        self._gyro_ready = False
        self._start_xy = None
        self._calib_sent_at = self._now()
        self._calib_pub.publish(Empty())
        self.get_logger().info('GO -> recalibrando gyro (mantener el robot QUIETO)')

    def _on_odom(self, msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y

    def _on_yaw(self, msg):
        # /imu/yaw reaparece tras la gracia = gyro recalibrado (los previos al trigger se ignoran).
        if self._state == _CALIBRATING and self._now() - self._calib_sent_at >= self.calib_settle:
            self._gyro_ready = True

    def _on_fused(self, msg):
        self._converged = True

    def _tick(self):
        if self._state == _IDLE:
            return
        if self._state == _CALIBRATING:
            self._publish(0.0)                     # quieto mientras el gyro se recalibra
            if self._gyro_ready:
                self._start_xy = (self._x, self._y)
                self._state = _DRIVING_STRAIGHT
                self.get_logger().info(
                    f'gyro calibrado -> avanzando recto (min {self.min_distance:.1f}m, max {self.max_distance:.1f}m)')
            return
        traveled = self._traveled()
        if self._converged and traveled >= self.min_distance:
            self._state = _IDLE                    # listo: dejar de publicar /init/cmd_vel
            self.get_logger().info(f'heading listo ({traveled:.1f}m) -> cediendo a la navegacion')
            return
        if traveled >= self.max_distance:
            self._state = _IDLE
            self.get_logger().warn(
                f'tope {self.max_distance:.1f}m sin converger el heading -> cediendo igual (revisar COG/RTK)')
            return
        self._publish(self.straight_speed)

    def _traveled(self):
        if self._start_xy is None or self._x is None:
            return 0.0
        return math.hypot(self._x - self._start_xy[0], self._y - self._start_xy[1])

    def _publish(self, linear):
        t = Twist()
        t.linear.x = linear
        self._cmd_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationInitializer()
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
