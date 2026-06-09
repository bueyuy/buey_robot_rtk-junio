#!/usr/bin/env python3
"""Monitor de navegacion: odom (yaw fusionado), headings, cmd_vel, velocidad."""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


def yaw_from_quat(q):
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(s, c))


class NavMon(Node):
    def __init__(self):
        super().__init__('nav_monitor')
        self.x = self.y = None
        self.fused = None
        self.v_odom = 0.0
        self.h_imu = None
        self.h_gps = None
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.odom_count = 0
        self.create_subscription(Odometry, '/odom_filtered', self._odom, 10)
        self.create_subscription(Float64, '/heading/imu', self._himu, 10)
        self.create_subscription(Float64, '/heading/gps', self._hgps, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd, 10)
        self.create_timer(1.0, self._report)
        self._t0 = None
        print('[nav_mon] activo', flush=True)

    def _now(self):
        s = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = s
        return s - self._t0

    def _odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.fused = yaw_from_quat(m.pose.pose.orientation)
        self.v_odom = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        self.odom_count += 1

    def _himu(self, m):
        self.h_imu = m.data

    def _hgps(self, m):
        self.h_gps = m.data

    def _cmd(self, m):
        self.cmd_v = m.linear.x
        self.cmd_w = m.angular.z

    def _report(self):
        if self.x is None:
            print(f'[t={self._now():5.1f}] SIN /odom_filtered todavia '
                  f'(falta BASE para fijar origen) | '
                  f'h_imu={self.h_imu} h_gps={self.h_gps}', flush=True)
            return
        hi = '--' if self.h_imu is None else f'{self.h_imu:6.1f}'
        hg = '--' if self.h_gps is None else f'{self.h_gps:6.1f}'
        print(f'[t={self._now():5.1f}] pos=({self.x:6.2f},{self.y:6.2f}) '
              f'yaw_fus={self.fused:6.1f} | h_imu={hi} h_gps={hg} | '
              f'cmd v={self.cmd_v:+.2f} w={self.cmd_w:+.2f} | v_odom={self.v_odom:.2f} '
              f'[odom#{self.odom_count}]', flush=True)


def main():
    rclpy.init()
    n = NavMon()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
