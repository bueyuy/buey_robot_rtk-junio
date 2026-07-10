#!/usr/bin/env python3
"""Logger de validacion IMU (captura mag + headings + odom a CSV)."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float32

CSV = '/home/nexlab/imu_cal/imu_cal_log.csv'
V_MIN = 0.3


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class Logger(Node):
    def __init__(self):
        super().__init__('imu_cal_logger')
        self.h_imu = None
        self.h_gps = None
        self.mag_x = []
        self.mag_y = []
        self.x = self.y = None
        self.t_prev = None
        self.v = 0.0
        self.sin_sum = 0.0
        self.cos_sum = 0.0
        self.n_delta = 0
        self.n_glitch = 0

        self.f = open(CSV, 'w')
        self.f.write('t,x,y,v,heading_imu,heading_gps,delta_wrap,mag_x,mag_y,mag_z\n')

        self.create_subscription(Float64, '/heading/imu', self._h_imu, 10)
        self.create_subscription(Float64, '/heading/gps', self._h_gps, 10)
        self.create_subscription(MagneticField, '/imu/mag', self._mag, 10)
        self.create_subscription(Odometry, '/odom_filtered', self._odom, 10)
        self.create_timer(2.0, self._report)
        self._t0 = None
        print('[logger] activo -> ' + CSV, flush=True)

    def _now(self):
        s = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = s
        return s - self._t0

    def _h_imu(self, m):
        self.h_imu = m.data

    def _h_gps(self, m):
        self.h_gps = m.data

    def _odom(self, m):
        t = self._now()
        x = m.pose.pose.position.x
        y = m.pose.pose.position.y
        if self.x is not None and self.t_prev is not None:
            dt = t - self.t_prev
            if dt > 1e-3:
                self.v = math.hypot(x - self.x, y - self.y) / dt
        self.x, self.y, self.t_prev = x, y, t

    def _mag(self, m):
        t = self._now()
        mx = m.magnetic_field.x
        my = m.magnetic_field.y
        mz = m.magnetic_field.z
        glitch = abs(mx) > 1500 or abs(my) > 1500 or abs(mz) > 1500
        if not glitch:
            self.mag_x.append(mx)
            self.mag_y.append(my)
        else:
            self.n_glitch += 1
        delta = ''
        if self.h_imu is not None and self.h_gps is not None:
            d = wrap180(self.h_gps - self.h_imu)
            delta = f'{d:.2f}'
            if self.v >= V_MIN:
                r = math.radians(d)
                self.sin_sum += math.sin(r)
                self.cos_sum += math.cos(r)
                self.n_delta += 1
        hi = '' if self.h_imu is None else f'{self.h_imu:.2f}'
        hg = '' if self.h_gps is None else f'{self.h_gps:.2f}'
        self.f.write(f'{t:.2f},{self.x},{self.y},{self.v:.3f},'
                     f'{hi},{hg},{delta},{mx:.4f},{my:.4f},{mz:.4f}\n')
        self.f.flush()

    def _report(self):
        line = f'[t={self._now():6.1f}] v={self.v:.2f} '
        if self.h_imu is not None:
            line += f'h_imu={self.h_imu:6.1f} '
        if self.h_gps is not None:
            line += f'h_gps={self.h_gps:6.1f} '
        if self.h_imu is not None and self.h_gps is not None:
            line += f'delta={wrap180(self.h_gps - self.h_imu):6.1f} '
        if len(self.mag_x) > 5:
            cx = 0.5 * (max(self.mag_x) + min(self.mag_x))
            cy = 0.5 * (max(self.mag_y) + min(self.mag_y))
            rx = 0.5 * (max(self.mag_x) - min(self.mag_x))
            ry = 0.5 * (max(self.mag_y) - min(self.mag_y))
            line += (f'| mag n={len(self.mag_x)} centro=({cx:.1f},{cy:.1f}) '
                     f'radio=({rx:.1f},{ry:.1f}) glitch={self.n_glitch}')
        if self.n_delta > 0:
            mean = math.degrees(math.atan2(self.sin_sum, self.cos_sum))
            line += f' | DELTA_circ(mov)={mean:.2f} (n={self.n_delta})'
        print(line, flush=True)


def main():
    rclpy.init()
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
