#!/usr/bin/env python3
"""Monitor de cobertura del heading durante un giro 360.

Compara /imu/heading_calibrated (nodo imu_compass) contra /imu/heading (crudo
del firmware). Reporta cuanto barre cada uno de los 360 grados.
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

CSV = '/home/nexlab/imu_cal/heading_monitor.csv'


class Mon(Node):
    def __init__(self):
        super().__init__('heading_monitor')
        self.cal = None
        self.raw = None
        self.cal_hist = [0] * 36
        self.raw_hist = [0] * 36
        self.cal_vals = []
        self.f = open(CSV, 'w')
        self.f.write('t,calibrado,crudo\n')
        self._t0 = None
        self.create_subscription(Float32, '/imu/heading_calibrated', self._cal, 10)
        self.create_subscription(Float32, '/imu/heading', self._raw, 10)
        self.create_timer(2.0, self._report)
        print('[mon] activo -> calibrado vs crudo', flush=True)

    def _now(self):
        s = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = s
        return s - self._t0

    def _cal(self, m):
        self.cal = m.data % 360
        self.cal_hist[int(self.cal // 10)] += 1
        self.cal_vals.append(self.cal)
        self.f.write(f'{self._now():.2f},{self.cal:.2f},'
                     f'{"" if self.raw is None else f"{self.raw:.2f}"}\n')
        self.f.flush()

    def _raw(self, m):
        self.raw = m.data % 360
        self.raw_hist[int(self.raw // 10)] += 1

    def _report(self):
        cal_cov = sum(1 for h in self.cal_hist if h > 0) / 36 * 100
        raw_cov = sum(1 for h in self.raw_hist if h > 0) / 36 * 100
        c = '--' if self.cal is None else f'{self.cal:6.1f}'
        r = '--' if self.raw is None else f'{self.raw:6.1f}'
        print(f'[t={self._now():5.1f}] calibrado={c} (cobertura {cal_cov:3.0f}%) | '
              f'crudo={r} (cobertura {raw_cov:3.0f}%)', flush=True)


def main():
    rclpy.init()
    n = Mon()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.f.close()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
