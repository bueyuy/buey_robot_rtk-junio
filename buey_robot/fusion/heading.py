"""Fusion de heading: /imu/yaw (gyro, suave) + /gps/course (referencia absoluta) ->
/heading/fused (deg ENU) = imu_yaw + offset. Publica SOLO cuando el offset convergio."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from buey_robot.contracts import IMU_YAW, IMU_RATE, GPS_COURSE, HEADING_FUSED
from buey_robot.utils.params import load_params
from buey_robot.utils.math import wrap180

PARAMS = {
    'offset_alpha': float,
    'init_samples': ('offset_init_samples', int),
    'straight_max_yaw_rate': float,
    'converge_tol_deg': float,
    'converge_min_samples': int,
}


class FusionHeading(Node):
    def __init__(self):
        super().__init__('fusion_heading')
        load_params(self, PARAMS)
        self._imu_yaw = None         # deg, ultimo /imu/yaw
        self._yaw_rate = 0.0         # rad/s, ultimo /imu/rate (gate de recta)
        self._offset = None          # deg, imu -> ENU absoluto; None hasta el warm-up
        self._init_sin = 0.0         # media circular del warm-up
        self._init_cos = 0.0
        self._init_n = 0
        self._converge_count = 0
        self._converged = False
        self._fused_pub = self.create_publisher(Float32, HEADING_FUSED, 10)
        self.create_subscription(Float32, IMU_YAW, self._on_yaw, 10)
        self.create_subscription(Float32, IMU_RATE, self._on_rate, 10)
        self.create_subscription(Float32, GPS_COURSE, self._on_course, 10)
        self.get_logger().info(
            f'fusion_heading iniciado: gyro+COG -> /heading/fused solo al converger '
            f'(alpha={self.offset_alpha}, tol={self.converge_tol_deg}deg x{self.converge_min_samples})')

    def _on_rate(self, msg):
        self._yaw_rate = msg.data

    def _on_yaw(self, msg):
        # El heading sigue al IMU (suave); el GPS solo corrige el offset. Publica recien al converger.
        self._imu_yaw = msg.data
        if self._converged:
            self._fused_pub.publish(Float32(data=float((self._imu_yaw + self._offset) % 360.0)))

    def _on_course(self, msg):
        # Corrige el offset con la referencia absoluta del GPS solo yendo derecho
        # (en giros el COG laggea al gyro). El GPS ya garantiza que el valor es confiable.
        if self._imu_yaw is None or abs(self._yaw_rate) > self.straight_max_yaw_rate:
            return
        target = wrap180(msg.data - self._imu_yaw)
        if self._offset is None:
            self._init_sin += math.sin(math.radians(target))
            self._init_cos += math.cos(math.radians(target))
            self._init_n += 1
            if self._init_n >= self.init_samples:
                self._offset = math.degrees(math.atan2(self._init_sin, self._init_cos))
                self.get_logger().info(f'offset inicial fijado: {self._offset:.1f} deg (gyro->ENU)')
            return
        residual = wrap180(target - self._offset)
        self._offset = wrap180(self._offset + self.offset_alpha * residual)
        self._converge_count = self._converge_count + 1 if abs(residual) <= self.converge_tol_deg else 0
        if not self._converged and self._converge_count >= self.converge_min_samples:
            self._converged = True
            self.get_logger().info(
                f'heading convergido (|residual|<{self.converge_tol_deg:.0f}deg x{self.converge_min_samples}) '
                f'-> publicando /heading/fused')


def main(args=None):
    rclpy.init(args=args)
    node = FusionHeading()
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
