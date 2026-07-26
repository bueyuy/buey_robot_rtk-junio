"""Fusion de heading: IMU (/imu/heading, los giros) + /gps/heading (referencia
absoluta, ya validada por el GPS) -> heading absoluto sin drift = imu_heading + offset.
El offset (= gps_heading - imu_heading, suavizado) se corrige solo yendo derecho
(yaw rate bajo del IMU); el GPS ya publica /gps/heading solo cuando es confiable.

Publica  /heading/fused (deg ENU), /heading/fused_ready (Bool, retiene ultimo valor), /fusion/status.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Bool, String

from buey_robot.contracts import (
    IMU_HEADING, IMU_RATE, GPS_HEADING, HEADING_FUSED, HEADING_FUSED_READY, FUSION_STATUS)
from buey_robot.utils.params import load_params
from buey_robot.utils.math import wrap180

PARAMS = {
    'offset_alpha': float,
    'init_samples': ('offset_init_samples', int),
    'straight_max_yaw_rate': float,
    'converge_tol_deg': float,
    'converge_min_samples': int,
}


class HeadingFusion(Node):
    def __init__(self):
        super().__init__('heading_fusion')

        load_params(self, PARAMS)

        # Estado
        self._imu_heading = None     # deg, ultimo /imu/heading
        self._yaw_rate = 0.0         # rad/s, ultimo /imu/rate (gate de recta)
        self._offset = None          # deg, imu -> ENU absoluto. None hasta el warm-up
        self._gps_heading = None     # deg ENU, ultimo /gps/heading recibido
        self._last_residual = 0.0
        self._correcting = False
        self._init_sin = 0.0         # media circular del warm-up
        self._init_cos = 0.0
        self._init_n = 0
        self._converge_count = 0
        self._ready = False

        retained = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._fused_pub = self.create_publisher(Float32, HEADING_FUSED, 10)
        self._ready_pub = self.create_publisher(Bool, HEADING_FUSED_READY, retained)
        self._status_pub = self.create_publisher(String, FUSION_STATUS, 10)
        self._ready_pub.publish(Bool(data=False))

        self.create_subscription(Float32, IMU_HEADING, self._on_heading, 10)
        self.create_subscription(Float32, IMU_RATE, self._on_rate, 10)
        self.create_subscription(Float32, GPS_HEADING, self._on_gps_heading, 10)

        self.get_logger().info(
            f'heading_fusion: {IMU_HEADING} + {GPS_HEADING} -> {HEADING_FUSED} '
            f'(alpha={self.offset_alpha})')

    def _on_rate(self, msg: Float32):
        self._yaw_rate = msg.data

    def _on_heading(self, msg: Float32):
        """El heading fusionado sigue al IMU (suave); el GPS solo corrige el offset."""
        self._imu_heading = msg.data
        if self._offset is not None:
            self._fused_pub.publish(Float32(data=float((self._imu_heading + self._offset) % 360.0)))
        self._publish_status()

    def _on_gps_heading(self, msg: Float32):
        """Corrige el offset con la referencia absoluta del GPS, solo yendo derecho
        (en giros el COG laggea al gyro). El GPS ya garantiza que el valor es confiable."""
        self._correcting = False
        if self._imu_heading is None:
            return
        if abs(self._yaw_rate) > self.straight_max_yaw_rate:
            return

        self._correcting = True
        self._gps_heading = msg.data
        target = wrap180(self._gps_heading - self._imu_heading)

        if self._offset is None:
            # Warm-up: promediar (media circular) init_samples muestras antes de fijar
            # el offset (la primera muestra al arrancar es transitoria y deja un snap corrido).
            self._init_sin += math.sin(math.radians(target))
            self._init_cos += math.cos(math.radians(target))
            self._init_n += 1
            if self._init_n >= self.init_samples:
                self._offset = math.degrees(math.atan2(self._init_sin, self._init_cos))
                self.get_logger().info(
                    f'offset inicial={self._offset:.1f} deg (media de {self._init_n} muestras)')
        else:
            self._last_residual = wrap180(target - self._offset)
            self._offset = wrap180(self._offset + self.offset_alpha * self._last_residual)
            self._track_convergence(self._last_residual)

    def _track_convergence(self, residual):
        """Publica /heading/fused_ready=true (una sola vez) tras N muestras rectas con
        residual chico -> el controller lo espera antes de soltar la navegacion."""
        self._converge_count = self._converge_count + 1 if abs(residual) <= self.converge_tol_deg else 0
        if not self._ready and self._converge_count >= self.converge_min_samples:
            self._ready = True
            self._ready_pub.publish(Bool(data=True))
            self.get_logger().info(
                f'heading fused CONVERGIDO (|residual|<{self.converge_tol_deg:.0f} deg '
                f'x{self.converge_min_samples} muestras) -> {HEADING_FUSED_READY}=true')

    def _publish_status(self):
        self._status_pub.publish(String(data=json.dumps({
            'imu_heading': round(self._imu_heading, 1) if self._imu_heading is not None else None,
            'gps_heading': round(self._gps_heading, 1) if self._gps_heading is not None else None,
            'offset': round(self._offset, 1) if self._offset is not None else None,
            'residual': round(self._last_residual, 1),
            'correcting': self._correcting,
            'converged': self._ready,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = HeadingFusion()
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
