"""Driver IMU MPU6050: bias del gyro (auto-calibrado en reposo) + integra yaw rate -> /imu/yaw."""

import json
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String, Empty

from buey_robot.contracts import IMU_YAW, IMU_RATE, IMU_STATUS, IMU_CALIBRATE
from buey_robot.utils.params import load_params

PARAMS = {
    'imu_topic': str,
    'bias_z': ('gyro.bias_z', float),
    'auto_samples': ('gyro.auto_bias_samples', int),
    'stationary_thresh': ('gyro.stationary_thresh', float),
    'gyro_alive_min_std': ('gyro.alive_min_std', float),
    'heading_initial': ('heading.initial_deg', float),
    'heading_invert': ('heading.invert', bool),
}


class ImuMpu6050(Node):
    def __init__(self):
        super().__init__('imu_mpu6050')

        load_params(self, PARAMS)

        # Estado
        self._heading_deg = self.heading_initial
        self._yaw_rate = 0.0
        self._last_time = None
        self._calibrating = True            # siempre auto-calibra al arrancar
        self._acc_z = 0.0                   # suma wz -> media del bias de yaw
        self._acc_sq_z = 0.0                # suma wz^2 -> std del yaw rate (gyro muerto)
        self._n = 0

        # Salidas
        self._yaw_pub = self.create_publisher(Float32, IMU_YAW, 10)
        self._rate_pub = self.create_publisher(Float32, IMU_RATE, 10)
        self._status_pub = self.create_publisher(String, IMU_STATUS, 10)

        # Entradas
        self.create_subscription(Imu, self.imu_topic, self._on_imu, 10)
        self.create_subscription(Empty, IMU_CALIBRATE, self._on_calibrate, 10)

        self.get_logger().info(
            f'imu_mpu6050: {self.imu_topic} -> {IMU_YAW} (+ {IMU_RATE}); '
            f'auto-bias ON ({self.auto_samples} muestras, robot QUIETO)')

    def _on_imu(self, msg: Imu):
        wx, wy, wz = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)

        if self._calibrating:
            self._collect_bias(wx, wy, wz)
            return

        cwz = wz - self.bias_z
        self._yaw_rate = cwz
        self._rate_pub.publish(Float32(data=float(cwz)))

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_time is None:
            self._last_time = now
        else:
            dt = now - self._last_time
            self._last_time = now
            if 0.0 < dt < 1.0:   # descartar dt no positivos o saltos (reconexion del agente)
                dpsi = -cwz * dt if self.heading_invert else cwz * dt
                self._heading_deg = (self._heading_deg + math.degrees(dpsi)) % 360.0

        self._yaw_pub.publish(Float32(data=float(self._heading_deg)))
        self._publish_status()

    def _publish_status(self):
        self._status_pub.publish(String(data=json.dumps({
            'calibrated': not self._calibrating,
            'bias_z': round(self.bias_z, 6),
            'yaw_rate': round(self._yaw_rate, 5),
        })))

    def _on_calibrate(self, msg: Empty):
        """Reinicia el auto-bias (deja de publicar /imu/yaw hasta recalibrar)."""
        self.get_logger().info('Recalibracion solicitada — reiniciando auto-bias')
        self._calibrating = True
        self._acc_z = 0.0
        self._acc_sq_z = 0.0
        self._n = 0
        self._last_time = None
        self._heading_deg = self.heading_initial

    def _collect_bias(self, wx, wy, wz):
        """Promedia el gyro quieto para el bias, y valida que este VIVO: un topic muerto da
        wz==0.0 exacto (std~0) -> no completa -> no publica /imu/yaw (mejor bloquear que
        navegar con heading muerto)."""
        if math.sqrt(wx * wx + wy * wy + wz * wz) > self.stationary_thresh:
            if self._n > 0:
                self.get_logger().warn(
                    'movimiento durante auto-bias: reiniciando (mantener quieto)',
                    throttle_duration_sec=2.0)
            self._acc_z = 0.0
            self._acc_sq_z = 0.0
            self._n = 0
            return

        self._acc_z += wz
        self._acc_sq_z += wz * wz
        self._n += 1

        if self._n >= self.auto_samples:
            mean_z = self._acc_z / self._n
            std_z = math.sqrt(max(0.0, self._acc_sq_z / self._n - mean_z * mean_z))
            if std_z < self.gyro_alive_min_std:
                self.get_logger().error(
                    f'GYRO SIN SEÑAL: std yaw rate={std_z:.2e} < {self.gyro_alive_min_std:.2e} '
                    f'rad/s. Revisar {self.imu_topic} (firmware / topic / cableado). '
                    f'NO se habilita el heading — robot bloqueado.',
                    throttle_duration_sec=5.0)
                self._acc_z = 0.0
                self._acc_sq_z = 0.0
                self._n = 0
                return
            self.bias_z = mean_z
            self._calibrating = False
            self._last_time = None
            self.get_logger().info(
                f'auto-bias OK (n={self._n}, std_z={std_z:.2e} rad/s): bias_z={self.bias_z:.5f} rad/s')


def main(args=None):
    rclpy.init(args=args)
    node = ImuMpu6050()
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
