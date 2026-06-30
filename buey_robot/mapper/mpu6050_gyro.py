"""mpu6050_gyro: calibra el giroscopio del MPU6050 e integra un heading de yaw.

El firmware ESP32+MPU6050 (Buey_ESP32_MPU6050_microROS_fw) publica sensor_msgs/Imu
CRUDO en /imu/data (accel + gyro, sin magnetometro). Este nodo:
  - resta el bias del giroscopio (de config o auto-estimado en reposo)
  - integra angular_velocity.z (yaw rate, rad/s) -> heading relativo en grados
  - republica el Imu corregido (/imu/data_calibrated) y el heading
    (/heading/gyro, junto a /heading/gps y /heading/imu)

IMPORTANTE: el heading de gyro DERIVA. No tiene referencia absoluta como la
brujula (mag, LSM303) o el COG del GPS: es suave y preciso a corto plazo pero
acumula error sin cota. Aca se expone para telemetria, para comparar contra
/imu/heading_calibrated (mag) y el heading GPS, y decidir cual fusionar.

Los topics de entrada/salida y la config (config/imu.yaml, bloque mpu6050_gyro)
son configurables. LSM303 y MPU6050 usan firmwares distintos sobre la misma
ESP32, asi que nunca publican /imu/data a la vez (no se reflashea nada).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


class Mpu6050Gyro(Node):
    def __init__(self):
        super().__init__('mpu6050_gyro')

        # --- Parametros (sin defaults: fail-fast si falta en el YAML) ---
        self.declare_parameter('imu_topic', Parameter.Type.STRING)
        self.declare_parameter('data_calibrated_topic', Parameter.Type.STRING)
        self.declare_parameter('heading_topic', Parameter.Type.STRING)
        # gyro_bias: [x, y, z] en rad/s, restado a angular_velocity
        self.declare_parameter('gyro_bias', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('auto_bias_enabled', Parameter.Type.BOOL)
        self.declare_parameter('auto_bias_samples', Parameter.Type.INTEGER)
        self.declare_parameter('stationary_thresh', Parameter.Type.DOUBLE)
        self.declare_parameter('heading_initial_deg', Parameter.Type.DOUBLE)
        self.declare_parameter('heading_invert', Parameter.Type.BOOL)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.data_cal_topic = self.get_parameter('data_calibrated_topic').value
        self.heading_topic = self.get_parameter('heading_topic').value
        bias = self.get_parameter('gyro_bias').value
        if len(bias) != 3:
            raise ValueError(f'gyro_bias debe tener 3 elementos (tiene {len(bias)})')
        self.bias = list(bias)  # [bx, by, bz] rad/s
        self.auto_bias = self.get_parameter('auto_bias_enabled').value
        self.auto_samples = self.get_parameter('auto_bias_samples').value
        self.stationary_thresh = self.get_parameter('stationary_thresh').value
        self.heading_deg = self.get_parameter('heading_initial_deg').value
        self.heading_invert = self.get_parameter('heading_invert').value

        # Estado de integracion
        self._last_time = None  # seg (reloj del nodo)

        # Estado de auto-calibracion de bias en reposo
        self._calibrating = self.auto_bias
        self._acc = [0.0, 0.0, 0.0]
        self._n = 0

        self.heading_pub = self.create_publisher(Float32, self.heading_topic, 10)
        self.data_pub = self.create_publisher(Imu, self.data_cal_topic, 10)
        self.create_subscription(Imu, self.imu_topic, self._imu_callback, 10)

        self.get_logger().info(
            f'mpu6050_gyro: {self.imu_topic} -> {self.heading_topic} (+ {self.data_cal_topic})')
        if self._calibrating:
            self.get_logger().info(
                f'  auto-bias: ON (mantener el robot QUIETO, {self.auto_samples} muestras)')
        else:
            self.get_logger().info(f'  auto-bias: OFF, usando gyro_bias={self.bias}')

    def _imu_callback(self, msg: Imu):
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        if self._calibrating:
            self._collect_bias(wx, wy, wz)
            return

        # Restar bias
        cwz = wz - self.bias[2]

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_time is None:
            self._last_time = now
        else:
            dt = now - self._last_time
            self._last_time = now
            # Guardas: descartar dt no positivos o saltos grandes (reconexion del agente)
            if 0.0 < dt < 1.0:
                dpsi = cwz * dt  # rad
                if self.heading_invert:
                    dpsi = -dpsi
                self.heading_deg = (self.heading_deg + math.degrees(dpsi)) % 360.0

        self.heading_pub.publish(Float32(data=float(self.heading_deg)))
        self._publish_calibrated(msg)

    def _collect_bias(self, wx, wy, wz):
        """Promedia el gyro con el robot quieto para estimar el bias."""
        mag = math.sqrt(wx * wx + wy * wy + wz * wz)
        if mag > self.stationary_thresh:
            # Se movio: descartar lo acumulado y reintentar desde cero
            if self._n > 0:
                self.get_logger().warn(
                    'movimiento durante auto-bias: reiniciando captura (mantener quieto)',
                    throttle_duration_sec=2.0)
            self._acc = [0.0, 0.0, 0.0]
            self._n = 0
            return

        self._acc[0] += wx
        self._acc[1] += wy
        self._acc[2] += wz
        self._n += 1

        if self._n >= self.auto_samples:
            self.bias = [a / self._n for a in self._acc]
            self._calibrating = False
            self._last_time = None
            self.get_logger().info(
                f'auto-bias OK (n={self._n}): gyro_bias='
                f'[{self.bias[0]:.5f}, {self.bias[1]:.5f}, {self.bias[2]:.5f}] rad/s')

    def _publish_calibrated(self, msg: Imu):
        """Republica el Imu con el gyro des-sesgado (accel y orientation intactos)."""
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.angular_velocity.x = msg.angular_velocity.x - self.bias[0]
        out.angular_velocity.y = msg.angular_velocity.y - self.bias[1]
        out.angular_velocity.z = msg.angular_velocity.z - self.bias[2]
        self.data_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Mpu6050Gyro()
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
