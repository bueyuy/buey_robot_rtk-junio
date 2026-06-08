"""Joystick Controller: suaviza input de joystick MQTT con rampa y publica /cmd_vel_joy.

Pipeline:
    1. Recibe MQTT joystick (formato "w&v")
    2. Escala por cruise_linear / cruise_angular
    3. Aplica RampProfile a linear y angular
    4. Publica /cmd_vel_joy (Twist) a frequency_hz
    5. Sin input MQTT (timeout) -> rampea a 0 y deja de publicar
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.navigation.ramp_profile import RampProfile
from buey_robot.utils.config import load_config


class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')

        # --- Parametros de joystick (ROS2 estandar, sin defaults) ---
        self.declare_parameter('joystick.cruise_linear_m_s', Parameter.Type.DOUBLE)
        self.declare_parameter('joystick.cruise_angular_rad_s', Parameter.Type.DOUBLE)
        self.declare_parameter('joystick.max_angular_rad_s', Parameter.Type.DOUBLE)
        self.declare_parameter('joystick.frequency_hz', Parameter.Type.DOUBLE)
        self.declare_parameter('joystick.timeout_ms', Parameter.Type.DOUBLE)
        self.declare_parameter('joystick.ramp.accel_rate_linear', Parameter.Type.DOUBLE)
        self.declare_parameter('joystick.ramp.decel_rate_linear', Parameter.Type.DOUBLE)
        self.declare_parameter('joystick.ramp.accel_rate_angular', Parameter.Type.DOUBLE)
        self.declare_parameter('joystick.ramp.decel_rate_angular', Parameter.Type.DOUBLE)

        self.cruise_linear = self.get_parameter('joystick.cruise_linear_m_s').value
        self.cruise_angular = self.get_parameter('joystick.cruise_angular_rad_s').value
        self.max_angular = self.get_parameter('joystick.max_angular_rad_s').value
        self.frequency = self.get_parameter('joystick.frequency_hz').value
        self.joy_timeout = self.get_parameter('joystick.timeout_ms').value / 1000.0

        # Rampas independientes para linear y angular
        self.ramp_linear = RampProfile(
            accel_rate=self.get_parameter('joystick.ramp.accel_rate_linear').value,
            decel_rate=self.get_parameter('joystick.ramp.decel_rate_linear').value,
        )
        self.ramp_angular = RampProfile(
            accel_rate=self.get_parameter('joystick.ramp.accel_rate_angular').value,
            decel_rate=self.get_parameter('joystick.ramp.decel_rate_angular').value,
        )

        # Cliente MQTT: mqtt.yaml se sigue leyendo con load_config (no es param ROS2)
        mqtt_cfg = load_config('mqtt.yaml')
        self.topic_joystick = mqtt_cfg['topics']['joystick']

        # Cliente MQTT compartido (singleton)
        self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())

        # Estado
        self._raw_w = 0.0
        self._raw_v = 0.0
        self._last_mqtt_stamp = 0.0

        # Suscripcion MQTT: joystick
        self._mqtt.subscribe(self.topic_joystick, self._joystick_mqtt_callback)

        # Publisher ROS2
        self.cmd_vel_joy_pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)

        # Timer de salida
        self.create_timer(1.0 / self.frequency, self._output_loop)

        self.get_logger().info('Joystick Controller iniciado')
        self.get_logger().info(
            f'  cruise: linear={self.cruise_linear} m/s, angular={self.cruise_angular} rad/s')

    def _joystick_mqtt_callback(self, client, userdata, msg):
        """Recibe joystick por MQTT. Formato: "w&v"."""
        try:
            payload = msg.payload.decode()
            parts = payload.split('&')
            self._raw_w = float(parts[0])
            self._raw_v = float(parts[1]) if len(parts) > 1 else 0.0
            self._last_mqtt_stamp = time.time()
        except Exception as e:
            self.get_logger().warn(f'Joystick parse error: {e}')

    def _output_loop(self):
        """Timer a frequency_hz: escala, rampea, publica solo si hay actividad."""
        now = time.time()
        mqtt_active = (now - self._last_mqtt_stamp) < self.joy_timeout

        if mqtt_active and (abs(self._raw_v) > 0.01 or abs(self._raw_w) > 0.01):
            target_v = self._raw_v * self.cruise_linear
            target_w = self._raw_w * self.cruise_angular
            target_w = max(-self.max_angular, min(self.max_angular, target_w))
        else:
            target_v = 0.0
            target_w = 0.0

        v = self.ramp_linear.apply(target_v)
        w = self.ramp_angular.apply(target_w)

        if abs(v) < 1e-4 and abs(w) < 1e-4:
            if not mqtt_active:
                return
            if abs(self._raw_v) <= 0.01 and abs(self._raw_w) <= 0.01:
                return

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.cmd_vel_joy_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()

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
