"""Joystick: recibe teleop por MQTT (formato "w&v"), lo escala y publica /joy/cmd_vel
CRUDO (el suavizado lo hace el gateway con su perfil joy). Sin input (timeout) deja de
publicar -> el gateway cede a la navegacion."""

import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.utils.config import load_config
from buey_robot.contracts import JOY_CMD_VEL


class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')

        self.declare_parameter('joystick.cruise_linear_m_s', Parameter.Type.DOUBLE)
        self.declare_parameter('joystick.cruise_angular_rad_s', Parameter.Type.DOUBLE)
        self.declare_parameter('joystick.max_angular_rad_s', Parameter.Type.DOUBLE)
        self.declare_parameter('joystick.frequency_hz', Parameter.Type.DOUBLE)
        self.declare_parameter('joystick.timeout_ms', Parameter.Type.DOUBLE)
        self.cruise_linear = self.get_parameter('joystick.cruise_linear_m_s').value
        self.cruise_angular = self.get_parameter('joystick.cruise_angular_rad_s').value
        self.max_angular = self.get_parameter('joystick.max_angular_rad_s').value
        self.frequency = self.get_parameter('joystick.frequency_hz').value
        self.joy_timeout = self.get_parameter('joystick.timeout_ms').value / 1000.0

        mqtt_cfg = load_config('mqtt.yaml')
        self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())
        self._raw_w = 0.0
        self._raw_v = 0.0
        self._last_stamp = 0.0
        self._mqtt.subscribe(mqtt_cfg['topics']['joystick'], self._on_joystick)
        self._pub = self.create_publisher(Twist, JOY_CMD_VEL, 10)
        self.create_timer(1.0 / self.frequency, self._output_loop)

    def _on_joystick(self, client, userdata, msg):
        try:
            parts = msg.payload.decode().split('&')
            self._raw_w = float(parts[0])
            self._raw_v = float(parts[1]) if len(parts) > 1 else 0.0
            self._last_stamp = time.time()
        except Exception:
            pass

    def _output_loop(self):
        if (time.time() - self._last_stamp) >= self.joy_timeout:
            return                                  # sin input: deja de publicar -> el gateway cede a nav
        t = Twist()
        t.linear.x = self._raw_v * self.cruise_linear
        t.angular.z = max(-self.max_angular, min(self.max_angular, self._raw_w * self.cruise_angular))
        self._pub.publish(t)


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
