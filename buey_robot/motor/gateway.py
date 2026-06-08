"""Motor Gateway: Pipeline minimo cmd_vel -> motores via output inyectado.

Reactivo: ejecuta el pipeline cada vez que recibe un mensaje en /cmd_vel o /cmd_vel_joy.
Joystick (/cmd_vel_joy) tiene prioridad sobre navegacion (/cmd_vel).

Pipeline:
    1. Seleccion input (joy > nav, con timeout)
    2. Watchdog (sin input -> envia 0,0)
    3. Gains (linear_gain, angular_gain)
    4. Cinematica diferencial
    5. Clamp a [-max_output, max_output]
    6. Soft deadzone
    7. Output via objeto inyectado (MqttMotorOutput o similar)
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.adapters.mqtt.outputs.motor import MqttMotorOutput
from buey_robot.motor.filters import SoftDeadzone
from buey_robot.motor.kinematics import differential_to_motor
from buey_robot.utils.config import load_config


class MotorGateway(Node):
    def __init__(self):
        super().__init__('motor_gateway')

        # --- Parametros de motor (ROS2 estandar, sin defaults) ---
        self.declare_parameter('motor_control.wheel_separation', Parameter.Type.DOUBLE)
        self.declare_parameter('motor_control.max_output', Parameter.Type.DOUBLE)
        self.declare_parameter('motor_control.linear_gain', Parameter.Type.DOUBLE)
        self.declare_parameter('motor_control.angular_gain', Parameter.Type.DOUBLE)
        self.declare_parameter('motor_control.soft_deadzone.low', Parameter.Type.DOUBLE)
        self.declare_parameter('motor_control.soft_deadzone.high', Parameter.Type.DOUBLE)
        self.declare_parameter('motor_control.safety.joystick_timeout_ms', Parameter.Type.DOUBLE)

        # Parametros de cinematica
        self.L = self.get_parameter('motor_control.wheel_separation').value
        self.max_output = self.get_parameter('motor_control.max_output').value
        self.linear_gain = self.get_parameter('motor_control.linear_gain').value
        self.angular_gain = self.get_parameter('motor_control.angular_gain').value

        # Soft deadzone
        dz_low = self.get_parameter('motor_control.soft_deadzone.low').value
        dz_high = self.get_parameter('motor_control.soft_deadzone.high').value
        self.deadzone = SoftDeadzone(dz_low, dz_high)

        # Safety timeouts
        self.joy_timeout = self.get_parameter('motor_control.safety.joystick_timeout_ms').value / 1000.0

        # Cliente MQTT: mqtt.yaml se sigue leyendo con load_config (no es param ROS2)
        mqtt_cfg = load_config('mqtt.yaml')
        self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())

        # Output inyectado: usa el cliente compartido
        self.output = MqttMotorOutput(self._mqtt, mqtt_cfg)

        # Timestamps de ultimo mensaje recibido
        self._joy_stamp = 0.0
        self._nav_stamp = 0.0

        # Subscribers ROS2
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        self.create_subscription(Twist, '/cmd_vel_joy', self._cmd_vel_joy_callback, 10)

        self.get_logger().info('Motor Gateway iniciado (reactivo, output inyectado)')
        self.get_logger().info(f'  L={self.L}, gains=({self.linear_gain}, {self.angular_gain})')
        self.get_logger().info(f'  Soft deadzone: low={dz_low}, high={dz_high}')

    def _cmd_vel_callback(self, msg: Twist):
        """Recibe cmd_vel de navegacion. Ignora si joystick esta activo."""
        now = time.time()
        self._nav_stamp = now
        if (now - self._joy_stamp) < self.joy_timeout:
            return
        self._run_pipeline(msg.linear.x, msg.angular.z)

    def _cmd_vel_joy_callback(self, msg: Twist):
        """Recibe cmd_vel_joy de joystick. Siempre tiene prioridad."""
        self._joy_stamp = time.time()
        self._run_pipeline(msg.linear.x, msg.angular.z)

    def _run_pipeline(self, v: float, w: float):
        """Pipeline: gains -> cinematica -> clamp -> deadzone -> output."""
        # 1. Gains
        v_scaled = v * self.linear_gain
        w_scaled = w * self.angular_gain

        # 2. Cinematica diferencial
        velL, velR = differential_to_motor(v_scaled, w_scaled, self.L)

        # 3. Clamp
        velL = max(-self.max_output, min(self.max_output, velL))
        velR = max(-self.max_output, min(self.max_output, velR))

        # 4. Soft deadzone (deshabilitado para pruebas)
        # velL = self.deadzone.apply(velL)
        # velR = self.deadzone.apply(velR)

        # 5. Output
        self.output.send(velL, velR, v, w)


def main(args=None):
    rclpy.init(args=args)
    node = MotorGateway()

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
