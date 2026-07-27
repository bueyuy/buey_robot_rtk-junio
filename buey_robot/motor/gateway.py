"""Motor Gateway: mux por prioridad (joy>init>nav) + rampa por fuente + cinematica
diferencial + clamp de rueda -> motores (via output inyectado). Corre a rate fijo:
toma la fuente activa mas prioritaria, rampa hacia su velocidad objetivo con el perfil
de esa fuente, y baja a ruedas."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.adapters.mqtt.motor_output import MqttMotorOutput
from buey_robot.motor.filters import SlewRateLimiter
from buey_robot.motor.kinematics import differential_to_motor
from buey_robot.utils.config import load_config, require_key
from buey_robot.utils.params import load_params
from buey_robot.utils.log import TransitionLogger
from buey_robot.contracts import NAV_CMD_VEL, JOY_CMD_VEL, INIT_CMD_VEL

PARAMS = {
    'L': ('wheel_separation', float),
    'max_output': ('max_output', float),
    'linear_gain': ('linear_gain', float),
    'angular_gain': ('angular_gain', float),
    'rate': ('velocity.rate_hz', float),
    'source_timeout_ms': ('velocity.source_timeout_ms', float),
    'brake_lin': ('velocity.brake.linear', float),
    'brake_ang': ('velocity.brake.angular', float),
    'nav_al': ('velocity.nav.accel_linear', float),
    'nav_dl': ('velocity.nav.decel_linear', float),
    'nav_aa': ('velocity.nav.accel_angular', float),
    'nav_da': ('velocity.nav.decel_angular', float),
    'joy_al': ('velocity.joy.accel_linear', float),
    'joy_dl': ('velocity.joy.decel_linear', float),
    'joy_aa': ('velocity.joy.accel_angular', float),
    'joy_da': ('velocity.joy.decel_angular', float),
    'init_al': ('velocity.init.accel_linear', float),
    'init_dl': ('velocity.init.decel_linear', float),
    'init_aa': ('velocity.init.accel_angular', float),
    'init_da': ('velocity.init.decel_angular', float),
}


class MotorGateway(Node):
    PRIORITY = ('joy', 'init', 'nav')

    def __init__(self):
        super().__init__('motor_gateway')
        load_params(self, PARAMS)

        # Salida a motores (MQTT)
        mqtt_cfg = load_config('mqtt.yaml')
        self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())
        self._output = MqttMotorOutput(self._mqtt, mqtt_cfg)

        # Estado
        self._vramp = SlewRateLimiter(0.0, 0.0)
        self._wramp = SlewRateLimiter(0.0, 0.0)
        self._profiles = {
            'nav': (self.nav_al, self.nav_dl, self.nav_aa, self.nav_da),
            'joy': (self.joy_al, self.joy_dl, self.joy_aa, self.joy_da),
            'init': (self.init_al, self.init_dl, self.init_aa, self.init_da),
        }
        self._target = {n: (0.0, 0.0) for n in self.PRIORITY}
        self._stamp = {n: 0.0 for n in self.PRIORITY}
        self._timeout = self.source_timeout_ms / 1000.0
        self._cmd_period = 1.0 / require_key(mqtt_cfg, 'telemetry_hz')   # cmd_vel es telemetria
        self._last_cmd_pub = 0.0
        self._mux_log = TransitionLogger(self.get_logger())   # cambio de fuente activa del mux

        # Entradas (fuentes de cmd_vel)
        self.create_subscription(Twist, NAV_CMD_VEL, lambda m: self._on_cmd('nav', m), 10)
        self.create_subscription(Twist, JOY_CMD_VEL, lambda m: self._on_cmd('joy', m), 10)
        self.create_subscription(Twist, INIT_CMD_VEL, lambda m: self._on_cmd('init', m), 10)

        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            f'motor_gateway iniciado (mux joy>init>nav, L={self.L}, max_output={self.max_output})')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, name, msg):
        self._target[name] = (msg.linear.x, msg.angular.z)
        self._stamp[name] = self._now()

    def _active(self):
        now = self._now()
        for name in self.PRIORITY:
            if now - self._stamp[name] < self._timeout:
                return name
        return None

    def _tick(self):
        name = self._active()
        self._mux_log.info(f'mux: {name} toma control' if name else 'mux: sin fuente -> frenando', key=name)
        if name is None:
            tv, tw = 0.0, 0.0
            self._vramp.set_rates(self.brake_lin, self.brake_lin)
            self._wramp.set_rates(self.brake_ang, self.brake_ang)
        else:
            tv, tw = self._target[name]
            al, dl, aa, da = self._profiles[name]
            self._vramp.set_rates(al, dl)
            self._wramp.set_rates(aa, da)
        v = self._vramp.apply(tv)
        w = self._wramp.apply(tw)
        velL, velR = differential_to_motor(v * self.linear_gain, w * self.angular_gain, self.L)
        velL = max(-self.max_output, min(self.max_output, velL))
        velR = max(-self.max_output, min(self.max_output, velR))
        self._output.send_motors(velL, velR)                  # rate de control (Pico)
        now = self._now()
        if now - self._last_cmd_pub >= self._cmd_period:      # cmd_vel: telemetria, throttleado
            self._last_cmd_pub = now
            self._output.send_cmd_vel(v, w)


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
