"""Controlador de trayectoria: navega entre waypoints con odometria.

Modos: stop_and_turn | arc (ALIGN->CRUISE->ARC->FINAL_APPROACH, ver arc_states.py).
Outputs MQTT via inyeccion. Logica ARC en ArcStateMixin.

Recibe /odom_filtered (ya filtrada y con offset de camara aplicado por odometry/zed.py
o odometry/rtk.py). NO filtra ni corrige posicion — eso es responsabilidad del nodo
de odometria correspondiente.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.adapters.mqtt.inputs.positions import MqttPositionsInput
from buey_robot.adapters.mqtt.outputs.config import MqttConfigOutput
from buey_robot.adapters.mqtt.outputs.status import MqttStatusOutput
from buey_robot.adapters.mqtt.outputs.waypoints import MqttWaypointsOutput
from buey_robot.navigation.arc_states import ArcStateMixin
from buey_robot.navigation.ramp_profile import RampProfile
from buey_robot.navigation.waypoint_manager import WaypointManager
from buey_robot.utils.config import load_config, require_key
from buey_robot.utils.gps_converter import GPSConverter
from buey_robot.utils.math import angle_diff, quaternion_to_yaw


class TrajectoryController(Node, ArcStateMixin):
    def __init__(self):
        super().__init__('trajectory_controller')

        # Declaracion de params (ROS2 estandar, sin defaults — falla si YAML no los provee)
        T = Parameter.Type
        _d = self.declare_parameter  # alias para compactar
        _d('controller.frequency_hz', T.DOUBLE);  _d('controller.mode', T.STRING)
        _d('controller.goal.position_tolerance_m', T.DOUBLE)
        _d('controller.goal.alignment_tolerance_deg', T.DOUBLE)
        _d('controller.velocity.cruise_linear_m_s', T.DOUBLE)
        _d('controller.velocity.cruise_angular_rad_s', T.DOUBLE)
        _d('controller.velocity.max_angular_rad_s', T.DOUBLE)
        _d('controller.velocity.min_linear_m_s', T.DOUBLE)
        _d('controller.angular.proportional_gain', T.DOUBLE)
        _d('controller.deceleration.distance_m', T.DOUBLE)
        _d('controller.ramp.accel_rate_linear', T.DOUBLE);  _d('controller.ramp.decel_rate_linear', T.DOUBLE)
        _d('controller.ramp.accel_rate_angular', T.DOUBLE); _d('controller.ramp.decel_rate_angular', T.DOUBLE)
        _d('controller.arc.linear_m_s', T.DOUBLE);  _d('controller.arc.angular_rad_s', T.DOUBLE)
        _d('controller.arc.start_distance_m', T.DOUBLE)
        _d('auto_load_waypoints', T.STRING)
        _d('goal_source', T.STRING)  # "waypoints_file" | "mqtt_positions"
        # Motor params — el gateway los lee por separado; aqui solo para publicar config MQTT
        _d('motor_control.wheel_separation', T.DOUBLE); _d('motor_control.max_output', T.DOUBLE)
        _d('motor_control.linear_gain', T.DOUBLE);      _d('motor_control.angular_gain', T.DOUBLE)
        _d('motor_control.soft_deadzone.low', T.DOUBLE); _d('motor_control.soft_deadzone.high', T.DOUBLE)

        # Lectura de params
        gp = self.get_parameter
        self.frequency = gp('controller.frequency_hz').value
        self.mode = gp('controller.mode').value
        self.goal_tolerance = gp('controller.goal.position_tolerance_m').value
        self.alignment_tolerance = math.radians(gp('controller.goal.alignment_tolerance_deg').value)
        self.cruise_linear = gp('controller.velocity.cruise_linear_m_s').value
        self.cruise_angular = gp('controller.velocity.cruise_angular_rad_s').value
        self.max_angular = gp('controller.velocity.max_angular_rad_s').value
        self.min_linear = gp('controller.velocity.min_linear_m_s').value
        self.angular_gain = gp('controller.angular.proportional_gain').value
        self.decel_distance = gp('controller.deceleration.distance_m').value

        # Rampas independientes para linear y angular
        self.ramp_linear = RampProfile(
            accel_rate=gp('controller.ramp.accel_rate_linear').value,
            decel_rate=gp('controller.ramp.decel_rate_linear').value,
        )
        self.ramp_angular = RampProfile(
            accel_rate=gp('controller.ramp.accel_rate_angular').value,
            decel_rate=gp('controller.ramp.decel_rate_angular').value,
        )

        # Arc mode params
        if self.mode == 'arc':
            self.arc_linear = gp('controller.arc.linear_m_s').value
            self.arc_angular = gp('controller.arc.angular_rad_s').value
            self.arc_start_distance = gp('controller.arc.start_distance_m').value
        self._arc_state = 'CRUISE'
        self._final_aligning = False

        # El controller suscribe directamente a /odom_filtered — la odometria ya llega
        # filtrada y con offset de camara aplicado por odometry/zed.py o odometry/rtk.py.
        self.waypoint_file = gp('auto_load_waypoints').value
        self.goal_source = gp('goal_source').value

        # Cliente MQTT (mqtt.yaml no es param ROS2)
        mqtt_cfg = load_config('mqtt.yaml')
        self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())
        self.status_output = MqttStatusOutput(self._mqtt, mqtt_cfg)
        self.config_output = MqttConfigOutput(self._mqtt, mqtt_cfg)
        self.waypoints_output = MqttWaypointsOutput(self._mqtt, mqtt_cfg)

        # Modo navegacion BASE->START: la meta es el punto START publicado por el
        # dashboard, convertido a x/y con el mismo origen (BASE) que usa rtk.py,
        # de modo que cae en el mismo frame que /odom_filtered.
        self._gps_converter = GPSConverter()
        self._start = None
        if self.goal_source == 'mqtt_positions':
            topics = {
                'base': require_key(mqtt_cfg, 'topics', 'positions_base'),
                'start': require_key(mqtt_cfg, 'topics', 'positions_start'),
            }
            self._positions_input = MqttPositionsInput(
                self._mqtt, on_base=self._on_base, on_start=self._on_start, topics=topics)

        # Estado
        self.current_x = self.current_y = self.current_heading = None
        self.current_speed = 0.0
        self.odom_received = False
        self.wp_manager = WaypointManager()
        self.trajectory_active = False
        self.aligning = False

        # Publishers: solo cmd_vel y status — la odometria la produce odometry/zed.py o rtk.py
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/trajectory/status', 10)

        # Subscriber: /odom_filtered ya viene procesada desde el nodo de odometria
        self.create_subscription(Odometry, '/odom_filtered', self.odom_callback, 10)

        # Timers
        self.create_timer(1.0 / self.frequency, self.control_loop)

        self.auto_load_attempted = False
        if self.goal_source == 'mqtt_positions':
            self.mqtt_goal_timer = self.create_timer(1.0, self.try_load_mqtt_goal)
            self.get_logger().info('Meta desde MQTT: navegacion BASE->START')
        elif self.waypoint_file:
            self.auto_load_timer = self.create_timer(1.0, self.try_auto_load_waypoints)
            self.get_logger().info(f'Auto-carga configurada: {self.waypoint_file}')

        self._config_published = False
        self.create_timer(3.0, self._publish_config_once)

        self.get_logger().info(f'Trajectory Controller iniciado (modo: {self.mode})')
        self.get_logger().info(f'  Odom: /odom_filtered (filtrada por odometry node)')
        self.get_logger().info(f'  Goal tolerance: {self.goal_tolerance:.2f}m')

    def odom_callback(self, msg: Odometry):
        """Recibe /odom_filtered: posicion ya filtrada y con offset de camara aplicado."""
        q = msg.pose.pose.orientation
        self.current_heading = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx**2 + vy**2)

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(
                f'Primer odom: x={self.current_x:.2f}, y={self.current_y:.2f}, '
                f'heading={math.degrees(self.current_heading):.1f} deg'
            )

    def _publish_config_once(self):
        if self._config_published or not self._mqtt.is_connected:
            return
        self._config_published = True
        gp = self.get_parameter
        self.config_output.publish_once(
            nav_params={
                'mode': self.mode,
                'goal_tolerance_m': self.goal_tolerance,
                'alignment_tolerance_deg': math.degrees(self.alignment_tolerance),
                'cruise_linear_m_s': self.cruise_linear,
                'cruise_angular_rad_s': self.cruise_angular,
                'max_angular_rad_s': self.max_angular,
            },
            motor_params={
                'wheel_separation': gp('motor_control.wheel_separation').value,
                'max_output': gp('motor_control.max_output').value,
                'linear_gain': gp('motor_control.linear_gain').value,
                'angular_gain': gp('motor_control.angular_gain').value,
                'deadzone_low': gp('motor_control.soft_deadzone.low').value,
                'deadzone_high': gp('motor_control.soft_deadzone.high').value,
            },
        )

    def try_auto_load_waypoints(self):
        if self.auto_load_attempted:
            return
        if not self.odom_received or not self.waypoint_file:
            return

        self.get_logger().info(f'Cargando waypoints: {self.waypoint_file}')
        try:
            origin_x = self.current_x if self.current_x is not None else 0.0
            origin_y = self.current_y if self.current_y is not None else 0.0
            wps = self.wp_manager.load_from_file(self.waypoint_file, origin_x, origin_y)
            self.trajectory_active = True
            self.aligning = False
            self.auto_load_attempted = True

            if hasattr(self, 'auto_load_timer'):
                self.auto_load_timer.cancel()

            self.get_logger().info(f'Cargados {len(wps)} waypoints')
            for i, (wx, wy) in enumerate(wps):
                self.get_logger().info(f'  WP{i}: x={wx:.2f}, y={wy:.2f}')

            self.waypoints_output.send_xy(self.wp_manager.waypoints)

        except Exception as e:
            self.get_logger().error(f'Error cargando waypoints: {e}')

    # ------------------------------------------------------------------
    # Modo BASE->START (goal_source == 'mqtt_positions')
    # ------------------------------------------------------------------

    def _on_base(self, base: dict):
        """BASE: fija el origen del frame local (mismo que rtk.py)."""
        self._gps_converter.set_origin(base['lat'], base['lon'])
        self.get_logger().info(
            f"BASE recibida (origen): lat={base['lat']:.8f}, lon={base['lon']:.8f}")

    def _on_start(self, start: dict):
        """START: punto meta. Se convierte a x/y cuando el origen esta listo."""
        self._start = start
        self.get_logger().info(
            f"START recibido (meta): lat={start['lat']:.8f}, lon={start['lon']:.8f}")

    def try_load_mqtt_goal(self):
        if self.auto_load_attempted:
            return
        if not self.odom_received:
            return
        if not self._gps_converter.origin_set or self._start is None:
            self.get_logger().warn(
                'Esperando BASE y START (bueyuy/positions/base|start)...',
                throttle_duration_sec=5.0,
            )
            return

        try:
            goal_x, goal_y = self._gps_converter.gps_to_local(
                self._start['lat'], self._start['lon'])
            self.wp_manager.set_waypoints([(goal_x, goal_y)])
            self.trajectory_active = True
            self.aligning = False
            self.auto_load_attempted = True

            if hasattr(self, 'mqtt_goal_timer'):
                self.mqtt_goal_timer.cancel()

            self.get_logger().info(f'Meta START: x={goal_x:.2f}, y={goal_y:.2f}')
            self.waypoints_output.send_xy(self.wp_manager.waypoints)

        except Exception as e:
            self.get_logger().error(f'Error cargando meta START: {e}')

    def control_loop(self):
        if self.current_x is None or self.current_y is None:
            return

        if not self.trajectory_active or self.wp_manager.is_complete():
            self._publish_stop()
            return

        if self.current_heading is None:
            goal = self.wp_manager.current_goal()
            if goal:
                self.current_heading = math.atan2(
                    goal[1] - self.current_y, goal[0] - self.current_x)

        goal = self.wp_manager.current_goal()
        if goal is None:
            self.trajectory_active = False
            self._publish_stop()
            self.get_logger().info('Trayectoria completada')
            self._publish_status('COMPLETED')
            return

        goal_x, goal_y = goal

        if self.mode == 'arc':
            self._control_arc(goal_x, goal_y)
        else:
            self._control_stop_and_turn(goal_x, goal_y)

    def _control_stop_and_turn(self, goal_x: float, goal_y: float):
        """Modo stop_and_turn: girar, avanzar, frenar, repetir."""
        if self.aligning:
            self.ramp_linear.reset()
            angle_to_goal = math.atan2(goal_y - self.current_y, goal_x - self.current_x)
            heading_error = angle_diff(angle_to_goal, self.current_heading)

            if abs(heading_error) < self.alignment_tolerance:
                self.aligning = False
                self.get_logger().info(
                    f'Alineado al WP {self.wp_manager.progress_string()} - Avanzando')
            else:
                angular_vel = math.copysign(self.cruise_angular, heading_error)
                self._publish_velocity(0.0, angular_vel)
                self._publish_status(
                    f'Aligning to WP {self.wp_manager.progress_string()}, '
                    f'heading_err={math.degrees(heading_error):.1f} deg')
                return

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.goal_tolerance:
            self.ramp_linear.reset()
            self.ramp_angular.reset()
            self.get_logger().info(
                f'WP {self.wp_manager.progress_string()} alcanzado (dist={distance:.2f}m)')
            self.wp_manager.advance()
            if not self.wp_manager.is_complete():
                self.aligning = True
                self._publish_stop()
            return

        if distance < self.decel_distance:
            target_vel = max(self.cruise_linear * (distance / self.decel_distance), self.min_linear)
        else:
            target_vel = self.cruise_linear

        linear_vel = self.ramp_linear.apply(target_vel)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = angle_diff(angle_to_goal, self.current_heading)
        target_angular = heading_error * self.angular_gain
        target_angular = max(-self.max_angular, min(self.max_angular, target_angular))
        angular_vel = self.ramp_angular.apply(target_angular)

        self._publish_velocity(linear_vel, angular_vel)
        self._publish_status(
            f'Following WP {self.wp_manager.progress_string()}, '
            f'dist={distance:.2f}m, heading_err={math.degrees(heading_error):.1f} deg')

    def _publish_velocity(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def _publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.status_output.send(status)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
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
