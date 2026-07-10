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
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Bool, Empty

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.adapters.mqtt.inputs.positions import MqttPositionsInput
from buey_robot.adapters.mqtt.inputs.start import MqttStartInput
from buey_robot.adapters.mqtt.inputs.waypoints import MqttWaypointsInput
from buey_robot.adapters.mqtt.outputs.config import MqttConfigOutput
from buey_robot.adapters.mqtt.outputs.status import MqttStatusOutput
from buey_robot.adapters.mqtt.outputs.waypoints import MqttWaypointsOutput
from buey_robot.navigation.arc_states import ArcStateMixin
from buey_robot.navigation.ramp_profile import RampProfile
from buey_robot.navigation.waypoint_manager import WaypointManager
from buey_robot.utils.config import load_config, require_key
from buey_robot.utils.gps_converter import GPSConverter
from buey_robot.utils.math import angle_diff, quaternion_to_yaw

# Segundos tras enviar /gyro/calibrate durante los que se ignora /heading/gyro
# (para descartar mensajes viejos previos a la recalibracion). El auto-bias tarda
# ~10s (200 muestras), asi que este margen corto no lo alcanza.
GYRO_CALIB_TRIGGER_GRACE_S = 1.5
# Retardo antes de disparar el trigger: da tiempo a descubrir la suscripcion de
# mpu6050_gyro y a que el primer odom llegue (robot ya parado en el inicio).
GYRO_CALIB_TRIGGER_DELAY_S = 2.0


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
        # Creep recto inicial: avanzar en linea recta antes de navegar para generar
        # COG y alinear el heading fused (gyro+GPS) — sin esto el robot rota en el
        # lugar con un heading todavia sin alinear.
        _d('controller.prealign.enabled', T.BOOL)
        _d('controller.prealign.distance_m', T.DOUBLE)
        # Endurecimiento del arranque: no soltar la navegacion solo por haber
        # avanzado distance_m; ademas confirmar que el heading fused convergio
        # (mpu6050_gyro publica /heading/fused_ready). max_distance_m es el tope de
        # seguridad del creep si la convergencia nunca llega (COG/RTK malos).
        _d('controller.prealign.require_fused_convergence', T.BOOL)
        _d('controller.prealign.max_distance_m', T.DOUBLE)
        # Esperar la calibracion del gyro (auto-bias, robot quieto) antes de moverse.
        # mpu6050_gyro solo publica /heading/gyro DESPUES de calibrar el bias.
        _d('controller.wait_for_gyro_calibration', T.BOOL)
        # Disparar una calibracion FRESCA del gyro al arrancar la navegacion (en vez
        # de usar la que hizo mpu6050_gyro al levantar outdoor_rtk): el robot esta
        # recien parado y quieto en el punto de inicio. El controller publica
        # /gyro/calibrate y espera a que reaparezca /heading/gyro (recalibrado).
        _d('controller.trigger_gyro_calibration', T.BOOL)
        _d('auto_load_waypoints', T.STRING)
        _d('goal_source', T.STRING)  # "waypoints_file" (x,y local) | "mqtt_waypoints" (ruta MQTT)
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
        self.prealign_enabled = gp('controller.prealign.enabled').value
        self.prealign_distance = gp('controller.prealign.distance_m').value
        self.prealign_require_converge = gp('controller.prealign.require_fused_convergence').value
        self.prealign_max_distance = gp('controller.prealign.max_distance_m').value
        self._prealign_done = False
        self._prealign_start = None
        # Convergencia del heading fused: si no se exige, arranca ya como True (indoor
        # /ZED no tiene gyro+GPS). Si se exige, lo habilita /heading/fused_ready.
        self._fused_converged = not self.prealign_require_converge
        self.wait_for_gyro_cal = gp('controller.wait_for_gyro_calibration').value
        self.trigger_gyro_cal = gp('controller.trigger_gyro_calibration').value
        # mpu6050_gyro publica /heading/gyro solo tras calibrar el bias -> el primer
        # mensaje = gyro calibrado. Hasta entonces el robot no se mueve.
        self._gyro_calibrated = not self.wait_for_gyro_cal
        # Al disparar una recalibracion, mpu6050_gyro deja de publicar /heading/gyro
        # ~10s. Ignoramos /heading/gyro durante este margen tras enviar el trigger
        # para no tomar un mensaje VIEJO (pre-recalibracion) como "ya calibrado".
        self._calib_trigger_time = None

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

        # Origen del frame local (ENU), fijado por la BASE del dashboard (mismo que
        # rtk.py), para convertir waypoints lat/lon al frame de /odom_filtered.
        self._gps_converter = GPSConverter()
        # Ruta empujada por MQTT que llego antes que la BASE: (waypoints_gps, loop).
        # Se convierte y carga apenas se fija el origen (ver _on_waypoints / _on_base).
        self._pending_waypoints_gps = None
        # Recorrer la ruta en bucle (lo setea _on_waypoints desde el flag MQTT "loop").
        self._loop = False
        # Ruta cargada (waypoints seteados) pero AUN NO arrancada: espera el GO
        # (bueyuy/navigation/start). Separar carga de arranque evita que una ruta
        # retenida haga mover el robot solo, y permite revisar la ruta en el mapa.
        self._route_loaded = False

        # Modo mqtt_waypoints: la ruta llega como lat/lon por MQTT (bueyuy/waypoints)
        # y se fija al origen BASE (fija, mismo frame que rtk.py) -> los waypoints no
        # se desplazan si el robot arranca corrido. Requiere la BASE del dashboard.
        if self.goal_source == 'mqtt_waypoints':
            topics = {
                'base': require_key(mqtt_cfg, 'topics', 'positions_base'),
                'start': require_key(mqtt_cfg, 'topics', 'positions_start'),
            }
            # Solo interesa la BASE (origen). START no se usa en este modo.
            self._positions_input = MqttPositionsInput(
                self._mqtt, on_base=self._on_base, on_start=lambda d: None, topics=topics)
            self._waypoints_input = MqttWaypointsInput(
                self._mqtt, on_waypoints=self._on_waypoints,
                topic=require_key(mqtt_cfg, 'topics', 'waypoints'))
            # Disparador de arranque (GO): el dashboard lo publica cuando el robot
            # esta en el inicio. Recien ahi arranca la ruta cargada.
            self._start_input = MqttStartInput(
                self._mqtt, on_start=self._on_nav_start,
                topic=require_key(mqtt_cfg, 'topics', 'nav_start'))

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

        # Señal de gyro calibrado: mpu6050_gyro publica /heading/gyro recien tras
        # el auto-bias (y solo si el gyro da señal real). El primer mensaje habilita
        # el movimiento.
        if self.wait_for_gyro_cal:
            self.create_subscription(Float32, '/heading/gyro', self._gyro_ready_cb, 10)

        # Disparo de una calibracion fresca del gyro desde la navegacion: el robot
        # esta recien parado en el inicio. Se publica /gyro/calibrate una vez tras un
        # retardo (descubrimiento + primer odom) y se espera /heading/gyro recalibrado.
        self._calib_pub = None
        if self.trigger_gyro_cal:
            self._calib_pub = self.create_publisher(Empty, '/gyro/calibrate', 10)
            self._calib_timer = self.create_timer(
                GYRO_CALIB_TRIGGER_DELAY_S, self._send_gyro_calibrate)

        # Señal de heading fused convergido: mpu6050_gyro publica /heading/fused_ready
        # (latched) cuando el offset gyro->GPS se estabiliza. El creep de pre-alineacion
        # no termina hasta recibirlo (evita soltar la nav con heading sin alinear).
        if self.prealign_require_converge:
            latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Bool, '/heading/fused_ready', self._fused_ready_cb, latched)

        # Timers
        self.create_timer(1.0 / self.frequency, self.control_loop)

        self.auto_load_attempted = False
        if self.goal_source == 'mqtt_waypoints':
            # La ruta la empuja el dashboard por bueyuy/waypoints (ver _on_waypoints);
            # no hay timer de carga, el callback MQTT arranca la trayectoria.
            self.get_logger().info('Meta desde MQTT: ruta de waypoints (bueyuy/waypoints, origen=BASE)')
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

    def _send_gyro_calibrate(self):
        """Dispara UNA calibracion fresca del gyro (robot parado en el inicio)."""
        self._calib_timer.cancel()
        self._calib_trigger_time = self.get_clock().now().nanoseconds * 1e-9
        self._gyro_calibrated = False  # esperar la recalibracion, no la vieja
        if self._calib_pub is not None:
            self._calib_pub.publish(Empty())
        self.get_logger().info(
            'Solicitada recalibracion del gyro (/gyro/calibrate) — mantener el robot QUIETO')

    def _gyro_ready_cb(self, msg: Float32):
        """/heading/gyro = mpu6050_gyro termino el auto-bias -> habilita mover.

        Si se disparo una recalibracion, ignorar los mensajes dentro del margen tras
        el trigger: son /heading/gyro VIEJOS (previos a que el gyro deje de publicar
        para recalibrar). Recien pasado el margen, el proximo mensaje es el recalibrado.
        """
        if self._gyro_calibrated:
            return
        if self.trigger_gyro_cal:
            if self._calib_trigger_time is None:
                return  # trigger aun no enviado
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._calib_trigger_time < GYRO_CALIB_TRIGGER_GRACE_S:
                return  # podria ser un /heading/gyro previo a la recalibracion
        self._gyro_calibrated = True
        self.get_logger().info('Gyro calibrado (/heading/gyro) — navegacion habilitada')

    def _fused_ready_cb(self, msg: Bool):
        """/heading/fused_ready = estado de convergencia del offset gyro->GPS.

        Bidireccional: al recalibrar el gyro, mpu6050_gyro republica false y hay que
        volver a esperar la convergencia (no quedarse con un true latcheado viejo)."""
        was = self._fused_converged
        self._fused_converged = bool(msg.data)
        if self._fused_converged and not was:
            self.get_logger().info(
                'Heading fused convergido (/heading/fused_ready) — pre-alineacion habilitada')

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
    # Modo mqtt_waypoints: origen BASE + ruta de waypoints por MQTT
    # ------------------------------------------------------------------

    def _on_base(self, base: dict):
        """BASE: fija el origen del frame local (mismo que rtk.py)."""
        self._gps_converter.set_origin(base['lat'], base['lon'])
        self.get_logger().info(
            f"BASE recibida (origen): lat={base['lat']:.8f}, lon={base['lon']:.8f}")
        # Si llego una ruta MQTT antes que la BASE, cargarla ahora que hay origen.
        if self._pending_waypoints_gps is not None:
            pending_wps, pending_loop = self._pending_waypoints_gps
            self._pending_waypoints_gps = None
            self._on_waypoints(pending_wps, pending_loop)

    def _on_waypoints(self, waypoints_gps: list, loop: bool = False):
        """CARGA una ruta de waypoints GPS via MQTT (bueyuy/waypoints), pero NO
        arranca: deja la ruta lista y espera el GO (bueyuy/navigation/start).

        Convierte cada {lat,lon} a x/y con el origen BASE (mismo frame que
        /odom_filtered) y la publica al mapa. loop=true: al completar el ultimo
        waypoint vuelve al primero (ver control_loop). Si el origen aun no esta
        (falta BASE), guarda la ruta y la carga al llegar la BASE (ver _on_base).

        Lista vacia (waypoints_gps: []) = comando LIMPIAR/idle: frena y descarta."""
        if not waypoints_gps:
            self._clear_route()
            return

        if not self._gps_converter.origin_set:
            self._pending_waypoints_gps = (waypoints_gps, loop)
            self.get_logger().warn(
                'Waypoints MQTT recibidos pero falta BASE: se cargaran al fijar el origen')
            return

        try:
            local = [self._gps_converter.gps_to_local(wp['lat'], wp['lon'])
                     for wp in waypoints_gps]
        except Exception as e:
            self.get_logger().error(f'Error cargando waypoints MQTT: {e}')
            return

        self.wp_manager.set_waypoints(local)
        self._loop = loop
        self._route_loaded = True
        self.trajectory_active = False   # NO arrancar: esperar el GO
        self.aligning = False
        self.auto_load_attempted = True

        self.get_logger().info(
            f'Ruta cargada: {len(local)} waypoints (origen=BASE, loop={loop}) — '
            f'esperando GO (bueyuy/navigation/start)')
        for i, (wx, wy) in enumerate(local):
            self.get_logger().info(f'  WP{i}: x={wx:.2f}, y={wy:.2f}')

        self.waypoints_output.send_xy(self.wp_manager.waypoints)
        self._publish_status(f'Ruta cargada ({len(local)} wps) — esperando GO')

    def _clear_route(self):
        """Comando limpiar/idle (waypoints_gps: []): frena y descarta la ruta.
        El robot no vuelve a moverse hasta cargar otra ruta + GO. Es el 'Detener'
        del dashboard."""
        self._route_loaded = False
        self.trajectory_active = False
        self._loop = False
        self._pending_waypoints_gps = None
        self._publish_stop()
        self.waypoints_output.send_xy([])   # limpiar la ruta del mapa
        self.get_logger().info('Ruta limpiada (waypoints vacio) — idle')
        self._publish_status('Idle (sin ruta)')

    def _on_nav_start(self):
        """GO (bueyuy/navigation/start): arranca la ruta ya cargada desde el primer
        waypoint. Ignorado si no hay ruta cargada. Es la confirmacion del operador
        (robot en el inicio) para soltar la navegacion."""
        if not self._route_loaded:
            self.get_logger().warn('GO recibido pero no hay ruta cargada — ignorado')
            self._publish_status('GO sin ruta cargada')
            return

        # Re-hacer la secuencia de arranque en CADA GO: recalibrar el gyro
        # (bias + offset) y re-correr el creep antes de navegar. Sin esto, el heading
        # fused viejo puede estar corrompido (p.ej. tras alinear con joystick /
        # marcha atras: el COG apunta hacia atras y desvia el offset) y el robot
        # gira al reves en el primer waypoint. La recalibracion re-snapea el offset
        # desde el COG yendo derecho durante el creep.
        self._prealign_done = False
        self._prealign_start = None
        self._fused_converged = not self.prealign_require_converge
        if self.trigger_gyro_cal:
            self._send_gyro_calibrate()   # resetea bias+offset; robot quieto hasta recalibrar

        self.wp_manager.restart()        # arrancar/re-correr desde WP0
        self.trajectory_active = True
        self.aligning = False
        self.get_logger().info('GO recibido — recalibrando gyro + creep antes de navegar la ruta')
        self._publish_status('GO — recalibrando + alineando')

    def control_loop(self):
        if self.current_x is None or self.current_y is None:
            return

        if not self.trajectory_active:
            self._publish_stop()
            return

        if self.wp_manager.is_complete():
            if self._loop:
                # Ruta en bucle: volver al primer waypoint y re-alinear en vez de terminar.
                self.wp_manager.restart()
                self.aligning = True
                self.get_logger().info('Loop: ruta completa -> reiniciando desde WP0')
            else:
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

        # Esperar la calibracion del gyro (auto-bias con el robot QUIETO) antes de
        # cualquier movimiento. Hasta el primer /heading/gyro el robot no se mueve.
        if not self._gyro_calibrated:
            self._publish_stop()
            self._publish_status('Esperando calibracion del gyro (robot quieto)...')
            return

        # Creep recto inicial: antes de rotar/navegar, avanzar en linea recta para
        # generar COG y que el heading fused (gyro+GPS) se alinee. Sin esto el robot
        # rota en el lugar con un heading todavia sin alinear.
        if self.prealign_enabled and not self._prealign_done:
            self._prealign_straight()
            return

        if self.mode == 'arc':
            self._control_arc(goal_x, goal_y)
        else:
            self._control_stop_and_turn(goal_x, goal_y)

    def _prealign_straight(self):
        """Avanza recto para generar COG y alinear el heading fused (gyro+GPS) antes
        de navegar. Evita rotar en el lugar sin heading.

        No basta con avanzar distance_m: el creep sigue hasta que ADEMAS el heading
        fused convergio (/heading/fused_ready, cuando se exige). Asi se evita soltar
        la navegacion con el offset aun desalineado (causa del 'arranca diagonal').
        max_distance_m es el tope de seguridad si la convergencia nunca llega."""
        if self._prealign_start is None:
            self._prealign_start = (self.current_x, self.current_y)
            self.ramp_linear.reset()
            self.get_logger().info(
                f'Pre-alineacion: avanzando recto (min {self.prealign_distance:.2f}m'
                + (', esperando convergencia del heading fused' if self.prealign_require_converge else '')
                + f', max {self.prealign_max_distance:.2f}m) para generar COG')
        dx = self.current_x - self._prealign_start[0]
        dy = self.current_y - self._prealign_start[1]
        traveled = math.sqrt(dx * dx + dy * dy)

        min_reached = traveled >= self.prealign_distance
        if min_reached and self._fused_converged:
            self._prealign_done = True
            self.ramp_linear.reset()
            self.get_logger().info(
                f'Pre-alineacion completa ({traveled:.2f}m, heading fused alineado) — navegando')
            return

        # Tope de seguridad: no arrastrarse indefinidamente si el heading no converge
        # (COG/RTK malos). Navegar igual, avisando fuerte.
        if traveled >= self.prealign_max_distance:
            self._prealign_done = True
            self.ramp_linear.reset()
            self.get_logger().warn(
                f'Pre-alineacion: tope {self.prealign_max_distance:.2f}m sin confirmar '
                f'convergencia del heading fused — navegando igual (revisar COG/RTK)')
            return

        linear_vel = self.ramp_linear.apply(self.cruise_linear)
        self._publish_velocity(linear_vel, 0.0)  # recto: sin componente angular
        conv = 'OK' if self._fused_converged else 'esperando'
        self._publish_status(
            f'Pre-align recto {traveled:.2f}m (min {self.prealign_distance:.2f}, '
            f'max {self.prealign_max_distance:.2f}) fused={conv}')

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
