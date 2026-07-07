"""mpu6050_gyro: calibra el giroscopio del MPU6050 e integra un heading de yaw.

El firmware DUAL de la ESP32 (nodo esp32_dual_imu_node) publica las dos IMUs:
/imu/data (LSM303, accel+mag, sin gyro) y /mpu6050/imu/data (MPU6050, accel+gyro).
Este nodo consume el MPU6050 (sensor_msgs/Imu CRUDO en /mpu6050/imu/data) y:
  - resta el bias del giroscopio (de config o auto-estimado en reposo)
  - integra angular_velocity.z (yaw rate, rad/s) -> heading relativo en grados
  - republica el Imu corregido (/mpu6050/imu/data_calibrated) y el heading
    (/heading/gyro, junto a /heading/gps y /heading/imu)
  - (opcional) fusiona con el COG del GPS: en movimiento + RTK fix estima un
    offset y lo suma al gyro -> /heading/fused, heading ABSOLUTO ENU sin drift
    (gyro = parte rapida de los giros; GPS = referencia absoluta lenta). Ese
    /heading/fused es EL heading que rtk.py adopta para /odom_filtered (con
    gps.imu.use_imu_heading=true) y que llega a telemetria via pose_publisher.

IMPORTANTE: el heading de gyro DERIVA. No tiene referencia absoluta: es suave y
preciso a corto plazo pero acumula error sin cota. Por eso se fusiona con el COG
del GPS (referencia absoluta lenta) -> /heading/fused. /heading/gyro (crudo) se
expone aparte para telemetria/depuracion.

Los topics de entrada/salida y la config (config/imu.yaml, bloque mpu6050_gyro)
son configurables.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool, Empty
from gps_msgs.msg import GPSFix


def _wrap180(a):
    """Normaliza un angulo en grados a (-180, 180]."""
    return (a + 180.0) % 360.0 - 180.0


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
        # Topic para disparar una recalibracion on-demand (std_msgs/Empty). Lo usa el
        # controller para recalibrar el bias al arrancar la navegacion (robot recien
        # parado y quieto en el inicio), en vez de depender de la calibracion que se
        # hizo al levantar outdoor_rtk.
        self.declare_parameter('calibrate_topic', Parameter.Type.STRING)
        # Deteccion de gyro muerto/congelado: un MPU6050 vivo siempre tiene ruido
        # (std del wz en reposo > 0); un topic equivocado (LSM303, sin gyro) o un
        # gyro que no reporta da wz == 0.0 exacto (std ~ 0). Si el spread queda por
        # debajo de esto, NO se completa el auto-bias (no publica /heading/gyro) ->
        # el controller queda bloqueado en vez de navegar con heading muerto.
        self.declare_parameter('gyro_alive_min_std', Parameter.Type.DOUBLE)
        self.declare_parameter('heading_initial_deg', Parameter.Type.DOUBLE)
        self.declare_parameter('heading_invert', Parameter.Type.BOOL)
        # Fusion con COG del GPS (heading absoluto sin drift)
        self.declare_parameter('gps_fusion_enabled', Parameter.Type.BOOL)
        self.declare_parameter('gps_fix_topic', Parameter.Type.STRING)
        self.declare_parameter('fused_heading_topic', Parameter.Type.STRING)
        # Gyro alineado al COG, en convencion BRUJULA (0=Norte, horario): = (90-fused).
        # Yendo derecho coincide con el COG del GPS -> la flecha del dashboard queda
        # alineada, no defasada como el /heading/gyro crudo (ENU, cero arbitrario).
        self.declare_parameter('gyro_compass_topic', Parameter.Type.STRING)
        self.declare_parameter('gps_min_speed', Parameter.Type.DOUBLE)
        self.declare_parameter('gps_require_rtk', Parameter.Type.BOOL)
        self.declare_parameter('offset_alpha', Parameter.Type.DOUBLE)
        self.declare_parameter('offset_init_samples', Parameter.Type.INTEGER)
        self.declare_parameter('straight_max_yaw_rate', Parameter.Type.DOUBLE)
        # Convergencia del offset: cuando el residual (target-offset) por fix se
        # mantiene chico varias muestras seguidas, el heading fused esta alineado.
        # Se publica /heading/fused_ready (Bool latched) -> el controller lo espera
        # antes de terminar el creep de pre-alineacion y soltar la navegacion.
        self.declare_parameter('fused_ready_topic', Parameter.Type.STRING)
        self.declare_parameter('converge_tol_deg', Parameter.Type.DOUBLE)
        self.declare_parameter('converge_min_samples', Parameter.Type.INTEGER)

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
        self.gyro_alive_min_std = self.get_parameter('gyro_alive_min_std').value
        self.calibrate_topic = self.get_parameter('calibrate_topic').value
        self.heading_deg = self.get_parameter('heading_initial_deg').value
        self.heading_invert = self.get_parameter('heading_invert').value
        self.gps_fusion = self.get_parameter('gps_fusion_enabled').value
        self.gps_fix_topic = self.get_parameter('gps_fix_topic').value
        self.fused_topic = self.get_parameter('fused_heading_topic').value
        self.gyro_compass_topic = self.get_parameter('gyro_compass_topic').value
        self.gps_min_speed = self.get_parameter('gps_min_speed').value
        self.gps_require_rtk = self.get_parameter('gps_require_rtk').value
        self.offset_alpha = self.get_parameter('offset_alpha').value
        self.init_samples = self.get_parameter('offset_init_samples').value
        self.straight_max_yaw_rate = self.get_parameter('straight_max_yaw_rate').value
        self.fused_ready_topic = self.get_parameter('fused_ready_topic').value
        self.converge_tol_deg = self.get_parameter('converge_tol_deg').value
        self.converge_min_samples = self.get_parameter('converge_min_samples').value

        # Estado de integracion
        self._last_time = None  # seg (reloj del nodo)

        # Estado de auto-calibracion de bias en reposo
        self._calibrating = self.auto_bias
        self._acc = [0.0, 0.0, 0.0]
        self._acc_sq_z = 0.0  # suma de wz^2 -> std del yaw rate (deteccion gyro muerto)
        self._n = 0

        # Estado de fusion con GPS: offset (grados) que lleva el gyro al frame
        # absoluto ENU del COG GPS. None hasta completar el warm-up inicial.
        self._gps_offset = None
        self._yaw_rate = 0.0  # rad/s, ultimo yaw rate con bias corregido (gate de recta)
        self._init_sin = 0.0  # acumuladores media circular del warm-up
        self._init_cos = 0.0
        self._init_n = 0
        # Convergencia del offset: fixes rectos consecutivos con residual chico.
        self._converge_count = 0
        self._fused_ready = False

        self._heading_initial = self.heading_deg  # para reiniciar en recalibraciones

        self.heading_pub = self.create_publisher(Float32, self.heading_topic, 10)
        self.data_pub = self.create_publisher(Imu, self.data_cal_topic, 10)
        self.create_subscription(Imu, self.imu_topic, self._imu_callback, 10)
        # Recalibracion on-demand (el controller la dispara al arrancar la nav)
        self.create_subscription(Empty, self.calibrate_topic, self._calibrate_cb, 10)

        if self.gps_fusion:
            self.fused_pub = self.create_publisher(Float32, self.fused_topic, 10)
            # Gyro alineado al COG en convencion brujula (para la flecha del dashboard)
            self.gyro_compass_pub = self.create_publisher(Float32, self.gyro_compass_topic, 10)
            # Latched (transient_local): un subscriber tardio (el controller) recibe
            # el ultimo estado aunque la convergencia haya ocurrido antes de arrancar.
            latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.fused_ready_pub = self.create_publisher(Bool, self.fused_ready_topic, latched)
            self.fused_ready_pub.publish(Bool(data=False))  # estado inicial: no convergido
            self.create_subscription(GPSFix, self.gps_fix_topic, self._gps_callback, 10)

        self.get_logger().info(
            f'mpu6050_gyro: {self.imu_topic} -> {self.heading_topic} (+ {self.data_cal_topic})')
        if self.gps_fusion:
            self.get_logger().info(
                f'  fusion GPS: ON, {self.gps_fix_topic} -> {self.fused_topic} '
                f'(min_speed={self.gps_min_speed}, require_rtk={self.gps_require_rtk}, '
                f'alpha={self.offset_alpha})')
            self.get_logger().info(
                f'  convergencia: {self.fused_ready_topic} cuando |residual|<'
                f'{self.converge_tol_deg:.0f} deg x{self.converge_min_samples} fixes rectos')
            self.get_logger().info(
                f'  gyro alineado (brujula): {self.gyro_compass_topic} = (90 - fused)')
        if self._calibrating:
            self.get_logger().info(
                f'  auto-bias: ON (mantener el robot QUIETO, {self.auto_samples} muestras)')
        else:
            self.get_logger().info(f'  auto-bias: OFF, usando gyro_bias={self.bias}')
        self.get_logger().info(f'  recalibracion on-demand: {self.calibrate_topic}')

    def _imu_callback(self, msg: Imu):
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        if self._calibrating:
            self._collect_bias(wx, wy, wz)
            return

        # Restar bias
        cwz = wz - self.bias[2]
        self._yaw_rate = cwz  # para el gate de "yendo derecho" de la fusion GPS

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

        # Heading fusionado: gyro + offset GPS (absoluto ENU, sin drift). Solo
        # una vez que el GPS fijo el offset al menos una vez.
        if self.gps_fusion and self._gps_offset is not None:
            fused = (self.heading_deg + self._gps_offset) % 360.0
            self.fused_pub.publish(Float32(data=float(fused)))
            # Mismo heading en convencion BRUJULA (0=Norte, horario), alineado al COG:
            # (90 - fused). Yendo derecho == COG del GPS -> la flecha no queda defasada.
            gyro_compass = (90.0 - fused) % 360.0
            self.gyro_compass_pub.publish(Float32(data=float(gyro_compass)))

        self._publish_calibrated(msg)

    def _gps_callback(self, msg: GPSFix):
        """Alinea el gyro al COG del GPS en movimiento (complementario lento).

        El COG del GPS es heading absoluto confiable con RTK fix y velocidad. Se
        convierte a yaw ENU (90 - track, igual que rtk.py) y se estima el offset
        gyro->absoluto, suavizado con offset_alpha para no seguir el ruido del COG.
        """
        speed = msg.speed
        if speed < self.gps_min_speed:
            return
        if self.gps_require_rtk and msg.status.status < 2:
            return
        track = msg.track
        if math.isnan(track) or math.isnan(speed):
            return
        # Solo corregir el offset YENDO DERECHO: en giros el COG GPS laggea al
        # gyro e inyecta error (y un snap durante un giro queda muy corrido).
        # |yaw rate| bajo = recta.
        if abs(self._yaw_rate) > self.straight_max_yaw_rate:
            return

        # COG brujula -> yaw ENU (mismo frame que /heading/gps en rtk.py)
        gps_ref = (90.0 - track) % 360.0
        target = _wrap180(gps_ref - self.heading_deg)

        if self._gps_offset is None:
            # Warm-up: no fijar el offset en la primera muestra (el COG al arrancar
            # es transitorio y deja un snap corrido). Promediar (media circular)
            # init_samples muestras consecutivas en movimiento -> snap robusto.
            self._init_sin += math.sin(math.radians(target))
            self._init_cos += math.cos(math.radians(target))
            self._init_n += 1
            if self._init_n >= self.init_samples:
                self._gps_offset = math.degrees(math.atan2(self._init_sin, self._init_cos))
                self.get_logger().info(
                    f'fusion GPS: offset inicial={self._gps_offset:.1f} deg '
                    f'(media de {self._init_n} muestras)')
        else:
            err = _wrap180(target - self._gps_offset)
            self._gps_offset = _wrap180(self._gps_offset + self.offset_alpha * err)
            self._track_convergence(err)

    def _track_convergence(self, err):
        """Marca el heading fused como convergido tras N fixes rectos con residual
        chico. Publica /heading/fused_ready=true (latched) una sola vez -> el
        controller lo espera antes de terminar la pre-alineacion y navegar."""
        if abs(err) <= self.converge_tol_deg:
            self._converge_count += 1
        else:
            self._converge_count = 0
        if not self._fused_ready and self._converge_count >= self.converge_min_samples:
            self._fused_ready = True
            self.fused_ready_pub.publish(Bool(data=True))
            self.get_logger().info(
                f'fusion GPS: heading fused CONVERGIDO '
                f'(|residual|<{self.converge_tol_deg:.0f} deg x{self.converge_min_samples} fixes) '
                f'-> {self.fused_ready_topic}=true')

    def _calibrate_cb(self, msg: Empty):
        """Recalibracion on-demand: reinicia el auto-bias y el estado de fusion para
        una corrida fresca (robot recien parado y quieto). Mientras recalibra, deja
        de publicar /heading/gyro (el _imu_callback vuelve en _collect_bias) -> el
        controller lo detecta como 'no calibrado' hasta que reaparece.
        """
        self.get_logger().info('Recalibracion solicitada — reiniciando auto-bias y fusion GPS')
        # Reiniciar auto-bias
        self._calibrating = True
        self._acc = [0.0, 0.0, 0.0]
        self._acc_sq_z = 0.0
        self._n = 0
        self._last_time = None
        # Reiniciar integracion de heading y fusion (arranque limpio)
        self.heading_deg = self._heading_initial
        self._gps_offset = None
        self._init_sin = 0.0
        self._init_cos = 0.0
        self._init_n = 0
        self._converge_count = 0
        # Bajar la senial de convergencia: el creep de pre-alineacion debe reesperarla
        if self.gps_fusion and self._fused_ready:
            self._fused_ready = False
            self.fused_ready_pub.publish(Bool(data=False))

    def _collect_bias(self, wx, wy, wz):
        """Promedia el gyro con el robot quieto para estimar el bias.

        Ademas valida que el gyro este VIVO: mide el std del yaw rate en la ventana
        de captura. Un gyro real en reposo tiene ruido (std > 0); un topic equivocado
        (LSM303, sin gyro) o un gyro que no reporta da wz==0.0 exacto (std ~ 0). Si el
        gyro no da señal, NO se completa el auto-bias -> no se publica /heading/gyro
        -> el controller queda bloqueado en vez de navegar con heading muerto.
        """
        mag = math.sqrt(wx * wx + wy * wy + wz * wz)
        if mag > self.stationary_thresh:
            # Se movio: descartar lo acumulado y reintentar desde cero
            if self._n > 0:
                self.get_logger().warn(
                    'movimiento durante auto-bias: reiniciando captura (mantener quieto)',
                    throttle_duration_sec=2.0)
            self._acc = [0.0, 0.0, 0.0]
            self._acc_sq_z = 0.0
            self._n = 0
            return

        self._acc[0] += wx
        self._acc[1] += wy
        self._acc[2] += wz
        self._acc_sq_z += wz * wz
        self._n += 1

        if self._n >= self.auto_samples:
            mean_z = self._acc[2] / self._n
            var_z = max(0.0, self._acc_sq_z / self._n - mean_z * mean_z)
            std_z = math.sqrt(var_z)
            if std_z < self.gyro_alive_min_std:
                # Gyro muerto/congelado (std ~ 0): no completar. Reintentar y avisar
                # fuerte; /heading/gyro NO se publica -> el gate del controller bloquea.
                self.get_logger().error(
                    f'GYRO SIN SEÑAL: std yaw rate={std_z:.2e} < {self.gyro_alive_min_std:.2e} rad/s. '
                    f'El gyro no reporta (revisar {self.imu_topic}: firmware dual / topic / cableado). '
                    f'NO se habilita el heading — robot bloqueado.',
                    throttle_duration_sec=5.0)
                self._acc = [0.0, 0.0, 0.0]
                self._acc_sq_z = 0.0
                self._n = 0
                return
            self.bias = [a / self._n for a in self._acc]
            self._calibrating = False
            self._last_time = None
            self.get_logger().info(
                f'auto-bias OK (n={self._n}, std_z={std_z:.2e} rad/s): gyro_bias='
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
