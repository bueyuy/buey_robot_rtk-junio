"""Odometria RTK: convierte /gps/fix -> /odom_filtered usando GPSConverter.

Calcula heading desde movimiento GPS y lo fusiona con heading del IMU.
Abstrae la fuente de odometria exactamente igual que odometry/zed.py,
de modo que navigation/controller.py no sabe cual esta corriendo.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from gps_msgs.msg import GPSFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float32
import math

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.adapters.mqtt.inputs.positions import MqttPositionsInput
from buey_robot.utils.config import load_config, require_key
from buey_robot.utils.math import angle_normalize
from buey_robot.utils.filters import MovingAverageFilter, ExponentialFilter
from buey_robot.utils.gps_converter import GPSConverter


class RTKOdometry(Node):
    def __init__(self):
        super().__init__('rtk_odometry')

        # --- Parametros GPS/navegacion (ROS2 estandar, sin defaults) ---
        self.declare_parameter('gps.origin.auto_set', Parameter.Type.BOOL)
        self.declare_parameter('gps.origin.use_mqtt_base', Parameter.Type.BOOL)
        self.declare_parameter('gps.quality.min_satellites', Parameter.Type.INTEGER)
        self.declare_parameter('gps.quality.require_rtk_fix', Parameter.Type.BOOL)
        self.declare_parameter('gps.filter.enabled', Parameter.Type.BOOL)
        self.declare_parameter('gps.filter.window_size', Parameter.Type.INTEGER)
        self.declare_parameter('gps.heading.min_movement_threshold', Parameter.Type.DOUBLE)
        self.declare_parameter('gps.heading.filter_alpha', Parameter.Type.DOUBLE)
        self.declare_parameter('gps.imu.use_imu_heading', Parameter.Type.BOOL)
        self.declare_parameter('gps.imu.fusion_alpha', Parameter.Type.DOUBLE)
        self.declare_parameter('gps.imu.fusion_min_speed', Parameter.Type.DOUBLE)
        self.declare_parameter('gps.imu.magnetic_declination_deg', Parameter.Type.DOUBLE)
        self.declare_parameter('gps.imu.heading_topic', Parameter.Type.STRING)
        self.declare_parameter('gps.frame_id', Parameter.Type.STRING)

        self.auto_set_origin = self.get_parameter('gps.origin.auto_set').value
        self.use_mqtt_base = self.get_parameter('gps.origin.use_mqtt_base').value
        self.min_satellites = self.get_parameter('gps.quality.min_satellites').value
        self.require_rtk = self.get_parameter('gps.quality.require_rtk_fix').value
        self.filter_enabled = self.get_parameter('gps.filter.enabled').value
        self.filter_window = self.get_parameter('gps.filter.window_size').value
        self.min_movement = self.get_parameter('gps.heading.min_movement_threshold').value
        self.heading_alpha = self.get_parameter('gps.heading.filter_alpha').value
        self.use_imu_heading = self.get_parameter('gps.imu.use_imu_heading').value
        self.fusion_alpha = self.get_parameter('gps.imu.fusion_alpha').value
        self.fusion_min_speed = self.get_parameter('gps.imu.fusion_min_speed').value
        self.magnetic_declination = self.get_parameter('gps.imu.magnetic_declination_deg').value
        self.imu_heading_topic = self.get_parameter('gps.imu.heading_topic').value
        self.frame_id = self.get_parameter('gps.frame_id').value

        # Conversor GPS y filtros
        self.gps_converter = GPSConverter()
        self.x_filter = MovingAverageFilter(self.filter_window) if self.filter_enabled else None
        self.y_filter = MovingAverageFilter(self.filter_window) if self.filter_enabled else None
        self.heading_filter = ExponentialFilter(self.heading_alpha)

        # Estado
        self.current_x = None
        self.current_y = None
        self.current_heading = None
        self.imu_heading = None
        self.gps_heading = None
        self.current_speed = 0.0
        self.gps_received = False
        self.imu_heading_received = False

        # Publishers: misma interfaz que odometry/zed.py
        self.odom_filtered_pub = self.create_publisher(Odometry, '/odom_filtered', 10)
        self.heading_pub = self.create_publisher(Float64, '/heading/gps', 10)
        self.heading_imu_pub = self.create_publisher(Float64, '/heading/imu', 10)

        # Subscribers
        self.create_subscription(GPSFix, '/gps/fix', self.gps_callback, 10)
        # IMU desde micro-ROS (firmware ESP32 + LSM303DLH).
        #
        # CAMINO B: consumimos un heading de brujula (std_msgs/Float32, GRADOS) en
        # vez del quaternion de /imu/data_raw (que sale vacio, cov[0]=-1).
        #
        # El topic es configurable (gps.imu.heading_topic). Por defecto apunta a
        # /imu/heading_calibrated, que publica el nodo imu_compass aplicando la
        # calibracion hard/soft-iron sobre /imu/mag (el firmware /imu/heading sale
        # con el offset hard-iron sin corregir y barre solo ~20% de los 360).
        #
        # Siempre nos suscribimos: derivamos el heading ENU para telemetria
        # (/heading/imu que consume pose.py) y, si use_imu_heading, ademas lo
        # fusionamos con el heading GPS en _fuse_heading.
        self.create_subscription(Float32, self.imu_heading_topic, self.imu_callback, 10)
        if self.use_imu_heading:
            self.get_logger().info('RTK Odometry: usando fusion heading GPS+IMU')
        else:
            self.get_logger().info('RTK Odometry: heading solo-GPS (IMU solo para telemetria)')

        # BASE como origen del frame local (via MQTT, retained).
        # Cuando use_mqtt_base=true, el origen ENU lo fija la BASE publicada por
        # el dashboard (repetible entre corridas) en vez del primer fix GPS.
        self._positions_input = None
        if self.use_mqtt_base:
            mqtt_cfg = load_config('mqtt.yaml')
            mqtt = get_client(mqtt_cfg, logger=self.get_logger())
            topics = {
                'base': require_key(mqtt_cfg, 'topics', 'positions_base'),
                'start': require_key(mqtt_cfg, 'topics', 'positions_start'),
            }
            self._positions_input = MqttPositionsInput(
                mqtt, on_base=self._on_base, on_start=lambda d: None, topics=topics)

        self.get_logger().info('RTK Odometry iniciado')
        self.get_logger().info(f'  Min sats: {self.min_satellites}, Require RTK: {self.require_rtk}')
        if self.use_mqtt_base:
            self.get_logger().info('  Origen: BASE MQTT (esperando bueyuy/positions/base)')

    def _on_base(self, base: dict):
        """Fija el origen ENU desde la BASE publicada por el dashboard."""
        self.gps_converter.set_origin(base['lat'], base['lon'])
        self.gps_received = True
        self.get_logger().info(
            f"Origen GPS desde BASE: lat={base['lat']:.8f}, lon={base['lon']:.8f}")

    def gps_callback(self, msg: GPSFix):
        """Callback GPS: convierte a ENU, calcula heading, publica /odom_filtered."""
        # Verificar calidad — loguear rechazos para no quedar mudos en testing
        if self.require_rtk and msg.status.status < 2:
            self.get_logger().warn(
                f'GPS fix rechazado: status={msg.status.status} (requiere RTK, status>=2)',
                throttle_duration_sec=5.0,
            )
            return

        # Establecer origen. Con use_mqtt_base, el origen lo fija _on_base desde
        # la BASE retenida; NO auto-fijar desde el primer fix (esperar la BASE).
        if not self.gps_converter.origin_set:
            if self.use_mqtt_base:
                self.get_logger().warn(
                    'Esperando BASE (bueyuy/positions/base) para fijar origen...',
                    throttle_duration_sec=5.0,
                )
                return
            if self.auto_set_origin:
                self.gps_converter.set_origin(msg.latitude, msg.longitude)
                self.get_logger().info(
                    f'Origen GPS establecido: lat={msg.latitude:.8f}, lon={msg.longitude:.8f}')
                self.gps_received = True

        if not self.gps_converter.origin_set:
            return

        x, y = self.gps_converter.gps_to_local(msg.latitude, msg.longitude)

        if self.filter_enabled and self.x_filter and self.y_filter:
            x = self.x_filter.update(x)
            y = self.y_filter.update(y)

        self.current_x = x
        self.current_y = y

        self._update_heading(msg.speed, msg.track)
        self._publish_odometry(msg)

    def imu_callback(self, msg: Float32):
        """Callback heading IMU desde /imu/heading (Float32, grados de brujula).

        Convierte la convencion de brujula a yaw ENU, que es el marco en que
        trabaja todo rtk.py (igual que el heading GPS), para poder fusionarlos:

          - Brujula (lo que da el firmware): 0 = Norte magnetico, sentido HORARIO.
          - Yaw ENU (lo que usa rtk.py):     0 = Este, sentido ANTIHORARIO,
                                             referido al Norte GEOGRAFICO.

        Conversion:
            yaw_enu = 90 - heading_brujula - declinacion_magnetica

        El termino de declinacion pasa de Norte magnetico a geografico (en Uruguay
        es negativa, ~-8.5 grados = Oeste). Ademas ABSORBE cualquier offset
        constante por el montaje de la IMU en el robot, asi que en la practica se
        calibra empiricamente: con el robot andando en recta, ajustar
        magnetic_declination_deg hasta que /heading/imu coincida con /heading/gps.
        """
        yaw_enu = angle_normalize(
            math.radians(90.0 - msg.data - self.magnetic_declination)
        )

        # Siempre reenviar heading IMU para pose.py (en grados ENU, comparable
        # directamente con /heading/gps para tunear la declinacion).
        heading_msg = Float64()
        heading_msg.data = math.degrees(yaw_enu)
        self.heading_imu_pub.publish(heading_msg)

        # Solo fusionar en la odometria si use_imu_heading; si no, heading solo-GPS
        if self.use_imu_heading:
            self.imu_heading = yaw_enu
            self.imu_heading_received = True
            self._fuse_heading()

    def _update_heading(self, speed: float, track: float):
        """Actualiza heading GPS usando speed (m/s) y track/COG (grados) del GPSFix.

        track viene en convencion brujula (0=Norte, horario). Lo convertimos
        a yaw ENU (0=Este, antihorario) para ser consistente con el resto del sistema.
        """
        self.current_speed = speed

        if self.current_speed > self.min_movement:
            # COG brujula -> yaw ENU: yaw = 90 - track
            raw_gps_heading = angle_normalize(math.radians(90.0 - track))
            self.gps_heading = self.heading_filter.update(raw_gps_heading)

            heading_msg = Float64()
            heading_msg.data = math.degrees(self.gps_heading)
            self.heading_pub.publish(heading_msg)

        self._fuse_heading()

    def _fuse_heading(self):
        """Fusiona heading GPS+IMU con filtro complementario.

        Logica:
        - Parado o sin GPS heading: usa solo IMU.
        - Sin IMU: usa solo GPS.
        - En movimiento con ambos: alpha * IMU + (1-alpha) * GPS.
        """
        if self.imu_heading is not None and self.gps_heading is not None:
            if self.current_speed >= self.fusion_min_speed:
                diff = math.atan2(
                    math.sin(self.imu_heading - self.gps_heading),
                    math.cos(self.imu_heading - self.gps_heading)
                )
                self.current_heading = angle_normalize(
                    self.gps_heading + self.fusion_alpha * diff
                )
            else:
                self.current_heading = self.imu_heading
        elif self.imu_heading is not None:
            self.current_heading = self.imu_heading
        elif self.gps_heading is not None:
            self.current_heading = self.gps_heading

    def _publish_odometry(self, gps_msg: GPSFix):
        """Publica nav_msgs/Odometry a /odom_filtered."""
        if self.current_x is None:
            return
        if self.current_heading is None:
            # Sin heading todavia, usar 0
            heading = 0.0
        else:
            heading = self.current_heading

        msg = Odometry()
        msg.header.stamp = gps_msg.header.stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = self.frame_id

        msg.pose.pose.position.x = self.current_x
        msg.pose.pose.position.y = self.current_y
        msg.pose.pose.position.z = gps_msg.altitude if gps_msg.altitude else 0.0

        # Orientacion desde heading (yaw)
        msg.pose.pose.orientation.z = math.sin(heading / 2.0)
        msg.pose.pose.orientation.w = math.cos(heading / 2.0)

        # Velocidad aproximada
        msg.twist.twist.linear.x = self.current_speed

        self.odom_filtered_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RTKOdometry()
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
