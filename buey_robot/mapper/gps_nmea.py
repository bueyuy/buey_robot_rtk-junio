"""GPS NMEA driver: nodo ROS2 que publica /gps/fix (GPSFix).

Recibe datos del adapter de adquisicion (SerialNmeaInput) via callback.
El transport es invisible para este nodo — si manana el GPS llega por
otro medio, se instancia otro adapter con la misma firma.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from gps_msgs.msg import GPSFix, GPSStatus
from std_msgs.msg import String, Float32

from buey_robot.adapters.mqtt.client import get_client
from buey_robot.adapters.mqtt.outputs.gps import MqttGpsOutput
from buey_robot.adapters.serial.inputs.nmea import SerialNmeaInput
from buey_robot.utils.config import load_config

# Metros por grado de latitud (equirectangular local; para el lever-arm de antena).
_M_PER_DEG = 111320.0


class GPSNmeaDriver(Node):
    def __init__(self):
        super().__init__('gps_nmea_driver')

        # Parametros del driver GPS (fail-fast si sensors.yaml no los provee)
        self.declare_parameter('gps.serial.port', Parameter.Type.STRING)
        self.declare_parameter('gps.serial.baudrate', Parameter.Type.INTEGER)
        self.declare_parameter('gps.serial.timeout', Parameter.Type.DOUBLE)
        self.declare_parameter('gps.min_satellites', Parameter.Type.INTEGER)
        self.declare_parameter('gps.require_rtk_fix', Parameter.Type.BOOL)
        self.declare_parameter('gps.frame_id', Parameter.Type.STRING)
        # Lever-arm: la antena esta offset_x_m ADELANTE del centro del robot (trompa).
        # El driver traslada la posicion al CENTRO acá (con el heading), asi TODO aguas
        # abajo (/gps/fix, rtk/location/json, /odom_filtered) usa una UNICA ubicacion
        # ya con offset -> no hay una cruda y una corregida. Vive en robot.yaml
        # (propiedad fisica de montaje del robot).
        self.declare_parameter('robot.antenna.offset_x_m', Parameter.Type.DOUBLE)

        port = self.get_parameter('gps.serial.port').value
        baud = self.get_parameter('gps.serial.baudrate').value
        timeout = self.get_parameter('gps.serial.timeout').value
        self._min_satellites = self.get_parameter('gps.min_satellites').value
        self._require_rtk = self.get_parameter('gps.require_rtk_fix').value
        self._frame_id = self.get_parameter('gps.frame_id').value
        self._antenna_offset = self.get_parameter('robot.antenna.offset_x_m').value

        # Heading (yaw ENU, grados) para orientar el lever-arm. Lo publica mpu6050_gyro
        # (/heading/fused). None hasta el primer heading -> mientras, posicion sin offset.
        self._heading_rad = None

        # Publisher ROS2
        self._gps_pub = self.create_publisher(GPSFix, '/gps/fix', 10)
        self._status_pub = self.create_publisher(String, '/gps/status', 10)
        self.create_subscription(Float32, '/heading/fused', self._on_heading, 10)

        # Contadores
        self._messages_published = 0
        self._last_quality = None       # para loguear cambios de calidad

        # Cliente MQTT compartido (singleton) — para output de telemetria
        mqtt_cfg = load_config('mqtt.yaml')
        self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())

        # Output MQTT: rtk/location/json (posicion centro + calidad)
        self._gps_output = MqttGpsOutput(self._mqtt, mqtt_cfg)

        # Adapter de adquisicion: lee serial, parsea NMEA, llama _on_fix
        self._nmea_input = SerialNmeaInput(
            port=port,
            baud=baud,
            on_fix=self._on_fix,
            timeout=timeout,
            logger=self.get_logger(),
        )

        self.get_logger().info('GPS NMEA Driver iniciado')
        self.get_logger().info(f'  Puerto: {port}, Baudrate: {baud}')

    # ------------------------------------------------------------------
    # Callback del adapter
    # ------------------------------------------------------------------

    def _on_heading(self, msg: Float32):
        self._heading_rad = math.radians(msg.data)

    def _center(self, lat, lon):
        """Traslada la posicion de la ANTENA al CENTRO del robot (lever-arm). La antena
        esta offset ADELANTE (eje X del cuerpo), asi que el centro queda offset atras en
        la direccion del heading. Sin heading todavia (o sin lat/lon), devuelve crudo."""
        if lat is None or lon is None or self._antenna_offset == 0.0 or self._heading_rad is None:
            return lat, lon
        h = self._heading_rad
        d_east = -self._antenna_offset * math.cos(h)   # metros
        d_north = -self._antenna_offset * math.sin(h)
        clat = lat + d_north / _M_PER_DEG
        clon = lon + d_east / (_M_PER_DEG * math.cos(math.radians(lat)))
        return clat, clon

    def _on_fix(self, data: dict):
        """Recibe un fix GPS del adapter y publica /gps/fix (ya con offset de antena).

        data contiene: lat, lon, alt, quality, satellites, hdop, speed_knots, cog
        """
        # Loguear transiciones de calidad (No Fix -> GPS -> DGPS -> RTK Float -> RTK Fixed)
        if data['quality'] != self._last_quality:
            quality_names = {
                0: 'No Fix', 1: 'GPS', 2: 'DGPS', 4: 'RTK Fixed', 5: 'RTK Float',
            }
            old = quality_names.get(self._last_quality, f'Q{self._last_quality}') if self._last_quality is not None else 'inicial'
            new = quality_names.get(data['quality'], f'Q{data["quality"]}')
            self.get_logger().info(f'GPS calidad: {old} -> {new} (sats={data["satellites"]})')
            self._last_quality = data['quality']

        # UNICA ubicacion del robot: trasladar la antena al centro (lever-arm). Se usa
        # tanto para rtk/location/json (mapa) como para /gps/fix (odometria/nav).
        clat, clon = self._center(data['lat'], data['lon'])

        # MQTT: enviar cada fix que llega del hardware (centro, sin filtros de calidad)
        self._gps_output.send(
            clat, clon,
            data['alt'], data['quality'], data['satellites'],
            data.get('hdop', 99.9),
            cog=data.get('cog'),
            speed_knots=data.get('speed_knots'),
        )

        # /gps/fix ROS necesita coordenadas validas
        if data['lat'] is None or data['lon'] is None:
            self.get_logger().warn(
                f'Sin coordenadas todavia (quality={data["quality"]}, sats={data["satellites"]})',
                throttle_duration_sec=5.0,
            )
            return

        # Filtros de calidad solo para /gps/fix ROS (rtk_odometry depende de esto)
        if data['satellites'] < self._min_satellites:
            self.get_logger().warn(
                f'GPS fix no publicado a /gps/fix: sats={data["satellites"]} < min={self._min_satellites}',
                throttle_duration_sec=5.0,
            )
            return
        if self._require_rtk and data['quality'] < 4:
            self.get_logger().warn(
                f'GPS fix no publicado a /gps/fix: quality={data["quality"]} (requiere RTK Fixed=4)',
                throttle_duration_sec=5.0,
            )
            return

        quality = data['quality']

        gps_msg = GPSFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = self._frame_id

        # Status
        if quality >= 4:
            gps_msg.status.status = GPSStatus.STATUS_DGPS_FIX
        elif quality >= 1:
            gps_msg.status.status = GPSStatus.STATUS_FIX
        else:
            gps_msg.status.status = GPSStatus.STATUS_NO_FIX

        # Posicion (centro del robot, ya con lever-arm de antena)
        gps_msg.latitude = clat
        gps_msg.longitude = clon
        gps_msg.altitude = data['alt']

        # Velocidad (nudos -> m/s) y curso
        speed_knots = data.get('speed_knots')
        gps_msg.speed = speed_knots * 0.514444 if speed_knots is not None else 0.0
        cog = data.get('cog')
        gps_msg.track = cog if cog is not None else 0.0

        # DOP
        gps_msg.hdop = data.get('hdop', 99.9)

        # Covarianza de posicion (accuracy horizontal, m). OJO: q4=RTK Fixed (cm) y
        # q5=RTK Float (decimetro) son DISTINTOS -> hay que testear q==4 antes que
        # >=4, si no el Float hereda la accuracy del Fixed (bug: q5 nunca entraba).
        if quality == 4:        # RTK Fixed
            accuracy = 0.02
        elif quality == 5:      # RTK Float
            accuracy = 0.05
        elif quality >= 2:      # DGPS
            accuracy = 0.5
        elif quality >= 1:      # GPS single
            accuracy = min(data['hdop'] * 5.0, 10.0)
        else:
            accuracy = 10.0

        gps_msg.position_covariance[0] = accuracy ** 2
        gps_msg.position_covariance[4] = accuracy ** 2
        gps_msg.position_covariance[8] = (accuracy * 2) ** 2
        gps_msg.position_covariance_type = GPSFix.COVARIANCE_TYPE_APPROXIMATED

        self._gps_pub.publish(gps_msg)
        self._messages_published += 1

        if self._messages_published == 1 or self._messages_published % 10 == 0:
            quality_names = {
                0: 'No Fix', 1: 'GPS', 2: 'DGPS', 4: 'RTK Fixed', 5: 'RTK Float',
            }
            quality_str = quality_names.get(quality, f'Q{quality}')
            self.get_logger().info(
                f'GPS: lat={data["lat"]:.8f}, lon={data["lon"]:.8f}, '
                f'alt={data["alt"]:.2f}m, {quality_str}, '
                f'sats={data["satellites"]} -> MQTT {self._gps_output.topic_gps_json} '
                f'[{self._messages_published} msgs]'
            )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._nmea_input.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSNmeaDriver()
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
