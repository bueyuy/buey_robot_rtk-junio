"""GPS NMEA driver: publica /gps/fix (GPSFix), /gps/heading (Float32, solo cuando el
COG es confiable) y /gps/status."""

import json

import rclpy
from rclpy.node import Node
from gps_msgs.msg import GPSFix, GPSStatus
from std_msgs.msg import String, Float32

from buey_robot.adapters.serial.serial_lines import SerialLineReader
from buey_robot.drivers.nmea_parser import NmeaParser
from buey_robot.contracts import GPS_FIX, GPS_HEADING, GPS_STATUS
from buey_robot.utils.params import load_params

_KNOTS_TO_MS = 0.514444

PARAMS = {
    'port': ('gps.serial.port', str),
    'baud': ('gps.serial.baudrate', int),
    'timeout': ('gps.serial.timeout', float),
    'min_satellites': ('gps.min_satellites', int),
    'require_rtk': ('gps.require_rtk_fix', bool),
    'frame_id': ('gps.frame_id', str),
    'heading_min_speed': ('gps.heading.min_speed', float),
    'heading_require_rtk': ('gps.heading.require_rtk', bool),
}

QUALITY_NAMES = {0: 'No Fix', 1: 'GPS', 2: 'DGPS', 4: 'RTK Fixed', 5: 'RTK Float'}


class GPSNmeaDriver(Node):
    def __init__(self):
        super().__init__('gps_nmea')

        load_params(self, PARAMS)
        self._last_quality = None       # para loguear cambios de calidad

        self._gps_pub = self.create_publisher(GPSFix, GPS_FIX, 10)
        self._heading_pub = self.create_publisher(Float32, GPS_HEADING, 10)
        self._status_pub = self.create_publisher(String, GPS_STATUS, 10)

        self._parser = NmeaParser()
        self._serial_reader = SerialLineReader(
            port=self.port, baud=self.baud, on_line=self._on_line, timeout=self.timeout,
            logger=self.get_logger())

        self.get_logger().info(f'GPS NMEA Driver iniciado: {self.port} @ {self.baud}')

    def _on_line(self, sentence: str):
        fix = self._parser.feed(sentence)
        if fix is not None:
            self._on_fix(fix)

    @staticmethod
    def _accuracy(quality: int, hdop: float) -> float:
        # Accuracy horizontal (m) por calidad. q4=RTK Fixed (cm) y q5=RTK Float (dm)
        # son DISTINTOS -> testear q==4 antes que >=4 (si no el Float hereda el Fixed).
        if quality == 4:
            return 0.02
        if quality == 5:
            return 0.05
        if quality >= 2:
            return 0.5
        if quality >= 1:
            return min(hdop * 5.0, 10.0)
        return 10.0

    @staticmethod
    def _status(quality: int) -> int:
        if quality >= 4:
            return GPSStatus.STATUS_DGPS_FIX
        if quality >= 1:
            return GPSStatus.STATUS_FIX
        return GPSStatus.STATUS_NO_FIX

    def _log_quality(self, data: dict):
        if data['quality'] == self._last_quality:
            return
        prev = self._last_quality
        old = QUALITY_NAMES.get(prev, f'Q{prev}') if prev is not None else 'inicial'
        new = QUALITY_NAMES.get(data['quality'], f'Q{data["quality"]}')
        self.get_logger().info(f'GPS calidad: {old} -> {new} (sats={data["satellites"]})')
        self._last_quality = data['quality']

    def _publish_gps_heading(self, data: dict):
        """Publica /gps/heading (COG -> yaw ENU) SOLO cuando es confiable: en movimiento
        (el COG es basura quieto) y, si require_rtk, con fix RTK. Si no, no publica."""
        cog, speed_knots = data.get('cog'), data.get('speed_knots')
        if cog is None or speed_knots is None:
            return
        if speed_knots * _KNOTS_TO_MS < self.heading_min_speed:
            return
        if self.heading_require_rtk and data['quality'] < 4:
            return
        self._heading_pub.publish(Float32(data=float((90.0 - cog) % 360.0)))

    def _on_fix(self, data: dict):
        """Recibe un fix del parser, aplica gates de calidad y publica /gps/fix + /gps/status."""
        self._log_quality(data)
        self._publish_gps_heading(data)

        if data['lat'] is None or data['lon'] is None:
            self.get_logger().warn(
                f'Sin coordenadas todavia (quality={data["quality"]}, sats={data["satellites"]})',
                throttle_duration_sec=5.0)
            return
        if data['satellites'] < self.min_satellites:
            self.get_logger().warn(
                f'/gps/fix no publicado: sats={data["satellites"]} < min={self.min_satellites}',
                throttle_duration_sec=5.0)
            return
        if self.require_rtk and data['quality'] < 4:
            self.get_logger().warn(
                f'/gps/fix no publicado: quality={data["quality"]} (requiere RTK Fixed=4)',
                throttle_duration_sec=5.0)
            return

        quality = data['quality']
        speed_knots = data.get('speed_knots')
        accuracy = self._accuracy(quality, data['hdop'])

        msg = GPSFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.status.status = self._status(quality)
        msg.latitude = data['lat']
        msg.longitude = data['lon']
        msg.altitude = data['alt']
        msg.speed = speed_knots * _KNOTS_TO_MS if speed_knots is not None else 0.0
        msg.track = data['cog'] if data.get('cog') is not None else 0.0
        msg.hdop = data['hdop']
        msg.position_covariance[0] = accuracy ** 2
        msg.position_covariance[4] = accuracy ** 2
        msg.position_covariance[8] = (accuracy * 2) ** 2
        msg.position_covariance_type = GPSFix.COVARIANCE_TYPE_APPROXIMATED
        self._gps_pub.publish(msg)

        self._status_pub.publish(String(data=json.dumps({
            'quality': quality, 'sats': data['satellites'],
            'hdop': round(data['hdop'], 2), 'accuracy': round(accuracy, 3),
            'course': round(msg.track, 1), 'speed': round(msg.speed, 2),
        })))

    def destroy_node(self):
        self._serial_reader.stop()
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
