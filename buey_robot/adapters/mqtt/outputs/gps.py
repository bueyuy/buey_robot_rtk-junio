"""MqttGpsOutput: publica datos GPS por MQTT en formato JSON.

Clase reutilizable (no nodo). GPSNmeaDriver la instancia y llama .send().
Topic publicado:
  - rtk/location/json  (JSON consolidado con todo el estado GPS)

Solo se activa en outdoor_rtk launch. En indoor_zed no se instancia.
"""

import json
import time

from buey_robot.adapters.mqtt.client import MqttClient
from buey_robot.utils.config import require_key


QUALITY_NAMES = {
    0: 'No Fix', 1: 'GPS', 2: 'DGPS', 3: 'PPS',
    4: 'RTK Fixed', 5: 'RTK Float', 6: 'Estimated',
}


class MqttGpsOutput:
    """Publica datos GPS por MQTT como JSON consolidado."""

    def __init__(self, client: MqttClient, mqtt_cfg: dict):
        """
        Args:
            client:   Instancia compartida de MqttClient (de get_client()).
            mqtt_cfg: Diccionario de mqtt.yaml.
        """
        self._client = client
        self.topic_gps_json = require_key(mqtt_cfg, 'topics', 'gps_json')

    def send(self, lat, lon, alt: float, quality: int, satellites: int,
             hdop: float = 99.9, cog=None, speed_knots=None):
        """Publica datos GPS como JSON en rtk/location/json — todo el estado, sin filtros.

        Args:
            lat: Latitud en grados decimales, o None si todavia no hay fix.
            lon: Longitud en grados decimales, o None si todavia no hay fix.
            alt: Altitud en metros.
            quality: Calidad GPS (0=no fix, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float).
            satellites: Numero de satelites.
            hdop: Horizontal dilution of precision.
            cog: Course Over Ground en grados (0-360), o None.
            speed_knots: Velocidad en nudos, o None.
        """
        payload = json.dumps({
            'latitude': round(lat, 8) if lat is not None else None,
            'longitude': round(lon, 8) if lon is not None else None,
            'altitude': round(alt, 2),
            'quality': quality,
            'quality_name': QUALITY_NAMES.get(quality, f'Q{quality}'),
            'satellites': satellites,
            'hdop': round(hdop, 2),
            'cog': round(cog, 1) if cog is not None else None,
            'speed_knots': round(speed_knots, 2) if speed_knots is not None else None,
            'fix_valid': lat is not None and lon is not None and quality > 0,
            'timestamp': time.time(),
        })
        self._client.publish(self.topic_gps_json, payload)
