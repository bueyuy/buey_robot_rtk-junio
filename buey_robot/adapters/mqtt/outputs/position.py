"""MqttPositionOutput: publica la posicion del CENTRO del robot en lat/lon a MQTT.

El centro ya viene corregido por el lever-arm de la antena (rtk.py traslada la
posicion GPS de la antena al centro). Se publica en lat/lon para que el mapa de la
telemetria dibuje EL CENTRO — esquinas limpias en los giros — en vez de la antena
cruda (rtk/location/json), que barre y deja "narices" al girar en el lugar.

Es un output de la capa transport (adapters/mqtt/outputs/): odometry/rtk.py no
importa paho, publica via esta clase inyectada.
"""

import json

from buey_robot.adapters.mqtt.client import MqttClient
from buey_robot.utils.config import require_key


class MqttPositionOutput:
    def __init__(self, client: MqttClient, mqtt_cfg: dict):
        self._client = client
        self.topic = require_key(mqtt_cfg, 'topics', 'position_robot')

    def send(self, lat: float, lon: float):
        """Publica {lat, lon} del centro del robot (grados decimales)."""
        payload = json.dumps({'lat': round(lat, 8), 'lon': round(lon, 8)})
        self._client.publish(self.topic, payload)
