"""Adquisicion del ORIGEN del frame ENU via MQTT.

Sin logica ROS2. Suscribe al topic retenido que publica rtk.py (el primer fix GPS,
donde arranca el robot), parsea el JSON e invoca on_origin con {lat, lon}.

El controller usa este origen para convertir los waypoints lat/lon al MISMO frame
que /odom_filtered. Reemplaza la BASE del dashboard: ahora el origen es automatico.
"""

import json
from typing import Callable

from buey_robot.adapters.mqtt.client import MqttClient


class MqttOriginInput:
    """Suscripcion MQTT para el origen del frame ENU.

    Parametros
    ----------
    client : MqttClient
        Instancia del cliente MQTT compartido (de get_client()).
    on_origin : Callable[[dict], None]
        Callback invocado con {'lat', 'lon'} cuando llega/actualiza el origen.
    topic : str
        Topic MQTT (retenido) del origen.
    """

    def __init__(
        self,
        client: MqttClient,
        on_origin: Callable[[dict], None],
        topic: str,
    ):
        self._client = client
        self._on_origin = on_origin
        self._topic = topic

        self._client.subscribe(self._topic, self._on_origin_msg)

    def _on_origin_msg(self, client, userdata, msg):
        data = self._parse(msg)
        if data is not None:
            self._on_origin(data)

    @staticmethod
    def _parse(msg):
        """Parsea el payload {lat, lon}. None si invalido."""
        try:
            raw = json.loads(msg.payload.decode())
            if 'lat' not in raw or 'lon' not in raw:
                return None
            return {'lat': float(raw['lat']), 'lon': float(raw['lon'])}
        except Exception:
            return None
