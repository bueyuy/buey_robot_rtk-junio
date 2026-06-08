"""Adquisicion de puntos de referencia (BASE/START) via MQTT.

Sin logica ROS2. Suscribe a los topics retenidos que publica el dashboard,
parsea el JSON e invoca on_base / on_start con el dict completo.

El dashboard publica con retain=True, asi que al conectar llega el ultimo
valor guardado. Payload (JSON):
  { "lat", "lon", "alt", "x", "y", "heading",
    "quality", "quality_name", "hdop", "satellites", "timestamp" }

lat/lon es la fuente de verdad: el x/y del payload se calculo con el origen
que tenia el robot al guardar, que puede no coincidir con el origen activo.
Los consumidores deben recalcular x/y localmente desde lat/lon.
"""

import json
from typing import Callable

from buey_robot.adapters.mqtt.client import MqttClient


class MqttPositionsInput:
    """Suscripcion MQTT para los puntos de referencia BASE y START.

    Parametros
    ----------
    client : MqttClient
        Instancia del cliente MQTT compartido (de get_client()).
    on_base : Callable[[dict], None]
        Callback invocado cuando llega/actualiza la BASE.
    on_start : Callable[[dict], None]
        Callback invocado cuando llega/actualiza el START.
    topics : dict
        Mapa nombre->topic. Claves: 'base', 'start'.
    """

    def __init__(
        self,
        client: MqttClient,
        on_base: Callable[[dict], None],
        on_start: Callable[[dict], None],
        topics: dict,
    ):
        self._client = client
        self._on_base = on_base
        self._on_start = on_start
        self._topic_base = topics['base']
        self._topic_start = topics['start']

        self._client.subscribe(self._topic_base, self._on_base_msg)
        self._client.subscribe(self._topic_start, self._on_start_msg)

    # ------------------------------------------------------------------
    # Callbacks paho
    # ------------------------------------------------------------------

    def _on_base_msg(self, client, userdata, msg):
        data = self._parse(msg)
        if data is not None:
            self._on_base(data)

    def _on_start_msg(self, client, userdata, msg):
        data = self._parse(msg)
        if data is not None:
            self._on_start(data)

    @staticmethod
    def _parse(msg):
        """Parsea el payload JSON. Requiere lat/lon. None si invalido."""
        try:
            raw = json.loads(msg.payload.decode())
            if 'lat' not in raw or 'lon' not in raw:
                return None
            raw['lat'] = float(raw['lat'])
            raw['lon'] = float(raw['lon'])
            return raw
        except Exception:
            return None
