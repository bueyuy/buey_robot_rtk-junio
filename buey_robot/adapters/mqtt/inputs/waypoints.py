"""Adquisicion de una ruta de waypoints GPS via MQTT.

Sin logica ROS2. Suscribe al topic retenido que publica el dashboard, parsea el
JSON e invoca on_waypoints con la lista de puntos {lat, lon}.

El dashboard publica con retain=True, asi que al conectar llega la ultima ruta
guardada. Payload (JSON):
  { "waypoints_gps": [ {"lat", "lon"}, ... ], "loop": bool }

"loop" es opcional (default false): si es true, el consumidor recorre la ruta en
bucle (al llegar al ultimo waypoint vuelve al primero) en vez de terminar.

lat/lon es la fuente de verdad: el consumidor convierte a x/y localmente con el
origen (BASE) activo, igual que START (ver inputs/positions.py), para que la ruta
caiga en el mismo frame que /odom_filtered.
"""

import json
from typing import Callable

from buey_robot.adapters.mqtt.client import MqttClient


class MqttWaypointsInput:
    """Suscripcion MQTT para la ruta de waypoints GPS.

    Parametros
    ----------
    client : MqttClient
        Instancia del cliente MQTT compartido (de get_client()).
    on_waypoints : Callable[[list, bool], None]
        Callback invocado con (lista de dicts {'lat', 'lon'}, loop) cuando llega/
        actualiza la ruta. NO se invoca si el payload es vacio o invalido (no rompe).
    topic : str
        Topic MQTT (retenido) de la ruta de waypoints.
    """

    def __init__(
        self,
        client: MqttClient,
        on_waypoints: Callable[[list, bool], None],
        topic: str,
    ):
        self._client = client
        self._on_waypoints = on_waypoints
        self._topic = topic

        self._client.subscribe(self._topic, self._on_waypoints_msg)

    # ------------------------------------------------------------------
    # Callbacks paho
    # ------------------------------------------------------------------

    def _on_waypoints_msg(self, client, userdata, msg):
        waypoints, loop = self._parse(msg)
        if waypoints:
            self._on_waypoints(waypoints, loop)

    @staticmethod
    def _parse(msg):
        """Parsea el payload {"waypoints_gps": [{"lat","lon"}, ...], "loop": bool}.

        Devuelve (lista de puntos {'lat','lon'} float, loop bool). Descarta los
        puntos sin lat/lon; loop es false si falta. Lista vacia si el payload es
        vacio/invalido: el caller no llama el callback, asi un retained vacio o mal
        formado no rompe nada.
        """
        try:
            raw = json.loads(msg.payload.decode())
            points = raw.get('waypoints_gps')
            loop = bool(raw.get('loop', False))
            if not isinstance(points, list):
                return [], False
            out = []
            for p in points:
                if not isinstance(p, dict) or 'lat' not in p or 'lon' not in p:
                    continue
                out.append({'lat': float(p['lat']), 'lon': float(p['lon'])})
            return out, loop
        except Exception:
            return [], False
