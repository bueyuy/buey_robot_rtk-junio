"""Adquisicion del disparador de arranque de navegacion (GO) via MQTT.

Sin logica ROS2. Suscribe a un topic de comando e invoca on_start() ante CUALQUIER
mensaje: es un trigger, el payload no importa. El dashboard publica aca (NO retained)
cuando el operador aprieta "GO", una vez cargada la ruta y con el robot en el inicio.

NO retained a proposito: un start retenido se re-dispararia al reconectar y el robot
arrancaria solo. La ruta (bueyuy/waypoints) si es retained; el arranque no.
"""

from typing import Callable

from buey_robot.adapters.mqtt.client import MqttClient


class MqttStartInput:
    """Suscripcion MQTT para el disparador de arranque de navegacion (GO).

    Parametros
    ----------
    client : MqttClient
        Instancia del cliente MQTT compartido (de get_client()).
    on_start : Callable[[], None]
        Callback invocado ante cualquier mensaje en el topic (el GO).
    topic : str
        Topic MQTT del comando de arranque (no retained).
    """

    def __init__(
        self,
        client: MqttClient,
        on_start: Callable[[], None],
        topic: str,
    ):
        self._client = client
        self._on_start = on_start
        self._topic = topic

        self._client.subscribe(self._topic, self._on_start_msg)

    def _on_start_msg(self, client, userdata, msg):
        self._on_start()
