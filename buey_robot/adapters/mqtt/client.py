"""Cliente MQTT singleton para todos los nodos del sistema.

Unica fuente de la instancia paho. Ningun otro archivo importa paho.mqtt.

Uso:
    from buey_robot.adapters.mqtt.client import get_client

    # En el __init__ del nodo:
    mqtt_cfg = load_config('mqtt.yaml')
    self._mqtt = get_client(mqtt_cfg, logger=self.get_logger())
    self._mqtt.publish('mi/topic', 'hola')
    self._mqtt.subscribe('mi/topic/in', mi_callback)

El singleton se crea en la primera llamada a get_client().
Las siguientes llamadas devuelven la misma instancia.

Para tests unitarios: llamar reset_client() antes de cada test.
"""

import threading
import paho.mqtt.client as mqtt

# -----------------------------------------------------------------------
# Singleton global
# -----------------------------------------------------------------------

_client_instance: 'MqttClient | None' = None
_client_lock = threading.Lock()


def get_client(mqtt_cfg: dict, logger=None) -> 'MqttClient':
    """Devuelve el cliente MQTT compartido.

    La primera llamada crea el cliente y conecta al broker usando mqtt_cfg.
    Las siguientes llamadas ignoran mqtt_cfg y devuelven la instancia existente.

    Args:
        mqtt_cfg: Diccionario de mqtt.yaml (necesario solo en la primera llamada).
        logger:   Logger de ROS2 (get_logger()). Opcional; solo se usa en la primera llamada.

    Returns:
        Instancia compartida de MqttClient.
    """
    global _client_instance
    with _client_lock:
        if _client_instance is None:
            _client_instance = MqttClient(mqtt_cfg, logger=logger)
        return _client_instance


def reset_client():
    """Resetea el singleton. Solo para tests unitarios."""
    global _client_instance
    with _client_lock:
        if _client_instance is not None:
            _client_instance.shutdown()
            _client_instance = None


# -----------------------------------------------------------------------
# Clase MqttClient
# -----------------------------------------------------------------------

class MqttClient:
    """Wrapper sobre paho.mqtt.Client.

    Encapsula:
    - Conexion inicial al broker.
    - Logica de reconexion con intervalo configurable (via timer de Python, no ROS2).
    - subscribe / publish / add_message_callback.
    - shutdown limpio.

    No hereda de nada. No sabe nada de ROS2.
    """

    def __init__(self, mqtt_cfg: dict, logger=None):
        self._logger = logger
        self._connected = False
        self._subscriptions: list[tuple[str, object, int]] = []

        broker = mqtt_cfg['broker']
        self._host = broker['host']
        self._port = broker['port']
        keepalive = broker.get('keepalive', 60)
        creds = broker.get('credentials', {})

        reconnect_cfg = mqtt_cfg.get('reconnect', {})
        self._reconnect_interval = reconnect_cfg.get('interval_sec', 5.0)

        self._client = mqtt.Client()
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect

        if creds.get('username') and creds.get('password'):
            self._client.username_pw_set(creds['username'], creds['password'])

        self._reconnect_timer: threading.Timer | None = None

        self._log_info(f'MQTT: conectando a {self._host}:{self._port}...')
        try:
            self._client.connect(self._host, self._port, keepalive)
            self._client.loop_start()
        except Exception as e:
            self._log_error(f'MQTT: error conectando: {e}')
            self._schedule_reconnect()

    # ------------------------------------------------------------------
    # API publica
    # ------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        """True si el cliente esta actualmente conectado al broker."""
        return self._connected

    def publish(self, topic: str, payload, qos: int = 0, retain: bool = False):
        """Publica un mensaje. No hace nada si no hay conexion."""
        if not self._connected:
            return
        try:
            self._client.publish(topic, payload, qos=qos, retain=retain)
        except Exception as e:
            self._log_error(f'MQTT: error publicando en {topic}: {e}')

    def subscribe(self, topic: str, callback, qos: int = 0):
        """Registra suscripcion. Se reactiva en cada reconexion.

        Args:
            topic:    Topic MQTT a suscribir.
            callback: Funcion paho (client, userdata, message).
            qos:      Nivel de QoS (0, 1 o 2).
        """
        self._subscriptions.append((topic, callback, qos))
        if self._connected:
            self._client.subscribe(topic, qos)
            self._client.message_callback_add(topic, callback)

    def add_message_callback(self, topic: str, callback):
        """Agrega callback para un topic ya suscrito (sin re-suscribir).

        Util cuando el cliente llama subscribe() directamente (ej: MqttImuInput).
        """
        self._client.message_callback_add(topic, callback)

    def raw_subscribe(self, topic: str, qos: int = 0):
        """Suscribe sin registrar callback. Usar junto a add_message_callback()."""
        if self._connected:
            self._client.subscribe(topic, qos)
        # Guardar para reconexion sin callback (el caller lo re-registra en on_connect)
        self._subscriptions.append((topic, None, qos))

    def shutdown(self):
        """Detiene el loop y desconecta. Llamar al destruir el proceso."""
        if self._reconnect_timer is not None:
            self._reconnect_timer.cancel()
            self._reconnect_timer = None
        try:
            self._client.loop_stop()
            self._client.disconnect()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Callbacks paho
    # ------------------------------------------------------------------

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._connected = True
            self._log_info('MQTT: conectado')
            if self._reconnect_timer is not None:
                self._reconnect_timer.cancel()
                self._reconnect_timer = None
            # Re-suscribir todo
            for topic, callback, qos in self._subscriptions:
                client.subscribe(topic, qos)
                if callback is not None:
                    client.message_callback_add(topic, callback)
                self._log_info(f'MQTT: suscrito a {topic}')
        else:
            self._connected = False
            self._log_error(f'MQTT: conexion fallida (rc={rc})')
            self._schedule_reconnect()

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        self._log_warn(f'MQTT: desconectado (rc={rc})')
        if rc != 0:
            self._schedule_reconnect()

    # ------------------------------------------------------------------
    # Reconexion con backoff simple
    # ------------------------------------------------------------------

    def _schedule_reconnect(self):
        if self._reconnect_timer is not None:
            return  # ya hay uno programado
        self._reconnect_timer = threading.Timer(
            self._reconnect_interval, self._do_reconnect)
        self._reconnect_timer.daemon = True
        self._reconnect_timer.start()

    def _do_reconnect(self):
        self._reconnect_timer = None
        if self._connected:
            return
        self._log_warn('MQTT: intentando reconexion...')
        try:
            self._client.reconnect()
        except Exception as e:
            self._log_error(f'MQTT: error reconexion: {e}')
            self._schedule_reconnect()

    # ------------------------------------------------------------------
    # Logging (tolerante a logger=None)
    # ------------------------------------------------------------------

    def _log_info(self, msg: str):
        if self._logger:
            self._logger.info(msg)

    def _log_warn(self, msg: str):
        if self._logger:
            self._logger.warn(msg)

    def _log_error(self, msg: str):
        if self._logger:
            self._logger.error(msg)
