"""Cliente MQTT singleton: unica fuente de la instancia paho (ningun otro archivo la importa).
Reconecta solo, re-suscribe al reconectar. get_client() devuelve la instancia compartida."""

import threading
import paho.mqtt.client as mqtt

_client_instance = None
_client_lock = threading.Lock()


def get_client(mqtt_cfg: dict, logger=None) -> 'MqttClient':
    """Instancia compartida. La primera llamada conecta con mqtt_cfg; las demas la reusan."""
    global _client_instance
    with _client_lock:
        if _client_instance is None:
            _client_instance = MqttClient(mqtt_cfg, logger=logger)
        return _client_instance


class MqttClient:
    def __init__(self, mqtt_cfg: dict, logger=None):
        self._logger = logger
        self._connected = False
        self._subscriptions = []                 # (topic, callback, qos), re-aplicadas al reconectar

        broker = mqtt_cfg['broker']
        self._host = broker['host']
        self._port = broker['port']
        keepalive = broker.get('keepalive', 60)
        creds = broker.get('credentials', {})
        self._reconnect_interval = mqtt_cfg.get('reconnect', {}).get('interval_sec', 5.0)
        self._reconnect_timer = None

        self._client = mqtt.Client()
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        if creds.get('username') and creds.get('password'):
            self._client.username_pw_set(creds['username'], creds['password'])

        self._log_info(f'MQTT: conectando a {self._host}:{self._port}...')
        try:
            self._client.connect(self._host, self._port, keepalive)
            self._client.loop_start()
        except Exception as e:
            self._log_error(f'MQTT: error conectando: {e}')
            self._schedule_reconnect()

    def publish(self, topic: str, payload, qos: int = 0, retain: bool = False):
        if not self._connected:
            return
        try:
            self._client.publish(topic, payload, qos=qos, retain=retain)
        except Exception as e:
            self._log_error(f'MQTT: error publicando en {topic}: {e}')

    def subscribe(self, topic: str, callback, qos: int = 0):
        self._subscriptions.append((topic, callback, qos))
        if self._connected:
            self._client.subscribe(topic, qos)
            self._client.message_callback_add(topic, callback)

    def shutdown(self):
        if self._reconnect_timer is not None:
            self._reconnect_timer.cancel()
            self._reconnect_timer = None
        try:
            self._client.loop_stop()
            self._client.disconnect()
        except Exception:
            pass

    def _on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            self._connected = False
            self._log_error(f'MQTT: conexion fallida (rc={rc})')
            self._schedule_reconnect()
            return
        self._connected = True
        self._log_info('MQTT: conectado')
        if self._reconnect_timer is not None:
            self._reconnect_timer.cancel()
            self._reconnect_timer = None
        for topic, callback, qos in self._subscriptions:
            client.subscribe(topic, qos)
            client.message_callback_add(topic, callback)

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False
        self._log_warn(f'MQTT: desconectado (rc={rc})')
        if rc != 0:
            self._schedule_reconnect()

    def _schedule_reconnect(self):
        if self._reconnect_timer is not None:
            return
        self._reconnect_timer = threading.Timer(self._reconnect_interval, self._do_reconnect)
        self._reconnect_timer.daemon = True
        self._reconnect_timer.start()

    def _do_reconnect(self):
        self._reconnect_timer = None
        if self._connected:
            return
        try:
            self._client.reconnect()
        except Exception as e:
            self._log_error(f'MQTT: error reconexion: {e}')
            self._schedule_reconnect()

    def _log_info(self, msg):
        if self._logger:
            self._logger.info(msg)

    def _log_warn(self, msg):
        if self._logger:
            self._logger.warn(msg)

    def _log_error(self, msg):
        if self._logger:
            self._logger.error(msg)
