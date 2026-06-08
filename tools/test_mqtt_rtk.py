#!/usr/bin/env python3
"""
Script de prueba para verificar conectividad MQTT.
"""

import rclpy
from rclpy.node import Node
import json
import time
import sys

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False


class MQTTTester(Node):
    def __init__(self, broker_host, broker_port, mode='listen'):
        super().__init__('mqtt_tester')

        if not MQTT_AVAILABLE:
            self.get_logger().error('paho-mqtt no instalado. pip3 install paho-mqtt')
            return

        self.broker_host = broker_host
        self.broker_port = broker_port
        self.mode = mode

        # Crear cliente MQTT
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        self.get_logger().info(f'Conectando a {broker_host}:{broker_port}...')

        try:
            self.mqtt_client.connect(broker_host, broker_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'Error conectando: {str(e)}')

    def on_connect(self, client, userdata, flags, rc):
        """Callback de conexión."""
        if rc == 0:
            self.get_logger().info('Conectado exitosamente a MQTT broker!')

            if self.mode == 'listen':
                # Suscribirse a todos los topics del robot
                client.subscribe('robot/#')
                self.get_logger().info('Escuchando topics robot/*')
            elif self.mode == 'publish-gps':
                self.get_logger().info('Modo publicación GPS activado')
                # Timer para publicar GPS simulado
                self.pub_timer = self.create_timer(1.0, self.publish_fake_gps)
        else:
            self.get_logger().error(f'Conexión fallida con código: {rc}')

    def on_message(self, client, userdata, msg):
        """Callback de mensaje."""
        try:
            payload = json.loads(msg.payload.decode())
            self.get_logger().info(f'[{msg.topic}] {json.dumps(payload, indent=2)}')
        except:
            self.get_logger().info(f'[{msg.topic}] {msg.payload.decode()}')

    def publish_fake_gps(self):
        """Publica GPS falso para pruebas."""
        gps_data = {
            'latitude': 40.123456 + (time.time() % 100) * 0.00001,
            'longitude': -3.654321 + (time.time() % 100) * 0.00001,
            'altitude': 650.5,
            'fix_quality': 4,
            'timestamp': time.time()
        }

        payload = json.dumps(gps_data)
        self.mqtt_client.publish('robot/gps/fix', payload, qos=1)
        self.get_logger().info(f'GPS publicado: lat={gps_data["latitude"]:.6f}')

    def destroy_node(self):
        """Cleanup."""
        if hasattr(self, 'mqtt_client'):
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    if not MQTT_AVAILABLE:
        print('ERROR: paho-mqtt no instalado')
        print('Instalar con: pip3 install paho-mqtt')
        return

    # Argumentos
    broker_host = 'localhost'
    broker_port = 1883
    mode = 'listen'

    if len(sys.argv) >= 2:
        if sys.argv[1] == '--publish-gps':
            mode = 'publish-gps'
        elif sys.argv[1] == '--help':
            print('Uso: ros2 run buey_robot test_mqtt [opciones]')
            print('Opciones:')
            print('  (ninguna)       - Escuchar todos los topics robot/*')
            print('  --publish-gps   - Publicar GPS simulado')
            print('  --broker HOST   - Especificar host del broker (default: localhost)')
            print('  --port PORT     - Especificar puerto (default: 1883)')
            return
        elif sys.argv[1] == '--broker' and len(sys.argv) >= 3:
            broker_host = sys.argv[2]
        elif sys.argv[1] == '--port' and len(sys.argv) >= 3:
            broker_port = int(sys.argv[2])

    rclpy.init(args=args)
    node = MQTTTester(broker_host, broker_port, mode)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
