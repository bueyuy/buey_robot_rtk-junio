# Motor Gateway — Plan futuro: systemd + hot-reload

## Contexto

`motor_gateway` es el unico nodo que debe estar **siempre activo**, independiente de la ZED o navegacion.
Hoy se lanza manualmente con `ros2 launch buey_robot motor_gateway.launch.py`.
Este documento describe el plan para convertirlo en un servicio systemd con hot-reload via MQTT.

## Fase 1: Servicio systemd

```ini
# /etc/systemd/system/motor-gateway.service
[Unit]
Description=Motor Gateway ROS2 Node
After=network.target mosquitto.service

[Service]
Type=simple
User=buey
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch buey_robot motor_gateway.launch.py"
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable motor-gateway
sudo systemctl start motor-gateway
```

## Fase 2: Hot-reload via MQTT

Permitir cambiar parametros del motor gateway (gains, deadzone, etc.) sin reiniciar el nodo.

### Propuesta

1. motor_gateway se suscribe a `bueyuy/config/motor` (retained)
2. Cuando recibe un mensaje, actualiza los parametros internos en caliente
3. La web puede publicar cambios desde el panel de configuracion
4. El nodo confirma el cambio publicando en `bueyuy/config/motor/ack`

### Parametros hot-reloadable

- `linear_gain`, `angular_gain`
- `soft_deadzone.low`, `soft_deadzone.high`
- `max_output`
- `joystick.max_speed`, `joystick.angular_scale`

### Parametros que requieren reinicio

- `wheel_separation` (afecta cinematica fundamental)
- `vel_normalization`

## Fase 3: Watchdog mejorado

- systemd `WatchdogSec=10` + `sd_notify` desde el nodo
- Si el nodo no hace notify en 10s, systemd lo reinicia automaticamente
