# buey_robot — Navegacion autonoma del robot Buey V

ROS2 Humble. Skid-steer agricola con navegacion FULL RTK: sigue una ruta de waypoints
(lat/lon, en vivo por MQTT) usando GPS RTK + IMU, en modo stop_and_turn.

- **Arquitectura** (capas, nodos, topics, diagramas): [docs/architecture.md](docs/architecture.md)
- **Convenciones** (reglas de diseño para devs/agentes): [docs/CONVENTIONS.md](docs/CONVENTIONS.md)

## Flujo en una linea

sensores → fusion de heading → odometria (GPS→x/y) → controller → gateway → motores.
La ruta y el GO entran por MQTT via `CommandBridge`; la telemetria sale por `TelemetryBridge`.

## Requisitos

- ROS2 Humble + `gps_msgs`.
- Python: `paho-mqtt`, `pyserial`, `pyyaml`.
- Broker MQTT accesible (host/port en `config/mqtt.yaml`).

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select buey_robot --symlink-install
source install/setup.bash
```

## Uso

```bash
# Stack completo (sensores + odometria + navegacion + motores + bridges MQTT)
ros2 launch buey_robot nav_outdoor.launch.py

# Solo drive manual (joystick + gateway)
ros2 launch buey_robot motor_gateway.launch.py
```

Desde la web: cargar la ruta (`bueyuy/waypoints`, lat/lon) y dar el GO
(`bueyuy/navigation/start`). El robot recalibra el gyro, se alinea avanzando recto y navega.
El joystick (`bueyuy/navigation/joystick`) tiene prioridad sobre la navegacion.

## Config

100% en YAML bajo `config/` (espeja la estructura de `buey_robot/`). Cada nodo declara sus
parametros sin default: si falta una key, el nodo falla al arrancar (fail-fast). Ver la regla
en [CONVENTIONS.md](docs/CONVENTIONS.md).
