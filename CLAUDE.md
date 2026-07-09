# buey_robot — Guia para Claude

## Proyecto

Sistema de navegacion autonoma para robot agricola skid-steer (Buey V).
ROS2 Humble. Dos modos de odometria: **outdoor** (GPS RTK + IMU placa) o **indoor**
(ZED Visual Odometry). MQTT para telemetria al dashboard web y control de motores.

## Estructura

```
buey_robot/
  mapper/          Drivers de sensores fisicos (GPS NMEA, IMU). Exponen topics ROS2 estandar.
  odometry/        Productores de /odom_filtered (zed.py y rtk.py — agnostico para el controller).
  navigation/      controller (ARC + FINAL_APPROACH), joystick, waypoint_manager, ramp_profile.
  motor/           gateway (cmd_vel -> motores via output inyectado), kinematics, filtros.
  adapters/        Capa transport intercambiable: mqtt/ (hoy) y serial/ (futuro).
                   inputs/ = clases de adquisicion; outputs/ = clases o nodos de salida.
  utils/           Utilidades genericas: math, filters, gps_converter, config.
config/            YAML como unica fuente de verdad. Cada nodo declare_parameter sin defaults.
launch/            nav_outdoor (stack completo) y motor_gateway (drive manual).
docs/              Specs y notas.

Los waypoints llegan SIEMPRE por MQTT (bueyuy/waypoints, lat/lon). No hay carga
por archivo YAML.
```

Ver `docs/scaffold.md` para el detalle de archivos y responsabilidades por capa.

## Convenciones

- **Config**: 100% en YAML. Los nodos usan `declare_parameter` con tipo (sin default).
  Si falta una key en el YAML, el nodo falla al arrancar (fail-fast).
  Solo `mqtt.yaml` y `sensors.yaml` se leen con `load_config()` para cosas que NO son
  parametros ROS2 (config del cliente MQTT singleton).

- **MQTT**: la conexion vive en `adapters/mqtt/client.py` como singleton thread-safe.
  Los nodos de `navigation/`, `motor/`, `odometry/`, `mapper/` **NUNCA** importan
  `paho-mqtt` ni `paho`. Si necesitan publicar al broker, lo hacen via la clase
  `Mqtt*Output` (inyectada) o un nodo dedicado en `adapters/mqtt/outputs/`.

- **Patron drivers vs adapters**:
  - `mapper/Y` = sensor logico (publica topic ROS2 estandar)
  - `adapters/<transport>/inputs/Y` = adquisicion: como se trae el dato del medio
  - El mapper instancia el adapter de adquisicion correspondiente y recibe via callback.

- **Idioma**: comentarios y docs en espanol (sin acentos). Nombres en ingles.

## Flujo de trabajo tipico (en el robot)

1. Un solo launch levanta todo: `ros2 launch buey_robot nav_outdoor.launch.py`
   (motor_gateway + sensores + odometria RTK + telemetria + trajectory_controller).

2. Con el joystick remoto (MQTT), llevar el robot al inicio y pararlo (quieto).
   El joystick tiene prioridad sobre la nav por 600ms.

3. Desde la telemetria (dashboard) cargar la ruta (bueyuy/waypoints, lat/lon) y dar
   el GO (bueyuy/navigation/start). La nav es FULL RTK con geolocalizacion dinamica:
   no hay waypoints fijos ni archivos. El origen del frame lo fija rtk (primer fix).

4. Para detener: publicar ruta vacia (idle) o mover el joystick.

(`motor_gateway.launch.py` queda aparte por si se quiere solo drive manual.)

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select buey_robot --symlink-install
source install/setup.bash
```

## Topics MQTT (lo que la web consume)

| Topic | Publisher | Notas |
|---|---|---|
| `bueyuy/telemetry/json` | `pose_publisher` | pose + headings (gyro/fused, gps) |
| `bueyuy/navigation/motors` | `motor_gateway` | `"velL&velR"` al Pico |
| `bueyuy/navigation/cmd_vel` | `motor_gateway` | `"v&w"` debug |
| `bueyuy/navigation/status` | `trajectory_controller` | "Following WP n/m, dist=..." |
| `bueyuy/waypoints` y `waypoints_xy` | `trajectory_controller` | retained, JSON |
| `bueyuy/config` y `config/motors` | `trajectory_controller` | retained, params runtime |
| `rtk/location/json` | `gps_nmea_driver` | solo en outdoor: lat/lon/alt/quality/sats |

`bueyuy/emergency` lo subscribe la **Pico (firmware .ino)** directo, ortogonal al stack ROS2.

## Notas importantes

- `nav_outdoor` ya incluye `motor_gateway`; NO correr ademas `motor_gateway.launch.py`
  aparte (evitar dos motor_gateway).
- `setup.py` usa `glob('launch/*.py')` y `glob('config/*.yaml')`: agregar archivos
  nuevos no requiere editar el setup.
- Para overrides indoor/outdoor: editar `navigation_indoor.yaml` o
  `navigation_outdoor.yaml`. ROS2 mergea nativamente porque los nodos declaran
  cada param individualmente.
