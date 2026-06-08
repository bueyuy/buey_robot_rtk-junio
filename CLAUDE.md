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
launch/            outdoor_rtk, indoor_zed, motor_gateway, telemetry.
waypoints/         Archivos YAML de waypoints (locales x,y o lat/lon).
docs/              Specs y notas.
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

1. Terminal 1 (siempre encendida): `ros2 launch buey_robot motor_gateway.launch.py`
   Arranca `joystick_controller` y `motor_gateway`. El robot responde al joystick
   remoto (MQTT) o al controller autonomo segun llegue input.

2. Llevar el robot al punto de inicio con joystick.

3. Terminal 2: `ros2 launch buey_robot outdoor_rtk.launch.py` (o `indoor_zed`).
   Arranca el stack autonomo (drivers + odometry + trajectory_controller + pose).
   Publica `/cmd_vel` que el motor_gateway de T1 obedece (a menos que el joystick
   tome prioridad por 600ms).

4. Cuando termina la corrida: Ctrl+C en T2. T1 sigue activa.

5. Joystick lleva el robot de vuelta al inicio. Iterar para tunear params.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select buey_robot --symlink-install
source install/setup.bash
```

## Topics MQTT (lo que la web consume)

| Topic | Publisher | Notas |
|---|---|---|
| `bueyuy/telemetry/json` | `pose_publisher` | pose + 3 headings (zed, imu, gps) |
| `bueyuy/navigation/motors` | `motor_gateway` | `"velL&velR"` al Pico |
| `bueyuy/navigation/cmd_vel` | `motor_gateway` | `"v&w"` debug |
| `bueyuy/navigation/status` | `trajectory_controller` | "Following WP n/m, dist=..." |
| `bueyuy/waypoints` y `waypoints_xy` | `trajectory_controller` | retained, JSON |
| `bueyuy/config` y `config/motors` | `trajectory_controller` | retained, params runtime |
| `rtk/location/json` | `gps_nmea_driver` | solo en outdoor: lat/lon/alt/quality/sats |

`bueyuy/emergency` lo subscribe la **Pico (firmware .ino)** directo, ortogonal al stack ROS2.

## Notas importantes

- `motor_gateway` corre en una terminal aparte permanente; outdoor_rtk e indoor_zed
  NO lo incluyen (evitar conflicto de dos motor_gateway).
- `setup.py` usa `glob('launch/*.py')` y `glob('config/*.yaml')`: agregar archivos
  nuevos no requiere editar el setup.
- Para overrides indoor/outdoor: editar `navigation_indoor.yaml` o
  `navigation_outdoor.yaml`. ROS2 mergea nativamente porque los nodos declaran
  cada param individualmente.
