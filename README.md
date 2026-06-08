# buey_robot — Navegacion autonoma del robot Buey V

ROS2 Humble. Robot agricola skid-steer con dos modos de odometria:

- **Outdoor**: GPS RTK serial NMEA + IMU placa (via MQTT) + cinematica diferencial.
- **Indoor**: ZED X Visual-Inertial Odometry + cinematica diferencial.

Telemetria al dashboard web y control de motores por MQTT.

## Arquitectura

```
                 ┌────────────────┐
                 │  MOTORES PICO  │ ← bueyuy/navigation/motors
                 └───────▲────────┘
                         │ MQTT
                 ┌───────┴────────┐    /cmd_vel       ┌────────────────────────┐
                 │ motor_gateway  │←──────────────────│ trajectory_controller  │
                 │   (T1, on)     │←──────────────────│      (T2, on/off)      │
                 │                │  /cmd_vel_joy     │                        │
                 │   joystick     │←────────┐         │ + odometry/zed o rtk   │
                 └────────────────┘         │         │ + drivers (gps, imu)   │
                                            │         │ + pose_publisher       │
                                       MQTT │         └────────────────────────┘
                                  bueyuy/navigation/joystick
                                    (web envia)
```

**Workflow tipico**:
1. Terminal 1: `ros2 launch buey_robot motor_gateway.launch.py` (siempre encendida).
2. Llevar el robot por joystick al punto de inicio.
3. Terminal 2: `ros2 launch buey_robot outdoor_rtk.launch.py` (o `indoor_zed.launch.py`).
4. El robot navega autonomo. El joystick toma prioridad por 600ms si llega input.
5. Ctrl+C en T2 cuando termina la corrida. T1 sigue activa para volver con joystick.

## Estructura

```
buey_robot/
  buey_robot/
    mapper/         drivers de sensores HW
      gps_nmea.py   GPS RTK serial NMEA -> /gps/fix
      imu.py        IMU placa MQTT -> /imu/data, /heading/imu
    odometry/
      zed.py        /zed/zed_node/odom -> /odom_filtered + /heading/zed,imu
      rtk.py        /gps/fix + /heading/imu -> /odom_filtered + /heading/gps
    navigation/
      controller.py    ARC + ALIGN/CRUISE/FINAL_APPROACH (waypoint following)
      joystick.py      MQTT bueyuy/navigation/joystick -> /cmd_vel_joy
      waypoint_manager.py, ramp_profile.py, arc_states.py
    motor/
      gateway.py    /cmd_vel + /cmd_vel_joy -> bueyuy/navigation/motors
      kinematics.py, filters.py
    adapters/mqtt/  client.py + inputs/imu + outputs/{motor, status, ...}/pose
    adapters/serial/inputs/nmea.py
    utils/          math, filters, gps_converter, config
  config/
    navigation.yaml + navigation_indoor.yaml + navigation_outdoor.yaml
    motor.yaml, sensors.yaml, mqtt.yaml, robot.yaml
  launch/
    motor_gateway.launch.py    (T1)
    outdoor_rtk.launch.py      (T2 modo outdoor)
    indoor_zed.launch.py       (T2 modo indoor)
    telemetry.launch.py        (debug, solo pose)
  waypoints/   archivos .yaml
  tools/       record_waypoints, send_waypoints, gps_waypoint_converter, etc.
  docs/        scaffold.md + specs
```

## Requisitos

- ROS2 Humble (`ros-humble-ros-base` minimo).
- Python: `paho-mqtt`, `pyserial`, `pyyaml`.
- ZED X + ZED SDK + ROS2 wrapper (solo para indoor).
- Broker MQTT accesible (default `192.168.90.24:1883`).

```bash
sudo apt install -y python3-pip
pip3 install paho-mqtt pyserial pyyaml
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select buey_robot --symlink-install
source install/setup.bash
```

## Uso

### Drive manual permanente (T1, siempre encendida)
```bash
ros2 launch buey_robot motor_gateway.launch.py
```

### Navegacion outdoor (T2)
```bash
ros2 launch buey_robot outdoor_rtk.launch.py
ros2 launch buey_robot outdoor_rtk.launch.py waypoints_file:=/ruta/a/waypoints.yaml
```

### Navegacion indoor (T2)
```bash
ros2 launch buey_robot indoor_zed.launch.py
```

### Debug solo telemetria
```bash
ros2 launch buey_robot telemetry.launch.py
```

## Configs (YAML)

| Archivo | Contenido |
|---------|-----------|
| `navigation.yaml` | controller + algoritmos + waypoint + joystick + ramps (params comunes) |
| `navigation_indoor.yaml` | override indoor (vacio por defecto, llenar al tunear) |
| `navigation_outdoor.yaml` | override outdoor (vacio por defecto) |
| `motor.yaml` | gains, deadzone, max_output, watchdog, joystick_timeout |
| `sensors.yaml` | GPS serial port/baud, IMU topics MQTT, filtros del sensor |
| `mqtt.yaml` | broker host/port + topic names |
| `robot.yaml` | geometria fisica + frames TF |

**YAML es la unica fuente de verdad.** Los nodos declaran cada parametro con
`declare_parameter` sin defaults — si falta una key, el nodo falla al arrancar.
Para overrides indoor/outdoor, ROS2 mergea nativamente cuando se pasan varios
YAMLs a `parameters=[...]`.

## Topics MQTT (publica el robot)

| Topic | Publisher | Formato |
|-------|-----------|---------|
| `bueyuy/telemetry/json` | `pose_publisher` | JSON consolidado |
| `bueyuy/navigation/motors` | `motor_gateway` | `velL&velR` |
| `bueyuy/navigation/cmd_vel` | `motor_gateway` | `v&w` (debug) |
| `bueyuy/navigation/status` | `trajectory_controller` | string parseable |
| `bueyuy/waypoints` y `waypoints_xy` | `trajectory_controller` | JSON retained |
| `bueyuy/config` y `config/motors` | `trajectory_controller` | JSON retained |
| `rtk/location/json` | `gps_nmea_driver` | JSON (solo outdoor) |

Topic que **consume** el robot: `bueyuy/navigation/joystick` (lo publica la web/app).

`bueyuy/emergency` lo subscribe la **Pico (firmware .ino)** directo del broker.
No pasa por ROS2.

## Monitoreo

```bash
# Telemetria
mosquitto_sub -h 192.168.90.24 -t 'bueyuy/#' -t 'rtk/#' -v

# Estado nav (ROS2)
ros2 topic echo /trajectory/status

# Topics ROS2 activos
ros2 topic list
```

## Tools CLI

```bash
ros2 run buey_robot test_mqtt           # ping del broker
ros2 run buey_robot test_nmea_serial    # validar serial GPS
ros2 run buey_robot record_waypoints    # grabar waypoints durante drive manual
ros2 run buey_robot send_waypoints      # publicar waypoints al broker
ros2 run buey_robot gps_waypoint_converter   # lat/lon <-> XY local
```

## Notas

- `motor_gateway` NO se incluye en `outdoor_rtk` ni `indoor_zed` (esos lanzan solo
  drivers + odometry + controller + pose). El motor gateway corre permanente en T1.
- ZED wrapper externo: el launch `indoor_zed` incluye `robot_state_publisher_launch.py`
  del paquete `buey_robot` (el mismo). Asegurate que el ZED SDK este instalado.
- Algoritmo ARC: incluye fase ALIGN (stop-and-turn) al inicio, CRUISE recto con
  micro-correcciones, ARC para curvar entre waypoints sin frenar, y FINAL_APPROACH
  para alinear el angulo final.
# buey_robot_rtk-junio
