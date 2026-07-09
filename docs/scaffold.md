# Scaffold — `buey_robot`

```
buey_robot/
  buey_robot/
    mapper/                       ★ Drivers de sensores. Cada uno expone un topic ROS2 estandar.
      gps_nmea.py                  (GPS NMEA por serial → /gps/fix NavSatFix.
                                    Hoy serial; el dia que cambie, este archivo cambia internamente.)
      imu.py                       (IMU del robot → /imu/data sensor_msgs/Imu.
                                    Hoy usa adapters/mqtt/client.py para suscribirse a imu/* del broker.
                                    Manana puede ser serial; se cambia el transport, la interfaz queda.)
      # ZED: usa zed_ros2_wrapper externo, no escribimos driver.

    odometry/                      ★ Productores de odometria. Salida unificada: /odom_filtered.
      zed.py                       (/zed/odom → /odom_filtered con filtros)
      rtk.py                       (/gps/fix + /heading/imu → /odom_filtered usando GPSConverter)
                                   El controller arriba no sabe cual de los dos esta corriendo.

    navigation/                    ★ Logica de navegacion pura. Recibe outputs por inyeccion.
      controller.py                (ARC con estados ALIGN → CRUISE → ARC → FINAL_APPROACH.
                                    Recibe objetos con .send() por __init__, los llama directo.)
      waypoint_manager.py          (cursor de waypoints: lista + actual + siguiente + progreso)
      ramp_profile.py              (rampas suaves accel/decel para linear y angular)
      joystick.py                  (/joy → /cmd_vel)

    motor/                         ★ Control de motores. Sin lag — output directo via objeto inyectado.
      gateway.py                   (/cmd_vel → cinematica → self.output.send())
      kinematics.py                (cinematica diferencial)
      filters.py                   (soft deadzone, ramp)

    adapters/                      ★ ★ ★ Capa tecnologica intercambiable.
      mqtt/                          Transport MQTT. Inputs para datos que llegan por broker,
                                     outputs para telemetria y comandos hacia el exterior.
        client.py                  (MQTTClient compartido — un solo lugar conecta al broker.
                                    Lo usan outputs/, inputs/ y el driver imu.py)
        inputs/                    Clases de ADQUISICION (NO nodos). Suscriben topics MQTT,
                                   parsean bytes a dict e invocan un callback.
          imu.py                   (MqttImuInput — suscribe imu/* del broker.
                                    on_data(dict) con accel_x/y/z, mag_x/y/z, heading.
                                    Sin rclpy. Instanciada por mapper/imu.py)
        outputs/                   Clases reutilizables (NO nodos). Los productores
                                   las instancian y llaman .send() directo, sin topics ROS2 intermedios.
          motor.py                 (MqttMotorOutput.send → bueyuy/navigation/motors + cmd_vel)
          status.py                (MqttStatusOutput.send → bueyuy/navigation/status)
          waypoints.py             (MqttWaypointsOutput.send → bueyuy/waypoints + waypoints_xy retained)
          gps.py                   (MqttGpsOutput.send → rtk/location/json)
          pose.py                  (EXCEPCION — es NODO: sub /odom_filtered + /heading/* ,
                                    agrega JSON consolidado, publica bueyuy/telemetry/json.
                                    Es nodo porque hace agregacion de varios topics ROS2.)
      serial/                      Transport serial. Inputs para sensores que llegan por UART/USB.
        inputs/                    Clases de ADQUISICION (NO nodos). Abren el puerto en un
                                   thread separado, parsean frames e invocan un callback.
          nmea.py                  (SerialNmeaInput — lee GGA + RMC por pyserial.
                                    on_fix(dict) con lat, lon, alt, quality, satellites, hdop.
                                    Sin rclpy. Loop en thread daemon. Instanciada por mapper/gps_nmea.py)
      # websocket/, ...             ← futuros adapters mismo patron: inputs/ y/o outputs/
      #
      # PATRON DE CAPAS:
      #   adapters/<transport>/inputs/X   adquisicion: como se trae el dato del medio
      #   mapper/X                       logica ROS2: toma el dato del adapter y publica topic estandar
      #
      # Ejemplo de migracion:
      #   IMU llega a serial → agregar adapters/serial/inputs/imu.py con misma firma on_data
      #   mapper/imu.py instancia SerialImuInput en lugar de MqttImuInput — el resto no cambia.

    utils/                         ★ Utilidades genericas, sin dominio.
      math.py                      (quaternion_to_yaw, angle_diff, normalize_angle)
      filters.py                   (MovingAverageFilter, ExponentialFilter)
      gps_converter.py             (lat/lon ↔ ENU)
      config.py                    (load_config + require_key)

  launch/
    nav_outdoor.launch.py          STACK COMPLETO (un solo comando): motor_gateway +
                                   micro_ros_agent + mapper.gps_nmea + mapper.mpu6050_gyro +
                                   odometry.rtk + adapters.mqtt.outputs.pose + imu_bridge +
                                   navigation.controller. Full RTK, waypoints por MQTT.
    motor_gateway.launch.py        navigation.joystick + motor.gateway (drive manual; lo
                                   incluye nav_outdoor). Recibe /cmd_vel del controller
                                   autonomo o /cmd_vel_joy del joystick (prioridad 600ms).

  config/
    navigation.yaml                controller + algoritmos + waypoint + joystick + ramps
    motor.yaml                     gains + deadzone + max_output + watchdog
    sensors.yaml                   NMEA serial port/baud + IMU topics MQTT (o futuros)
    mqtt.yaml                      broker + topics + qos
    robot.yaml                     geometria (wheel_separation, dimensiones)

  tools/                           ★ CLI scripts standalone (no son nodos ROS2)
    record_waypoints.py
    send_waypoints.py
    gps_waypoint_converter.py
    motor_calculator.py
    motor_profile.py
    analyze_recording.py
    pipeline_sim.py
    test_mqtt.py
    test_nmea_serial.py

  waypoints/                       (archivos YAML)
  docs/                            (specs + planes)
```

## Principio rector

Los nodos de `navigation/`, `motor/`, `odometry/` **NUNCA** importan `paho-mqtt` ni nada del broker. Solo publican/consumen topics ROS2.

Los `mapper/` representan **sensores logicos**. Cada driver instancia el adapter de adquisicion
correspondiente y recibe datos via callback. El driver solo hace logica ROS2 (construir mensajes,
publicar topics, aplicar filtros). Ningun driver importa `paho` ni `serial`.

Los `adapters/<transport>/inputs/X` encapsulan la adquisicion: como se trae el dato del medio
(MQTT subscribe, lectura serial, etc.), como se parsea, y como se invoca el callback. Sin rclpy.

Los `adapters/<transport>/outputs/X` encapsulan el envio hacia el exterior. Sin rclpy.

**Ejemplo de migracion sin romper nada:**
- IMU llega a serial → agregar `adapters/serial/inputs/imu.py` con firma `on_data(dict)`
- En `mapper/imu.py`, instanciar `SerialImuInput` en lugar de `MqttImuInput`
- `_on_imu_data` no cambia — recibe el mismo dict

**Separacion de responsabilidades:**

| Capa | Responsabilidad | Importa |
|------|----------------|---------|
| `adapters/X/inputs/Y` | Adquisicion: traer dato del medio y parsear | paho / serial / ... |
| `mapper/Y` | Logica ROS2: construir mensaje y publicar topic | rclpy, sensor_msgs |
| `adapters/X/outputs/Y` | Envio hacia exterior | paho / serial / ... |
