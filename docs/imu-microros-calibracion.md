# IMU micro-ROS (LSM303DLH) — heading calibrado lado ROS, fusion y navegacion

> Sesiones 2026-06-03/04. Robot Buey V. IMU = DFRobot LSM303DLH (accel + magnetometro,
> SIN giroscopio) sobre ESP32, micro-ROS por USB serial (/dev/ttyUSB0, CP2102, 115200).

---

## 0. TL;DR (estado actual)

- El firmware ESP32 publica el **campo magnetico crudo** en `/imu/mag`. NO usamos su
  heading (`/imu/heading`): sale con el offset hard-iron sin corregir y barre solo ~20%
  de los 360 (por eso al girar 90 la brujula casi no cambiaba).
- El **nodo `mapper/imu_compass.py`** lee `/imu/mag`, filtra glitches, aplica calibracion
  hard/soft-iron (de `config/imu.yaml`) y publica `/imu/heading_calibrated` (brujula
  lineal, barre 100% de los 360).
- `rtk.py` consume ese topic (param `gps.imu.heading_topic`), lo pasa a ENU con la
  declinacion y lo fusiona con el curso GPS.
- **Para navegar se confia en el curso GPS, no en el IMU** (los motores corrompen el
  magnetometro al navegar). Ver seccion 5.

---

## 1. Arrancar el agente micro-ROS

```bash
micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```
Truco de conexion: arrancar el agente PRIMERO y LUEGO resetear la ESP32 (boton RST).
Si no engancha, **reset fisico** (desenchufar/enchufar USB) con el agente apagado, luego
relanzar el agente. El agente corre en terminal aparte (como `motor_gateway`), NO se
lanza desde `outdoor_rtk.launch.py`.

Si la ESP32 se reinicia, el nodo `/buey_imu_node` desaparece de `ros2 node list`;
`imu_compass` sigue vivo y retoma solo cuando vuelve `/imu/mag` (no hay que relanzar el
stack ROS).

### Topics que publica el firmware (config.h)

| Topic | Tipo | Notas |
|---|---|---|
| `/imu/mag` | sensor_msgs/MagneticField | **campo crudo x,y,z — lo unico que usamos del firmware** |
| `/imu/heading` | std_msgs/Float32 | heading brujula crudo del firmware — NO usar (offset sin corregir) |
| `/imu/data_raw` | sensor_msgs/Imu | accel y quaternion ROTOS, ignorar (ver seccion 6) |
| `/imu/calibrate`, `/imu/calibration/status` | String | calibracion del firmware — NO se usa (ver seccion 6) |

PUBLISH_INTERVAL_MS=250 (4 Hz). Libreria Pololu LSM303.

---

## 2. imu_compass: heading calibrado lado ROS

`mapper/imu_compass.py` (entry point `imu_compass`):

```
/imu/mag (crudo)
  -> filtra glitches I2C (|componente| > 1500, descarta picos +-4096)
  -> m_cal = soft_iron @ (m_raw - hard_iron)     # centrar + de-sesgar elipse
  -> heading = atan2(m_cal_y, m_cal_x)           # 0-360
  -> (invert opcional) + suavizado por media circular
  -> /imu/heading_calibrated (Float32)
```

Mismo formato/convencion que el viejo `/imu/heading`, asi rtk.py lo consume sin cambiar
su formula. Se lanza desde `outdoor_rtk.launch.py` con `config/imu.yaml`.

### config/imu.yaml (calibracion, VERSIONADA — no se pierde en reboots)

```yaml
imu_compass:
  ros__parameters:
    mag_topic: "/imu/mag"
    heading_topic: "/imu/heading_calibrated"
    glitch_threshold: 1500.0
    filter_alpha: 0.3
    invert: true                # la brujula calibrada gira al reves del rumbo real
    hard_iron: [-51.55, 327.29]            # offset [x, y]
    soft_iron: [1.0080, 0.0703, 0.0703, 1.6180]   # 2x2 fila-mayor
```

### Calibracion: como obtener hard_iron / soft_iron

Causa raiz del problema original: el offset hard-iron del chasis (~-51, 327) es MAS
GRANDE que el radio del campo (~100-140), asi que el origen queda casi fuera de la nube
y `atan2` apenas se mueve al girar.

Procedimiento (herramientas en `~/imu_cal/`):
1. `python3 ~/imu_cal/imu_cal_logger.py` (graba `/imu/mag` a CSV).
2. Girar el robot **4-5 vueltas LENTAS y NIVELADAS** sobre su eje (sin disparar nada del
   firmware). Piso plano, lejos de metal.
3. `python3 ~/imu_cal/fit_ellipse.py` — ajusta la elipse y reporta centro (hard_iron),
   matriz (soft_iron), cobertura angular (debe pasar de ~20% cruda a ~100% calibrada) y
   residual. Copiar los valores a `config/imu.yaml`.
4. Validar: `python3 ~/imu_cal/heading_monitor.py` y girar 360 — la brujula calibrada
   debe barrer ~100% (la cruda se queda en ~25%).

> El residual del ajuste (~0.5) NO predice la precision real: domina el tilt durante el
> giro (sin tilt-compensation porque el accel del firmware esta roto). Una recaptura
> "mas limpia" puede dar peor en campo. Floor practico del heading ~+-10-15 grados.

---

## 3. rtk.py: brujula -> ENU + declinacion

`odometry/rtk.py` se suscribe a `gps.imu.heading_topic` (default `/imu/heading_calibrated`)
y convierte:

```
yaw_enu = 90 - heading_brujula - magnetic_declination_deg
```

Publica `/heading/imu` (grados ENU, comparable directo con `/heading/gps`). La declinacion
absorbe declinacion real + offset de montaje + convencion de la brujula.

### Ajuste de invert y declinacion (campo, 2026-06-04)

1. **invert**: hacer una recta y, sobre todo, una en "L" (cambiar de rumbo). Si al girar
   `/heading/imu` se mueve al REVES que `/heading/gps` -> `invert: true` en imu.yaml.
   (En el Buey quedo `invert: true`.)
2. **declinacion**: andar derecho, comparar `/heading/imu` vs `/heading/gps`; la diferencia
   constante se suma a `magnetic_declination_deg`. Confirmar con 2 direcciones distintas.
   Quedo `magnetic_declination_deg: -90.5` (en config/navigation.yaml).

---

## 4. Bridge MQTT para telemetria

`adapters/mqtt/outputs/imu_bridge.py` (entry `imu_bridge`, se lanza en outdoor_rtk)
republica como JSON al broker, prefijo `bueyuy/`:

| ROS | MQTT |
|---|---|
| `/imu/mag` | `bueyuy/imu/mag` |
| `/imu/heading` | `bueyuy/imu/heading` |
| `/imu/heading_calibrated` | `bueyuy/imu/heading_calibrated` |
| `/heading/imu` | `bueyuy/heading/imu` |
| `/heading/gps` | `bueyuy/heading/gps` |
| `/odom_filtered` | `bueyuy/odom_filtered` (yaw del quaternion = heading FUSIONADO) |

Es nodo en la capa adapters: los nodos de sensores/odometria NO importan paho.

---

## 5. Fusion para navegacion (IMPORTANTE)

**Los motores corrompen el magnetometro al navegar.** Las corrientes (sobre todo el
diferencial al girar) le meten campo: en pruebas el heading IMU llego a estar **60 grados
equivocado** vs el curso real JUSTO al navegar (no en las rectas tranquilas de calibracion).

=> Para navegar se confia en el **curso GPS** (la direccion real de movimiento a crucero,
en Fixed), NO en el IMU. Tuning validado en campo 2026-06-04 (`config/navigation.yaml`):

| Param | Valor | Por que |
|---|---|---|
| `gps.imu.fusion_alpha` | `0.2` | bajado de 0.8: mas peso al curso GPS. Con 0.9 (mas IMU) -> PEOR (banana). |
| `gps.heading.filter_alpha` | `0.15` | bajado de 0.3: amortigua el transitorio basura del curso GPS al arrancar. |

Resultado: navegacion BASE->START (recta) derecha, correccion chica y estable, arranque
limpio. El IMU queda como apoyo solo a muy baja velocidad (donde el curso GPS no existe).

> Navegar en RTK **Float** (calidad 5) anda pero con ~dm de error de posicion. El robot
> ya acepta Float (gps_nmea y rtk usan `quality<4` / `status<2`, que dejan pasar Fixed=4
> y Float=5). Si el dashboard solo deja marcar START en Fixed, ese gate es del lado web.

---

## 6. Bugs del firmware pendientes (no bloquean)

1. **Quaternion vacio**: `/imu/data_raw.orientation` = (0,0,0,1), cov[0]=-1. Por eso
   usamos `/imu/mag` y calculamos el heading nosotros. Fix: encodear yaw->quaternion.
2. **Accel mal escalado**: `(compass.a.x/1000)*9.80665` -> ~15 g en reposo. Bloquea la
   tilt-compensation (la mejora de fondo del heading). Fix: usar la escala real de la lib.
3. **Heading sin tilt-compensation**: en pendiente el heading se desvia. Requiere el #2.
4. **`/imu/mag` con glitches I2C**: ~7-16% de muestras saltan a +-4096. imu_compass las
   filtra, pero conviene filtrarlas en el firmware.
5. **`/imu/calibration/status` mudo**: el topic no entrega mensajes (probado con echo y
   con suscriptor rclpy dedicado, 0 mensajes; el firmware corre la calibracion en un loop
   bloqueante de 30s sin servir la sesion micro-ROS). Por eso NO usamos la calibracion del
   firmware (que ademas vive en RAM y se pierde en cada reboot).

---

## 7. Cheat-sheet

```bash
# Agente micro-ROS (terminal dedicada) — arrancar y luego RST en la ESP32
micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Heading calibrado
ros2 topic echo --once /imu/heading_calibrated
ros2 param get /imu_compass hard_iron

# Calibracion mag (herramientas en ~/imu_cal/)
python3 ~/imu_cal/imu_cal_logger.py      # grabar girando
python3 ~/imu_cal/fit_ellipse.py         # ajustar elipse -> valores para imu.yaml
python3 ~/imu_cal/heading_monitor.py     # validar cobertura calibrada vs cruda
python3 ~/imu_cal/nav_monitor.py         # monitor de navegacion (yaw_fus/headings/cmd_vel)

# Build (si tocas codigo, no para YAML con --symlink-install)
cd ~/ros2_ws && colcon build --packages-select buey_robot --symlink-install
source install/setup.bash
```
