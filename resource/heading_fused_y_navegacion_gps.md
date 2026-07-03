# Heading gyro+GPS y navegación por waypoints GPS — Buey V

Documento de referencia del trabajo de heading y navegación autónoma (jun–jul 2026).
Cubre: reemplazo de la brújula muerta por un heading fusionado **gyro MPU6050 + COG GPS**,
el tuning de esa fusión, y la navegación por **waypoints GPS absolutos** (rectángulo ABCD)
con creep recto inicial y espera de calibración del gyro.

Estado: validado en campo y pusheado a `main` (hasta commit `85968bf`).

---

## 1. Problema y solución

El heading del robot (yaw de `/odom_filtered`, el que usan navegación y telemetría) venía de
la brújula LSM303 fusionada con el COG GPS. La brújula quedó **inservible**:

- `/imu/mag` mudo en vivo, `rawCompass` congelado en un único valor (79.93°) toda la corrida.
- El sensor LSM303 no publica (I2C / cableado / el firmware dual no lo levanta).

**Solución:** heading fusionado gyro + COG GPS. El gyro del MPU6050 aporta los **giros**
(suave, sin lag, sin depender del magnetómetro); el **COG GPS** aporta la referencia
absoluta (corrige el cero arbitrario y el drift lento del gyro).

Rendimiento del gyro medido en el primer rectángulo (crudo, sin fusión): rotación total
acumulada **358°** sobre las 4 vueltas (ideal 360°), drift en reposo **~0.012 °/s**.

---

## 2. Firmware dual (dato clave)

El firmware micro-ROS que corre en la ESP32 (`/dev/ttyUSB0`, agente en `outdoor_rtk.launch`)
es un nodo **DUAL** llamado `esp32_dual_imu_node`. Publica las **dos** IMUs:

| Tópico | Sensor | Contenido | Uso |
|---|---|---|---|
| `/imu/data` | LSM303 | accel + **mag** (`angular_velocity` = 0.0, **sin gyro**) | `imu_compass` vía `/imu/mag` |
| `/mpu6050/imu/data` | MPU6050 | accel + **GYRO real** | `mpu6050_gyro` (heading) |

Rate observado en vivo: **~11 Hz** por tópico.

⚠️ El folder `Buey_ESP32_MPU6050_microROS_fw/` del repo es un firmware **single-IMU**
(publica `/imu/data`) que **NO es el que corre en el robot**. Para el gyro hay que
consumir `/mpu6050/imu/data`. Este fue el bug inicial: `mpu6050_gyro` escuchaba
`/imu/data` (la LSM303, sin gyro) → integraba yaw cero → `/heading/gyro` quedaba en 0.0.

---

## 3. Pipeline completo

```
firmware dual ESP32 (esp32_dual_imu_node)
   ├─ /mpu6050/imu/data  (Imu: accel + GYRO)     ┐
   └─ /imu/data (LSM303, sin gyro)               │      /gps/fix (GPSFix: track=COG, speed, status)
                                                 │                    │
                                                 ▼                    ▼
        ┌───────────────────── mapper/mpu6050_gyro.py ─────────────────────┐
        │ 1. auto-bias del gyro (robot quieto, descarta si hay movimiento)  │
        │ 2. integra yaw rate → heading relativo   → /heading/gyro (ENU)    │
        │ 3. fusión: offset = (90-cog) - gyro      → /heading/fused         │
        │    (solo yendo DERECHO + RTK fix + en movimiento)  (absoluto ENU) │
        └──────────────────────────────┬───────────────────────────────────┘
                                        ▼
                     odometry/rtk.py  (gps.imu.heading_is_enu = true)
                     adopta /heading/fused como current_heading (sin re-transformar)
                                        ▼
                              /odom_filtered (yaw)
                          ┌─────────────┴─────────────┐
                          ▼                           ▼
                 pose_publisher              navigation/controller.py
          bueyuy/telemetry/json {heading}         (navegación)
```

`imu_bridge` además espeja a MQTT: `bueyuy/heading/gyro` (crudo) y `bueyuy/heading/fused`.

---

## 4. Nodo `mpu6050_gyro.py` (el corazón)

### `_imu_callback` (cada muestra del gyro, ~11 Hz)

1. Si está calibrando bias → `_collect_bias` (promedia con robot quieto).
2. `cwz = wz - bias[z]` (yaw rate corregido). Guarda `self._yaw_rate` (para el gate de recta).
3. Integra: `heading_deg += degrees(cwz * dt)` → heading relativo ENU. Publica `/heading/gyro`.
4. Si el offset ya está fijado: `fused = (heading_deg + offset) % 360` → publica `/heading/fused`.

### `_gps_callback` (cada fix GPS, ~1 Hz) — la fusión

```python
if speed < gps_min_speed:               return   # COG no confiable lento
if not RTK fix:                         return
if abs(yaw_rate) > straight_max_yaw_rate: return  # ← CLAVE: solo corregir yendo DERECHO

gps_ref = (90 - cog) % 360              # COG brújula → yaw ENU (igual que rtk.py)
target  = wrap180(gps_ref - heading_deg)

if offset no inicializado:
    warm-up: promedio circular de N muestras → offset   # snap robusto
else:
    offset += alpha * wrap180(target - offset)          # EMA lenta
```

**Idea:** el `offset` es un escalar que lleva el gyro (relativo) al frame absoluto del GPS.
Solo se toca **yendo derecho** (donde el COG es confiable); en los giros el gyro lleva el
heading solo y el GPS no lo toca.

---

## 5. Parámetros de fusión (`config/imu.yaml`, bloque `mpu6050_gyro`)

| Param | Valor | Qué hace |
|---|---|---|
| `gps_min_speed` | `0.20` | vel mínima para confiar en el COG (el robot crucea ~0.25–0.29 m/s) |
| `straight_max_yaw_rate` | `0.08` rad/s | solo corrige el offset yendo derecho (giros ~8°/s quedan afuera) |
| `offset_alpha` | `0.10` | velocidad de corrección del offset (alto = rápido/ruidoso, bajo = lento/estable) |
| `offset_init_samples` | `5` | muestras (yendo derecho) a promediar para el snap inicial |
| `gps_require_rtk` | `true` | exige RTK fix para tocar el offset |
| `gyro_bias` / `auto_bias_*` | — | bias del gyro, auto-estimado en reposo al arrancar |

---

## 6. `rtk.py` — reusar el "fused" existente

`rtk.py` tiene el modo `gps.imu.heading_is_enu` (en `navigation.yaml`):

- **`true`** → `imu_callback` adopta `/heading/fused` **directo** como `current_heading`
  (sin re-transformar ni re-fusionar), y `_update_heading` no lo pisa. El yaw de
  `/odom_filtered` pasa a ser gyro+GPS. **Este es el modo activo.**
- **`false`** → modo brújula clásico (fusión mag+GPS). Se preserva **por si se arregla
  el LSM303**.

Un solo "fused", el de `/odom_filtered`, ahora alimentado por gyro+GPS en vez de la
brújula muerta. Lo consumen igual `pose_publisher` (telemetría) y `trajectory_controller`
(navegación).

### Convención (importante)

- COG / brújula: **compass** — 0 = Norte, positivo horario (CW).
- Yaw ENU (lo que usa `rtk.py`): 0 = Este, positivo antihorario (CCW).
- Conversión: **`yaw_enu = 90 - cog`** (normalizado). Ya implementado en `rtk.py`.

`/heading/gyro` (crudo) está en ENU con cero arbitrario → en el dashboard "gira al revés"
de una brújula. **Es normal**, es solo para comparar/depurar. El bueno es `/heading/fused`
(= `odomYaw`), ya alineado al GPS. El movimiento físico del crudo es correcto (coincide con
el joystick); solo la numeración va al revés por la convención.

---

## 7. Navegación por waypoints GPS

### Modo `goal_source: mqtt_waypoints` (la ruta llega por MQTT)

> **Actualización (jul 2026):** la ruta GPS ya **no** se lee de un `.yml`
> (`rectangulo_ABCD.yaml`). El dashboard la **publica por MQTT** en `bueyuy/waypoints`
> y el controller la carga en vivo (adapter `inputs/waypoints.py` →
> `MqttWaypointsInput`). Se eliminaron el modo de archivo `waypoints_gps` y el modo
> `mqtt_positions` (BASE→START); quedan **`mqtt_waypoints`** (outdoor) y
> **`waypoints_file`** (x,y local, indoor).

Contrato del payload (retained): `{"waypoints_gps": [{"lat","lon"}, ...], "loop": bool}`.

- Waypoints en **lat/lon absolutos**.
- Se convierten a local con el origen **BASE** (mismo `GPSConverter` y frame que `rtk.py` /
  `/odom_filtered`), reusando `gps_to_local`.
- **Quedan FIJOS** a su posición GPS: no se desplazan si el robot arranca corrido.
- `loop: true` recorre la ruta en bucle (al llegar al último waypoint vuelve al primero).

| | `waypoints_file` (local, indoor) | `mqtt_waypoints` (outdoor) |
|---|---|---|
| Fuente | YAML `x, y` metros | MQTT `bueyuy/waypoints` (lat/lon) |
| Origen | arranque del robot | BASE (fijo, = rtk) |
| Si arranca corrido | todo se desplaza | los waypoints quedan fijos ✓ |

Requiere `use_mqtt_base: true` + BASE publicada por el dashboard. Sin BASE, el controller
guarda la ruta y la carga al llegar la BASE.

### Creep recto inicial (`controller.prealign`)

El robot rotaba sobre su eje al iniciar (sin COG → heading sin alinear → giraba mal). Se
agregó un **creep recto** antes de navegar: avanza en línea recta `distance_m` para generar
COG y alinear el heading fused, y recién después navega.

| Param | Valor | Qué hace |
|---|---|---|
| `controller.prealign.enabled` | `true` | activa el creep recto inicial |
| `controller.prealign.distance_m` | `1.0` | metros a avanzar recto antes de navegar |

### Calibración del gyro disparada desde la navegación (`trigger_gyro_calibration` + `wait_for_gyro_calibration`)

La calibración del gyro se dispara **al arrancar la navegación**, no al levantar `outdoor_rtk`.
El robot está recién parado y quieto en el punto de inicio → el bias sale fresco. El controller
publica `/gyro/calibrate` (std_msgs/Empty); `mpu6050_gyro` re-corre el auto-bias (y reinicia la
fusión GPS), dejando de publicar `/heading/gyro` mientras recalibra. El controller espera a que
`/heading/gyro` reaparezca (recalibrado) y recién ahí se mueve.

Señal de "listo" sin tópico nuevo: la presencia de `/heading/gyro`. Para no tomar un
`/heading/gyro` **viejo** (previo a la recalibración) como listo, el controller ignora el tópico
durante un margen corto (`GYRO_CALIB_TRIGGER_GRACE_S=1.5s`) tras enviar el trigger; el auto-bias
tarda ~10s, así que el margen no lo alcanza.

Secuencia de arranque:

```
1. Llevar el robot al inicio con joystick y PARARLO (quieto).
2. Lanzar la navegación. El controller (~2s tras arrancar) publica /gyro/calibrate.
3. mpu6050_gyro recalibra el bias (robot inmóvil) → no publica /heading/gyro.
   El controller ESPERA ("Esperando calibracion del gyro (robot quieto)...").
4. Gyro recalibrado → creep recto → converge el heading fused (/heading/fused_ready).
5. Navega B → C → D → A.
```

| Config | `wait_for_gyro_calibration` | `trigger_gyro_calibration` | Motivo |
|---|---|---|---|
| `navigation.yaml` (base) | `false` | `false` | indoor/ZED no tiene gyro, no cuelga |
| `navigation_outdoor.yaml` | `true` | `true` | outdoor: recalibra fresco y espera |

`mpu6050_gyro` sigue calibrando también al arrancar su nodo (`auto_bias_enabled=true`), así que
correr `outdoor_rtk` solo (sin navegar) igual da heading para telemetría. Para recalibrar a mano
sin navegar: `ros2 topic pub --once /gyro/calibrate std_msgs/msg/Empty "{}"` (robot quieto).

---

## 8. Vértices del rectángulo ABCD

Extraídos de `resource/telemetry_2026-07-02T19-53-42.json` ("marcar los cuatro puntos"),
promediando el lat/lon en cada parada del stop-n-turn. Todos **RTK fixed (q=4, 12 sats)** →
precisión cm.

| Punto | Latitud | Longitud | Local ENU desde A (E, N) |
|---|---|---|---|
| **A** (origen/arranque) | −34.88083573 | −56.15160369 | (0, 0) |
| **B** | −34.88086011 | −56.15163464 | (−2.83, −2.71) |
| **C** | −34.88087342 | −56.15161409 | (−0.95, −4.20) |
| **D** | −34.88085204 | −56.15158580 | (1.63, −1.82) |

Rectángulo ~3.9 × 2.4 m. Lados: A-B = 3.92 · B-C = 2.39 · C-D = 3.51 · D-A = 2.44 m.

Recorrido: **A → B → C → D → A**. A es el **origen/arranque** (el robot arranca ahí, hace el
creep, y navega B→C→D→A). Estos lat/lon se **publican por MQTT** en `bueyuy/waypoints`
(payload `{"waypoints_gps": [...], "loop": bool}`) desde el dashboard; ya no se leen de un
archivo YAML.

Copy-paste para mapa/dashboard:

```
A: -34.88083573, -56.15160369
B: -34.88086011, -56.15163464
C: -34.88087342, -56.15161409
D: -34.88085204, -56.15158580
```

---

## 9. Calibración del gyro en `avion.html`

`web/avion.html` (cliente web de depuración) ya tenía calibración de bias + integración de
yaw, pero promediaba N muestras a ciegas. Se replicó la lógica robusta de `_collect_bias`
del nodo: **descarta y reinicia la calibración si detecta movimiento** (`|gyro| > 0.05 rad/s`)
durante la captura, para no hornear un bias con el robot en movimiento.

---

## 10. Cómo correrlo (3 terminales)

1. **T1** — joystick: `ros2 launch buey_robot motor_gateway.launch.py`
2. **T2** — sensores + odometría + heading fused: `ros2 launch buey_robot outdoor_rtk.launch.py`
   - **Sin reset de la ESP32**: el firmware reconecta solo (máquina de estados
     micro-ROS con ping al agente). Se puede relanzar `outdoor_rtk` sin tocar el ESP32.
   - Fijar la **BASE** en el dashboard (`use_mqtt_base=true` la necesita).
   - Con joystick, llevar el robot cerca de **A** y **pararlo** (quieto).
3. **T3** — navegación (default `goal_source:=mqtt_waypoints`):
   ```bash
   ros2 launch buey_robot trajectory_controller.launch.py
   ```
   Publicar la ruta por MQTT (retained), p.ej.:
   ```bash
   mosquitto_pub -t bueyuy/waypoints -r -m '{"waypoints_gps":[{"lat":...,"lon":...}, ...],"loop":false}'
   ```
   La navegación **dispara la calibración del gyro** (robot parado en A) → espera BASE +
   odom + el bias fresco → creep recto → navega la ruta (fija al BASE). Mantener el robot
   **quieto** los ~10 s del auto-bias tras lanzar T3.

---

## 11. Historial de tuning (por si hay que re-tunear)

Iteraciones sobre recordings de rectángulos stop-n-turn:

| Cambio | Antes → Ahora | Motivo |
|---|---|---|
| `offset_alpha` | 0.03 → **0.10** | τ≈33 s convergía muy lento; re-alinea en cada recta |
| snap inicial | 1ª muestra → **media circular de 5** | evitar lock con el COG transitorio al arrancar (err inicial −124°) |
| `gps_min_speed` | 0.30 → **0.20** | el robot crucea ~0.25–0.29 y nunca cruzaba 0.30 → offset no inicializaba (`odomYaw` quedaba en 0 toda la corrida) |
| `straight_max_yaw_rate` | — → **0.08 rad/s** (fix clave) | en giros el COG GPS laggea al gyro e inyectaba error (saltos −30° post-giro); ahora el offset solo se corrige yendo derecho |

**Resultado validado** (run 2026-07-01 19:53, `telemetry_2026-07-01T19-53-54.json`):
offset mediana **0.1°**, std **8.6°** (puro ruido del COG), snap inicial limpio (−1.5°),
sin drift ni saltos post-giro. Los picos puntuales ~±20° son **solo en el instante del giro**
(el COG GPS salta), donde el gyro es más correcto que el COG — no es error del fused.

---

## 12. Aprendizajes clave

- **Brújula LSM303 muerta**: `/imu/mag` mudo, `rawCompass` congelado. No es problema del
  dashboard, es el sensor. Inservible como fuente de heading.
- **El gyro es excelente**: 358°/360° de rotación acumulada, drift ~0.012 °/s. Reemplaza a la
  brújula sin discusión.
- **El COG GPS es basura girando en el lugar** (skid-steer, SOG≈0 en el pivote): de ahí el
  gate de yaw-rate (`straight_max_yaw_rate`) — corregir el offset **solo en rectas**.
- **Convención**: `/heading/gyro` crudo es ENU (CCW, cero arbitrario) → se ve "al revés" de una
  brújula en el dashboard. Es esperado. El bueno es `/heading/fused` (= `odomYaw`), vía `90-cog`.
- **Firmware dual**: el gyro vive en `/mpu6050/imu/data`, no en `/imu/data` (LSM303 sin gyro).

---

## 13. Estado en git

| Commit | Qué |
|---|---|
| `9fc6da3 → 1cf6457` | pipeline: nodo `mpu6050_gyro`, `/heading/fused`, `rtk heading_is_enu`, telemetría |
| `84ef34e` | tuning: `offset_alpha=0.10` + warm-up del snap |
| `46bd57f` | fix final: `gps_min_speed=0.20` + gate `straight_max_yaw_rate=0.08` |
| `85968bf` | nav: waypoints GPS (rectángulo ABCD) + creep recto + gate de calibración + firmware (sin `.pio`) + `avion.html` |

**Sin trackear** (no son del trabajo): `resource/telemetry_*.json` (datos de las corridas),
`buey_robot/odometry/rtk copy.py` (copia suelta, descartable), `tools/analyze_heading.py`
(herramienta preexistente).

---

## 14. Pendientes / follow-ups

- Validar el fused en una **corrida de navegación autónoma real** (hasta ahora fue joystick;
  el rectángulo ABCD con `waypoints_gps` es la primera prueba de navegación).
- Flecha del gyro **crudo** en el dashboard convertida a compass (`(90 - heading_gyro) % 360`)
  si molesta verla "al revés".
- **Arreglar el LSM303** (`/imu/mag` mudo) si se quiere la brújula como respaldo absoluto:
  revisar I2C / cableado / init en el firmware dual. Con `heading_is_enu=false` vuelve el modo
  brújula.
- Tuning fino de `offset_alpha` / `gps_min_speed` si cambia el crucero del robot.
