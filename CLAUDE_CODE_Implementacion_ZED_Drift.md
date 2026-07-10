# Implementación: Optimización Visual Odometry — Robot BUEY

## Contexto para el AI

Estás trabajando en el Robot BUEY, un robot agrícola autónomo skid-steer con navegación indoor basada en visual odometry. El robot usa ROS2 Humble.

### Hardware actual
- **Compute:** ZED Box Orin NX 16GB (Stereolabs integrated unit)
- **Cámara:** ZED X (GMSL2, global shutter, baseline 120mm)
- **ZED SDK:** 5.1.1
- **Motor control:** Raspberry Pi Pico via MQTT
- **4 ruedas** skid-steer, sin encoders actualmente

### Problema
Drift en visual odometry: ~3% lateral en rectas, hasta 10.7° de divergencia yaw en giros de 90°. Causas identificadas: configuración default del ZED wrapper, exposición excesiva por focos industriales, giros stop-and-turn que son degenerados para stereo VO, y falta de loop closure.

### Estructura de paquetes
- **`buey_robot`** — URDF, ZED wrapper launch, configs del wrapper. Contiene:
  - `urdf/buey_urdf.xacro` — descripción del robot
  - `launch/robot_state_publisher_launch.py` — lanza RSP + ZED wrapper
  - `launch/zed_wrapper_launch.py` — lanza la cámara ZED
  - `config/zed_wrapper_params.yaml` — parámetros de la ZED (ESTE ES EL ARCHIVO CLAVE)
- **`buey_robot`** — navegación, motores, telemetría. Contiene:
  - `launch/real_zed.launch.py` — launch principal para navegación autónoma
  - `launch/zed_manual.launch.py` — launch para pruebas manuales
  - `config/navigation.yaml` — parámetros de navegación
  - `config/motor.yaml` — parámetros de motores
  - `buey_robot/navigation/trajectory_controller.py` — controlador de trayectoria
  - `buey_robot/navigation/odom_publisher.py` — publicador de odometría
  - `waypoints/rectangulo_local.yaml` — waypoints del rectángulo 5×2m

### Datos dimensionales del robot (del URDF)
- Ancho total: 0.870m
- Wheelbase: 0.65m
- Track width center-to-center: 0.50m
- Radio de rueda: 0.285m
- Cámara: 0.875m adelante de base_link, 0.765m del piso
- Cámara: tilt recientemente cambiado a ~25° (estaba en 11.7°)

---

## PASO 1: Actualizar tilt de la cámara en URDF

**Archivo:** `buey_robot/urdf/buey_urdf.xacro`

La cámara fue movida físicamente de ~11.7° a ~25° de tilt hacia abajo. Actualizar el URDF para reflejar el cambio.

**Valor anterior:** `0.204` rad (11.7°)
**Valor nuevo:** `0.436` rad (25°)

Hay DOS joints que referencian el tilt (uno para `use_zed_localization=true`, otro para `false`). Ambos deben actualizarse:

En el bloque `xacro:if value="$(arg use_zed_localization)"` (cámara es padre):
```xml
<origin
  xyz="${-(camera_x_origin+robot_center_offset)} 0 ${-camera_z_origin}"
  rpy="0 ${-0.436} 0"
/>
```

En el bloque `xacro:unless value="$(arg use_zed_localization)"` (base_link es padre):
```xml
<origin
  xyz="${camera_x_origin+robot_center_offset} 0 ${camera_z_origin}"
  rpy="0 0.436 0"
/>
```

**Nota:** Si el ángulo exacto tras el ajuste físico no es 25° sino otro, medir el ángulo real y usar ese valor en radianes (grados × π/180).

---

## PASO 2: Crear/actualizar zed_wrapper_params.yaml

**Archivo:** `buey_robot/config/zed_wrapper_params.yaml`

Este es el cambio de mayor impacto. Crear o reemplazar con esta configuración optimizada para ZED Box Orin NX 16GB + SDK 5.1.1:

```yaml
# Configuración optimizada ZED X — Robot BUEY
# ZED Box Orin NX 16GB, SDK 5.1.1, GEN_3
# Última actualización: 2026-02-25

/**:
  ros__parameters:

    general:
      camera_model: 'zedx'
      grab_resolution: 'HD1200'        # 1920×1200 — resolución nativa ZED X, máx features
      grab_frame_rate: 30              # 30fps es óptimo para v ≤ 1m/s
      self_calib: true                 # Re-calibra stereo al inicio
      # pub_resolution y pub_frame_rate se pueden dejar en default
      # o reducir si hay carga de GPU:
      # pub_resolution: 'CUSTOM'
      # pub_downscale_factor: 2.0
      # pub_frame_rate: 15.0

    depth:
      depth_mode: 'PERFORMANCE'        # GEN_3 NO depende de depth, ahorramos GPU
      depth_stabilization: 1           # Mínimo — valores altos causan ghosting en movimiento
      remove_saturated_areas: true     # CRÍTICO — excluye zonas quemadas por focos
      min_depth: 0.3                   # 30cm — enmascara chasis si visible en FoV
      max_depth: 10.0                  # Limita jitter de depth lejano en indoor

    pos_tracking:
      pos_tracking_enabled: true
      pos_tracking_mode: 'GEN_3'       # Feature-based VSLAM con loop closure nativo
      imu_fusion: true                 # Visual-Inertial Odometry — crítico para ZED X
      two_d_mode: true                 # ESENCIAL — bloquea Z, roll, pitch para robot terrestre
      fixed_z_value: 0.0              # Altura Z cuando two_d_mode está activo
      set_gravity_as_origin: true     # Alinea frame con vector de gravedad del IMU
      area_memory: true                # Loop closure para trayectos repetidos
      area_file_path: ''               # Dejar vacío primera vez, luego poner path al .area guardado
      save_area_memory_on_closing: true
      reset_odom_with_loop_closure: true
      set_as_static: false             # La cámara se mueve con el robot
      depth_min_range: 0.3
      publish_tf: true
      publish_map_tf: true

    region_of_interest:
      automatic_roi: true              # Auto-detecta y enmascara partes estáticas del robot
      apply_to_depth: true
      apply_to_positional_tracking: true

    # Controles de exposición ZED X
    # Objetivo: evitar saturación por focos industriales directos
    video:
      auto_exposure_gain: true
      auto_exposure_time_range_min: 28
      auto_exposure_time_range_max: 10000    # REDUCIDO de 30000 — limita saturación y motion blur
      exposure_compensation: 42              # Ligeramente sub-expuesto para evitar reflejos
      saturation: 4
      sharpness: 4
```

**Verificar** que `zed_wrapper_launch.py` referencia este archivo como `ros_params_override_path`. El launch actual ya lo hace:
```python
config_path = os.path.join(buey_pkg_dir, 'config', 'zed_wrapper_params.yaml')
```

---

## PASO 3: Cambiar odometry topic a /pose

**Archivo:** `buey_robot/config/navigation.yaml`

Para beneficiarse del loop closure de area_memory, la navegación debe usar el topic `/pose` (frame `map`, corregido) en vez de `/odom` (frame `odom`, con drift acumulado).

**Cambiar:**
```yaml
odometry:
  topic: "/zed/zed_node/odom"
```

**A:**
```yaml
odometry:
  topic: "/zed/zed_node/pose"
```

**IMPORTANTE:** El topic `/pose` publica `geometry_msgs/PoseStamped`, no `nav_msgs/Odometry`. Verificar que `trajectory_controller.py` y `odom_publisher.py` pueden manejar ambos tipos de mensaje, o adaptar la suscripción.

Revisar en `trajectory_controller.py` el callback `odom_callback` — actualmente espera `Odometry` msg. Si `/pose` publica `PoseStamped`, hay que:
1. Agregar import: `from geometry_msgs.msg import PoseStamped`
2. Agregar suscripción alternativa o cambiar la existente
3. Extraer pose de `msg.pose` en vez de `msg.pose.pose`

Alternativamente, la ZED también publica `/zed/zed_node/pose_with_covariance` que es `PoseWithCovarianceStamped` — puede ser más compatible.

**Opción más simple:** En el zed-ros2-wrapper, el topic `/odom` ya incluye correcciones de area_memory si `publish_map_tf: true`. Verificar si basta con mantener `/odom` y usar el TF `map->odom->base_link`. Si el trajectory_controller usa TF lookups en vez de suscripción directa, ya estaría cubierto.

---

## PASO 4: Alinear wheel_separation

**Archivo 1:** `buey_robot/config/motor.yaml`
**Archivo 2:** `buey_robot/urdf/buey_urdf.xacro`

Hay una discrepancia: motor.yaml usa 0.52m, URDF usa 0.50m. Medir la distancia real centro-a-centro de las ruedas y usar ese valor en ambos lugares.

En `motor.yaml`:
```yaml
motor_control:
  wheel_separation: 0.XX   # ← valor medido real
```

En el URDF, la property `wheel_y_separation` debe coincidir:
```xml
<xacro:property name="wheel_y_separation" value="0.XX" />
```

**Nota:** Para skid-steer la "wheel_separation efectiva" para cinemática suele ser mayor que la geométrica por el slip lateral. El valor empírico calibrado puede diferir del geométrico. Si ya fue calibrado empíricamente a 0.52 y el geométrico es 0.50, puede ser correcto tener valores distintos pero hay que ser consciente.

---

## PASO 5: Implementar modo mini_arc en trajectory_controller

**Archivo:** `buey_robot/navigation/trajectory_controller.py`

Actualmente el controlador tiene `mode: "stop_and_turn"` con un placeholder para `mini_arc` (marcado como TODO). Stop-and-turn es geométricamente degenerado para stereo VO porque durante rotación pura no hay parallax translacional.

### Especificación del mini_arc

**Concepto:** En vez de frenar → girar en el lugar → avanzar, el robot desacelera → hace un arco suave combinando velocidad lineal + angular → vuelve a velocidad de crucero.

**Parámetros del arco:**
- Radio mínimo: R ≈ 0.55m (limitado por ancho del robot 0.87m en corredor de 2m)
- Velocidad lineal durante arco: 0.15–0.20 m/s
- Velocidad angular máxima durante arco: 0.36 rad/s (= v/R = 0.2/0.55)
- Perfil: S-curve con rampas de 0.5s para transiciones suaves
- Giro de 90° toma ~4.4s

**Valores empíricos ya calibrados** (de `tools/motor_profile.py`):
```python
# Estos valores ya fueron probados en el robot real
ARC = (-6, -50)     # motor values (L, R)
# Produce: v = 0.163 m/s, w = 0.134 rad/s
```

**Algoritmo sugerido para mini_arc:**

```
Estado APPROACHING:
  1. Navegar recto hacia waypoint con cruise_linear
  2. A distancia decel_distance del WP, empezar a desacelerar
  3. Calcular heading al SIGUIENTE waypoint
  4. Calcular la diferencia angular (cuánto hay que girar)

Estado ARC_TURN:
  5. Combinar velocidad lineal reducida + velocidad angular proporcional
  6. v_linear = min_linear (0.15-0.20 m/s)
  7. v_angular = angular_gain × error_heading, clamped a max 0.4 rad/s
  8. Aplicar rampas S-curve a ambas velocidades
  9. Cuando heading error < alignment_tolerance → transición a CRUISE

Estado CRUISE:
  10. Rampa de vuelta a cruise_linear, v_angular → 0
  11. Navegar recto al siguiente WP
```

**Config a agregar en navigation.yaml:**
```yaml
controller:
  mode: "mini_arc"          # ← cambiar de "stop_and_turn"
  arc:
    min_linear_m_s: 0.15    # velocidad lineal mínima durante arco
    max_angular_rad_s: 0.40 # velocidad angular máxima durante arco
    decel_before_wp_m: 1.5  # distancia para empezar a desacelerar antes del WP
    ramp_duration_s: 0.5    # duración de transición S-curve
```

**IMPORTANTE:** Mantener la opción de `stop_and_turn` como fallback. Implementar `mini_arc` como modo alternativo seleccionable por config.

---

## PASO 6: Agregar detección de AprilTags (futuro, Tier 3)

**Paquete nuevo** o nodo adicional. No es prioritario para la primera iteración pero es el upgrade más potente para eliminar drift.

### Concepto
4 AprilTags (familia `tag36h11`, tamaño ~15–20cm) colocados en las esquinas del rectángulo de navegación. Cada tag tiene una posición conocida en el mundo. Cuando la cámara ve un tag, calcula su pose absoluta.

### Opciones de detección
1. **`apriltag_ros`** — Standard ROS2, corre en CPU
2. **Isaac ROS AprilTag** — CUDA-acelerado, ideal para Jetson

### Integración con navegación
- **Opción simple:** Nodo que detecta tags, calcula pose, y cuando la diferencia con la pose actual del robot excede un threshold, publica una corrección
- **Opción robusta:** Usar `robot_localization` EKF fusionando ZED VIO + AprilTag poses (requiere más setup pero es más suave)

### Config de tags (ejemplo)
```yaml
# apriltag_positions.yaml
# Posiciones de tags en frame world/map (metros)
tags:
  - id: 0
    x: 0.0
    y: 0.0
    z: 0.0      # altura del tag en la pared o piso
    yaw: 0.0
  - id: 1
    x: 5.0
    y: 0.0
    z: 0.0
    yaw: 1.5708  # 90°
  - id: 2
    x: 5.0
    y: 2.0
    z: 0.0
    yaw: 3.1416  # 180°
  - id: 3
    x: 0.0
    y: 2.0
    z: 0.0
    yaw: 4.7124  # 270°
```

**Dejar esto para después de validar Pasos 1–5.**

---

## PASO 7: Configuración de performance del Jetson

Crear un script de startup para asegurar performance máxima:

**Archivo:** `scripts/jetson_performance.sh`

```bash
#!/bin/bash
# Configurar Jetson Orin NX 16GB para máximo rendimiento
# Ejecutar una vez al bootear o agregar a rc.local

# Modo MAXN (25W para Orin NX 16GB)
sudo nvpmodel -m 0

# Fijar clocks a máximo (previene throttling)
sudo jetson_clocks

# Verificar
echo "Power mode: $(nvpmodel -q)"
echo "GPU freq: $(cat /sys/devices/gpu.0/devfreq/17000000.ga10b/cur_freq 2>/dev/null || echo 'N/A')"

# Monitorear temperatura (opcional)
# tegrastats --interval 5000
```

---

## Orden de implementación

Seguir este orden estrictamente. Testear después de cada paso.

| Paso | Cambio | Archivos | Test |
|---|---|---|---|
| 1 | Actualizar tilt URDF | `buey_urdf.xacro` | Verificar TF tree en rviz2 |
| 2 | Config ZED wrapper | `zed_wrapper_params.yaml` | Lanzar cámara, verificar logs de GEN_3, two_d_mode |
| 3 | Cambiar a /pose topic | `navigation.yaml` + controller | Corrida manual, comparar drift vs baseline |
| 4 | Alinear wheel_separation | `motor.yaml` + URDF | Verificar cinemática |
| 5 | Implementar mini_arc | `trajectory_controller.py` + config | Corrida autónoma rectángulo, comparar telemetría |
| 6 | AprilTags (futuro) | Paquete nuevo | Después de validar 1–5 |
| 7 | Jetson performance | Script de startup | Verificar con tegrastats |

### Test de validación después de Pasos 1–3

Usar `tools/motor_profile.py` con el launch manual (`zed_manual.launch.py`) para hacer una corrida de referencia. Comparar la telemetría nueva contra los archivos `menos_luz.json` y `con_mas_luz_salto.json` existentes. Las métricas clave a comparar:

- Divergencia yaw VO vs IMU al final del recorrido (target: < 4°, antes: 10.7°)
- Max salto de posición en 1 frame (target: < 2cm, antes: 4.6cm)
- Drift lateral en recta de 3m (target: < 1%, antes: ~3%)
