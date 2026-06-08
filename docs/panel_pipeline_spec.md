# Panel Pipeline — Spec de Engineering/Debug

Vista de ingenieria para entender y tunear el sistema completo del robot.
Muestra el diagrama de flujo de las 17 etapas desde la ZED hasta los motores,
con parametros configurables, formulas y descripciones en cada caja.

**IMPORTANTE**: NO hay GPS. El robot usa odometria visual ZED X.

---

## 1. Fuente de datos

### 1.1 Conexion MQTT
- Broker: `ws://192.168.90.24:9001` (WebSocket)
- Libreria: `mqtt.js`
- Suscribirse a `bueyuy/#`

### 1.2 Topic principal: `bueyuy/pipeline` (retained)

Llega inmediatamente al suscribirse. Contiene la estructura completa del pipeline
con todas las etapas, parametros, formulas y descripciones.
**Toda la UI se genera dinamicamente de este JSON. No hardcodear nada.**

Estructura completa:

```json
{
  "stages": [
    {
      "order": 1,
      "id": "zed_odom",
      "node": "ZED Camera",
      "frequency_hz": 30,
      "name": "ZED Odometry",
      "description": "Camara stereo publica posicion y orientacion por visual-inertial odometry",
      "outputs": "x, y, qx, qy, qz, qw, vx, vy"
    },
    {
      "order": 2,
      "id": "quaternion_to_yaw",
      "node": "trajectory_controller",
      "frequency_hz": 10,
      "name": "Heading",
      "description": "Extrae yaw del quaternion fusionado (IMU+visual)",
      "formula": "heading = atan2(2(qw·qz + qx·qy), 1 - 2(qy² + qz²))"
    },
    {
      "order": 3,
      "id": "camera_offset",
      "node": "trajectory_controller",
      "name": "Camera Offset",
      "description": "La camara esta montada adelante. Compensa para obtener posicion del centro del robot",
      "formula": "center = cam - offset × cos/sin(heading)",
      "params": { "offset_x_m": 0.875 }
    },
    {
      "order": 4,
      "id": "position_filter",
      "node": "trajectory_controller",
      "name": "Position Filter",
      "description": "Media movil para suavizar ruido de odometria. Promedia ultimas N muestras",
      "formula": "filtered = sum(last N) / N",
      "params": { "enabled": true, "type": "moving_average", "window_size": 5 }
    },
    {
      "order": 5,
      "id": "distance_calc",
      "node": "trajectory_controller",
      "name": "Distance to WP",
      "description": "Distancia euclidiana al waypoint actual. Si menor a tolerancia, avanza al siguiente",
      "formula": "dist = √(dx² + dy²)",
      "params": { "goal_tolerance_m": 0.50 }
    },
    {
      "order": 6,
      "id": "deceleration",
      "node": "trajectory_controller",
      "name": "Decel Zone",
      "description": "Reduce velocidad proporcionalmente cerca del WP. Fuera: cruise. Dentro: proporcional",
      "formula": "vel = cruise × (dist / decel_dist) si dist < decel_dist, sino cruise",
      "params": { "decel_distance_m": 2.0, "cruise_linear_m_s": 0.3, "min_linear_m_s": 0.05 }
    },
    {
      "order": 7,
      "id": "ramp",
      "node": "trajectory_controller",
      "name": "Ramp Profile",
      "description": "Rampa asimetrica: acelera suave (protege visual odometry). Frena rapido (seguridad)",
      "formula": "vel += min(delta, accel_rate) o vel -= min(delta, decel_rate)",
      "params": { "accel_rate": 0.03, "decel_rate": 0.06, "accel_m_s2": 0.3, "decel_m_s2": 0.6 }
    },
    {
      "order": 8,
      "id": "heading_error",
      "node": "trajectory_controller",
      "name": "Heading Error",
      "description": "Angulo entre direccion actual y direccion al WP. Normalizado [-pi, pi]",
      "formula": "error = atan2(sin(target - current), cos(target - current))",
      "params": { "alignment_tolerance_deg": 10.0 }
    },
    {
      "order": 9,
      "id": "angular_control",
      "node": "trajectory_controller",
      "name": "Angular Control",
      "description": "Velocidad angular proporcional al error. En alineacion: minimo para vencer friccion",
      "formula": "ω = error × gain, clamped ±max_angular",
      "params": { "proportional_gain": 0.8, "max_angular_rad_s": 0.5, "alignment_min_vel": 0.4, "alignment_max_vel": 1.0 }
    },
    {
      "order": 10,
      "id": "cmd_vel",
      "node": "trajectory_controller",
      "name": "cmd_vel",
      "description": "Salida del controlador: velocidad lineal (rampeada) y angular (proporcional)",
      "outputs": "v (m/s), ω (rad/s)"
    },
    {
      "order": 11,
      "id": "input_select",
      "node": "motor_gateway",
      "frequency_hz": 20,
      "name": "Input Select",
      "description": "Joystick prioridad sobre nav. Sin input por timeout: watchdog envia 0,0",
      "params": { "input_timeout_ms": 500, "joystick_timeout_ms": 600, "joystick_max_speed": 0.5, "joystick_angular_scale": 0.8 }
    },
    {
      "order": 12,
      "id": "gains",
      "node": "motor_gateway",
      "name": "Gains",
      "description": "Escala velocidades al rango de trabajo de los motores",
      "formula": "v × linear_gain, w × angular_gain",
      "params": { "linear_gain": 2.5, "angular_gain": 2.0 }
    },
    {
      "order": 13,
      "id": "differential",
      "node": "motor_gateway",
      "name": "Differential",
      "description": "Cinematica diferencial: convierte v,ω a rueda izq/der. Normaliza a escala 0-100",
      "formula": "R = -((v + ω×L/2) / norm) × 100, L = -((v - ω×L/2) / norm) × 100",
      "params": { "wheel_separation": 0.52, "vel_normalization": 1.5 }
    },
    {
      "order": 14,
      "id": "clamp",
      "node": "motor_gateway",
      "name": "Clamp",
      "description": "Limita salida al rango maximo del motor",
      "formula": "val = clamp(val, -max, +max)",
      "params": { "max_output": 100 }
    },
    {
      "order": 15,
      "id": "soft_deadzone",
      "node": "motor_gateway",
      "name": "Soft Deadzone",
      "description": "Zona muerta suave con rampa. <low: apagado. low-high: rampa gradual. >high: directo",
      "formula": "|v|<low→0, low≤|v|<high→rampa, |v|≥high→passthrough",
      "params": { "low": 5.0, "high": 20.0 }
    },
    {
      "order": 16,
      "id": "motor_output",
      "node": "motor_gateway",
      "name": "Motor Output",
      "description": "Comando final al Pico via MQTT. Ejecuta raw sin procesamiento",
      "outputs": "velL, velR (-100 a 100)"
    },
    {
      "order": 17,
      "id": "pico",
      "node": "Pico W",
      "name": "Pico Motors",
      "description": "Microcontrolador aplica PWM directo a los motores"
    }
  ],
  "realtime_topics": {
    "telemetry": { "topic": "bueyuy/telemetry/json", "format": "JSON {x, y, heading_zed, heading_imu, timestamp}" },
    "cmd_vel":    { "topic": "bueyuy/navigation/cmd_vel", "format": "v&w" },
    "motors":     { "topic": "bueyuy/navigation/motors", "format": "velL&velR" },
    "status":     { "topic": "bueyuy/navigation/status", "format": "string",
                    "parse_regex": {
                      "mode": "(Following|Aligning|COMPLETED)",
                      "wp_progress": "WP (\\d+/\\d+)",
                      "distance": "dist=([0-9.]+)m",
                      "heading_err": "heading_err=([0-9.-]+)"
                    } },
    "waypoints":  { "topic": "bueyuy/waypoints_xy", "format": "JSON {waypoints: [{x, y}, ...]}" },
    "joystick":   { "topic": "bueyuy/joystick", "format": "JSON {v, w} or v&w" }
  },
  "controller_mode": "stop_and_turn"
}
```

### 1.3 Topics de tiempo real

Los unicos valores dinamicos de esta vista:

| Dato | Topic | Parseo | Widget |
|------|-------|--------|--------|
| Linear, Angular | `bueyuy/navigation/cmd_vel` | `split('&')` → `[v, w]` | Barras de velocidad + anotacion en pipeline |
| Motor L, R | `bueyuy/navigation/motors` | `split('&')` → `[velL, velR]` | Barras de motor + deadzone gauge + anotacion en pipeline |
| Estado nav | `bueyuy/navigation/status` | regex (ver parse_regex) | Status bar |

---

## 2. Layout general

Dark theme, fondo `#0d1117`. La pagina se divide en dos zonas verticales:

```
┌────────────────────────────────────────────────────────────────────┐
│ ┌─ STATUS BAR ───────────────────────────────────────────────────┐ │
│ │ ● FOLLOWING   WP 3/8   1.45m   5.2°                           │ │
│ └────────────────────────────────────────────────────────────────┘ │
│                                                                    │
│ ┌─ PIPELINE ─────────────────────────────────────────────────────┐ │
│ │                                                                 │ │
│ │  Fila 1: ZED → Controller stages (1-10)                        │ │
│ │  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐     │ │
│ │  │ 1 │→│ 2 │→│ 3 │→│ 4 │→│ 5 │→│ 6 │→│ 7 │→│ 8 │→ ... │ │
│ │  └───┘  └───┘  └───┘  └───┘  └───┘  └───┘  └───┘  └───┘     │ │
│ │                                                                 │ │
│ │  Fila 2: Motor Gateway stages (11-17)                          │ │
│ │  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐  ┌───┐            │ │
│ │  │11 │→│12 │→│13 │→│14 │→│15 │→│16 │→│17 │            │ │
│ │  └───┘  └───┘  └───┘  └───┘  └───┘  └───┘  └───┘            │ │
│ │                                                                 │ │
│ └─────────────────────────────────────────────────────────────────┘ │
│                                                                    │
│ ┌─ WIDGETS ──────────────────────────────────────────────────────┐ │
│ │  [Velocity bars]  [Motor bars]  [Deadzone gauge]  [Ramp]       │ │
│ └────────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────┘
```

- **Status bar**: arriba, compact, ancho completo
- **Pipeline**: zona central, la mas grande. Scroll horizontal si no cabe
- **Widgets**: abajo, en fila horizontal. Complementan con datos dinamicos

---

## 3. Status Bar

```
┌──────────────────────────────────────────────────────────────────┐
│  ● FOLLOWING        WP 3/8        1.45m to WP        5.2° err   │
└──────────────────────────────────────────────────────────────────┘
```

Datos de `bueyuy/navigation/status`, parsear con regex del pipeline:

- **Modo**: badge con color:
  - `FOLLOWING` → verde `#22c55e`
  - `ALIGNING` → amarillo `#eab308`, pulsante
  - `COMPLETED` → azul `#3b82f6`
  - Sin datos >2s → rojo `#ef4444` "OFFLINE"
- **WP progress**: "WP 3/8" blanco bold
- **Distancia**: "1.45m" cyan `#06b6d4`
- **Heading error**: "5.2°" blanco. >15° amarillo. >30° rojo.
- Fondo: `rgba(13,17,23,0.9)`, border-radius 6px, height ~36px

---

## 4. Pipeline — diagrama de flujo

### 4.1 Generacion dinamica

Iterar `pipeline.stages` ordenado por `order`. Por cada stage, dibujar una caja.
Agrupar por `node` con fondo y label de grupo.

### 4.2 Caja de cada stage

Estructura interna de una caja:

```
┌─────────────────────────────┐
│  3. Camera Offset           │  ← order + name
│─────────────────────────────│
│  La camara esta montada     │  ← description
│  adelante. Compensa para    │     (gris #9ca3af, 11px, max 3 lineas)
│  obtener posicion centro    │
│─────────────────────────────│
│  center = cam - offset      │  ← formula
│  × cos/sin(heading)         │     (monospace, cyan #06b6d4, 11px)
│─────────────────────────────│
│  offset_x_m     0.875      │  ← params
│                              │     (monospace, blanco, 11px)
└─────────────────────────────┘
```

Reglas:
- **Titulo**: `{order}. {name}` — bold, blanco, 13px
- **Descripcion**: siempre visible — gris `#9ca3af`, 11px, wrap a max 3 lineas
- **Formula**: solo si existe — monospace, cyan `#06b6d4`, 11px. Separada con linea fina
- **Params**: solo si existen — cada param como `key    value` monospace, 11px. Key en gris, value en blanco
- **Outputs**: si existe, mostrar al fondo: "→ v (m/s), ω (rad/s)" italic gris
- Cajas sin formula ni params (ej: zed_odom, pico): solo titulo + descripcion, mas compactas
- Width: ~200px min, ~260px max. Height: dinamica segun contenido
- Borde: 1px solid `rgba(255,255,255,0.1)`, border-radius 8px
- Fondo: `rgba(13,17,23,0.7)`

### 4.3 Agrupacion por nodo

Las cajas del mismo `node` se agrupan con un recuadro de fondo y un label con frecuencia:

| Node | Fondo | Color borde | Label |
|------|-------|-------------|-------|
| `ZED Camera` | `rgba(139,92,246,0.06)` | `rgba(139,92,246,0.3)` violeta | `ZED Camera 30Hz` |
| `trajectory_controller` | `rgba(6,182,212,0.06)` | `rgba(6,182,212,0.3)` cyan | `Trajectory Controller 10Hz` |
| `motor_gateway` | `rgba(34,197,94,0.06)` | `rgba(34,197,94,0.3)` verde | `Motor Gateway 20Hz` |
| `Pico W` | `rgba(245,158,11,0.06)` | `rgba(245,158,11,0.3)` amber | `Pico W` |

- Label del grupo: esquina superior izquierda del recuadro, font 11px bold, color del grupo
- El `frequency_hz` se toma de la primera stage del grupo que lo tenga
- Las frecuencias quedan integradas en el diagrama, no como badges separados

### 4.4 Flechas y conexiones

- **Entre cajas del mismo grupo**: flecha `──→` color del grupo, 1px
- **Entre grupos** (cambio de nodo): flecha mas gruesa 2px, blanca, con label:
  - Entre `cmd_vel` → `Input Select`: label `/cmd_vel`
  - Entre `Motor Output` → `Pico`: label `MQTT`

### 4.5 Anotaciones dinamicas en las flechas

Los unicos valores en tiempo real se muestran **en las flechas entre nodos**:

1. **Flecha cmd_vel → Input Select**:
   ```
   ──[ v: 0.18  ω: 0.15 ]──→
   ```
   Datos de `bueyuy/navigation/cmd_vel`. Font monospace 12px, fondo `rgba(6,182,212,0.15)` cyan.

2. **Flecha Motor Output → Pico**:
   ```
   ──[ L: -42.0  R: -58.0 ]──→
   ```
   Datos de `bueyuy/navigation/motors`. Font monospace 12px, fondo `rgba(34,197,94,0.15)` verde.

Si no hay datos aun: mostrar `[ — ]` en gris.

### 4.6 Layout en 2 filas

El pipeline se divide en 2 filas naturalmente por el cambio de nodo:

**Fila 1** — ZED + Controller (stages 1-10):
```
╔═ ZED 30Hz ══╗  ╔══════════════════════ Controller 10Hz ═══════════════════════════════════════════╗
║ ┌──────────┐ ║  ║ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐              ║
║ │ 1. ZED   │─╫──╫→│ 2. Head  │→│ 3. Cam   │→│ 4. Filter│→│ 5. Dist  │→│ 6. Decel │→ ...        ║
║ │ Odometry │ ║  ║ │          │ │ Offset   │ │          │ │          │ │ Zone     │              ║
║ └──────────┘ ║  ║ └──────────┘ └──────────┘ └──────────┘ └──────────┘ └──────────┘              ║
╚══════════════╝  ║                                                                                ║
                  ║ ... ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐                       ║
                  ║  →│ 7. Ramp  │→│ 8. Head  │→│ 9. Ang   │→│10. cmd   │                       ║
                  ║     │ Profile  │ │ Error    │ │ Control  │ │ _vel     │                       ║
                  ║     └──────────┘ └──────────┘ └──────────┘ └──────────┘                       ║
                  ╚═════════════════════════════════════════════════════════════════════════════════╝
                                                                     │
                                                          ──[ v: 0.18  ω: 0.15 ]──
                                                                     │
                                                                     ▼
**Fila 2** — Motor Gateway + Pico (stages 11-17):
╔══════════════════════════ Motor Gateway 20Hz ═══════════════════════════════════════╗  ╔═ Pico ═══╗
║ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐    ║  ║┌────────┐║
║ │11. Input │→│12. Gains │→│13. Diff  │→│14. Clamp │→│15. Soft  │→│16. Motor │─╫──╫│17. Pico││
║ │ Select   │ │          │ │          │ │          │ │ DZ       │ │ Output   │    ║  ║│ Motors │║
║ └──────────┘ └──────────┘ └──────────┘ └──────────┘ └──────────┘ └──────────┘    ║  ║└────────┘║
╚═════════════════════════════════════════════════════════════════════════════════════╝  ╚══════════╝
                                                                          │
                                                               ──[ L: -42  R: -58 ]──
                                                                          │
                                                                          ▼ Pico
```

Scroll horizontal si el viewport es angosto.

---

## 5. Widgets (zona inferior)

Debajo del pipeline. En fila horizontal, todos compact.

### 5.1 Barras de velocidad

Dos barras HORIZONTALES lado a lado. Datos de `bueyuy/navigation/cmd_vel`.

**Linear:**
```
Linear   [████████████░░░░░░░░░░░░░░░░░░]  0.18 m/s
          0    min(0.05)          cruise(0.3)
```
- Rango 0 a cruise. Fill: gradient cyan→azul
- Marker punteado en `min_linear_m_s`, solido en `cruise_linear_m_s`
- Valor numerico a la derecha

**Angular:**
```
Angular  [░░░░░░░░░░░░░░████████░░░░░░░░]  0.15 rad/s
          -0.5          0           +0.5
```
- Rango -max a +max, centro en 0
- Positivo: cyan. Negativo: rojo. Linea central blanca
- Valor numerico a la derecha

Parametros de rangos sacados del pipeline (stages "deceleration" y "angular_control").

### 5.2 Barras de motor

Dos barras VERTICALES lado a lado: L y R. Datos de `bueyuy/navigation/motors`.

```
 MOTORS
  L    R
┌──┐ ┌──┐
│  │ │  │  +100
│==│ │==│  0
│▓▓│ │▓▓│
│▓▓│ │▓▓│
└──┘ └──┘  -100
-42  -58
```

- Rango -100 a +100, centro en 0
- Positivo: verde (adelante). Negativo: rojo (atras). Cero: gris
- Linea central a 0 blanca
- Valor numerico debajo de cada barra con color
- Si ambos = 0 por >1s: "STOPPED" gris
- Width ~40px cada barra, height ~140px

### 5.3 Deadzone gauge

Dos mini barras horizontales L y R. Datos de `bueyuy/navigation/motors`.
Parametros del pipeline stage "soft_deadzone".

```
DZ  L [▓▓░░░██████████████████]  42.0
    R [▓▓░░░██████████████████]  58.0
       0  5  20            100
```

- 3 zonas proporcionales:
  - 0 a `low` (5): rojo `rgba(220,38,38,0.3)` — zona muerta
  - `low` a `high` (5-20): amarillo `rgba(234,179,8,0.3)` — rampa
  - `high` a `max_output` (20-100): verde `rgba(34,197,94,0.3)` — pass-through
- Marcador vertical en `|valor actual|`, color segun zona
- Ticks numericos: 0, low, high, max_output
- Valor numerico a la derecha
- Width ~250px, height ~8px cada barra

### 5.4 Rampa (opcional, si cabe)

Mini grafico estatico del perfil de rampa. Parametros del pipeline stage "ramp".

```
      ╱━━━━━━╲
    ╱          ╲
  ╱              ╲
━╱                ╲━
 accel 0.03    decel 0.06
 (0.3 m/s²)   (0.6 m/s²)
```

- Pendiente izquierda suave (accel): verde `#22c55e`
- Pendiente derecha pronunciada (decel): rojo `#ef4444`
- Linea punteada horizontal a nivel cruise
- Labels con valores reales del pipeline

---

## 6. Notas tecnicas

1. **Todo del pipeline retained**: no hardcodear parametros, nombres, formulas ni descripciones.
   Iterar `pipeline.stages` para generar las cajas. Si manana cambia una formula o se agrega
   una etapa, la web se actualiza sola.

2. **Valores dinamicos**: solo en las anotaciones de las flechas entre nodos y en los widgets.
   Las cajas del pipeline son estaticas (params de config, no datos en tiempo real).

3. **Frecuencias integradas**: estan en los labels de grupo del pipeline. NO usar badges
   sueltos separados. No mostrar "GPS 5Hz" ni nada de GPS.

4. **Dark theme**: fondo `#0d1117`, cajas `rgba(13,17,23,0.7)`, font monospace 13px base.

5. **Scroll horizontal**: si el pipeline no cabe en el viewport, scroll horizontal en la zona del diagrama.

6. **Responsive**: en mobile, el pipeline se scrollea horizontal. Los widgets se apilan vertical.
