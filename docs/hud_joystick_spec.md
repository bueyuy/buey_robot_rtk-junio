# Vista Joystick HUD — Spec fullscreen estilo DJI

Vista fullscreen para operar el robot con joystick en el campo.
Todo en una pantalla, sin scroll, sin navegacion. Informacion al borde, mapa al centro.
Estilo heads-up display como controles DJI.

**IMPORTANTE**: NO hay GPS. El robot usa odometria visual ZED X.

---

## 1. Fuentes de datos

### 1.1 Conexion MQTT
- Broker: `ws://192.168.90.24:9001` (WebSocket)
- Libreria: `mqtt.js`
- Suscribirse a `bueyuy/#`

### 1.2 Topics necesarios

| Topic | Tipo | Formato | Para que |
|-------|------|---------|----------|
| `bueyuy/pipeline` | retained | JSON | Parametros de config (rangos, deadzone, etc) |
| `bueyuy/telemetry/json` | ~10Hz | JSON | Posicion (x,y) y headings para mapa y brujula |
| `bueyuy/navigation/cmd_vel` | on change | `v&w` | Barras de velocidad |
| `bueyuy/navigation/motors` | 20Hz | `velL&velR` | Barras de motor y deadzone |
| `bueyuy/navigation/status` | ~10Hz | string | Estado, WP progress, distancia, heading err |
| `bueyuy/waypoints_xy` | retained | JSON | Waypoints en el mapa |

### 1.3 Parseo del status string

Regex del `pipeline.realtime_topics.status.parse_regex`:
- Modo: `/(Following|Aligning|COMPLETED)/`
- WP progress: `/WP (\d+\/\d+)/`
- Distancia: `/dist=([0-9.]+)m/`
- Heading error: `/heading_err=([0-9.-]+)/`

---

## 2. Principios de diseno

- **Una sola pantalla**: `width: 100vw; height: 100vh; overflow: hidden`
- **Centro despejado**: mapa 2D ocupa el area central
- **Datos en los bordes**: widgets flotantes semitransparentes
- **Tipografia monospace condensada**: numeros grandes, legibles a distancia
- **Color solo para estado**: verde=ok, amarillo=warning, rojo=error, cyan=dato activo
- **Sin bordes gruesos**: separadores sutiles
- **Fondo**: `#0a0e14`
- **Widgets**: `rgba(13,17,23,0.85)` con `backdrop-filter: blur(8px)`, border-radius 8px
- **Font**: `JetBrains Mono`, `Fira Code`, o `monospace`. Base 13px.

---

## 3. Layout

```
┌──────────────────────────────────────────────────────────────────────────┐
│                                                                          │
│  ┌─ STATUS ────────┐                           ┌─── FREQ ────────────┐  │
│  │ ● FOLLOWING      │   WP 3/8   1.45m   5.2°  │ CTL 10.1  MOT 20.0 │  │
│  └──────────────────┘                           │ ZED 29.8            │  │
│                                                 └─────────────────────┘  │
│                                                                          │
│  ┌ VELOCITY ┐       ┌──────────────────────────────┐     ┌ MOTORS ────┐ │
│  │ LINEAR   │       │                              │     │    L    R  │ │
│  │          │       │                              │     │  ┌──┐┌──┐ │ │
│  │ ▓▓▓░░░   │       │                              │     │  │▓▓││▓▓│ │ │
│  │ 0.18 m/s │       │         MAPA 2D               │     │  │▓▓││▓▓│ │ │
│  │          │       │                              │     │  │▓▓││  │ │ │
│  │ ANGULAR  │       │         robot + trail          │     │  │  ││  │ │ │
│  │ ▓▓░░░░   │       │         + waypoints           │     │  └──┘└──┘ │ │
│  │ 0.15 r/s │       │                              │     │ -42  -58  │ │
│  └──────────┘       └──────────────────────────────┘     └────────────┘ │
│                                                                          │
│  ◉ 45.2° ZED            12:34:56    ⚡ MQTT             ┌─ DZ GAUGE ────┐ │
│                                                       │ L [▓░░████████]│ │
│  ⚙                                                   │ R [▓░░████████]│ │
│                                                       └───────────────┘ │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 4. Widget: Status Bar (top-left)

```
● FOLLOWING        WP 3/8        1.45m to WP        5.2° err
```

- **Modo**: circulo + texto con color:
  - `FOLLOWING` → verde `#22c55e`
  - `ALIGNING` → amarillo `#eab308`, animacion pulse 1s
  - `COMPLETED` → azul `#3b82f6`
  - Sin datos >2s → rojo `#ef4444` "OFFLINE", pulsante
- **WP progress**: blanco bold
- **Distancia**: cyan `#06b6d4`
- **Heading error**: blanco. >15° amarillo. >30° rojo

---

## 5. Widget: Frequency Badges (top-right)

```
CTL 10.1   MOT 20.0   ZED 29.8
```

3 badges inline. Calcular frecuencia real midiendo intervalo entre mensajes:

| Badge | Topic para medir | Color |
|-------|------------------|-------|
| CTL | `bueyuy/navigation/status` | cyan `#06b6d4` |
| MOT | `bueyuy/navigation/motors` | verde `#22c55e` |
| ZED | `bueyuy/telemetry/json` | violeta `#8b5cf6` |

Calculo: ring buffer de 10 timestamps, `freq = 10 / (last - first)`. Actualizar cada 1s.
Si freq = 0 (>2s sin mensajes): badge rojo "OFF".
Font 11px monospace uppercase.

---

## 6. Widget: Velocity Gauges (left)

Dos barras VERTICALES apiladas. Datos de `bueyuy/navigation/cmd_vel`.
Rangos sacados del pipeline retained (stages "deceleration" y "angular_control").

### Linear (arriba)

```
┌──────────┐
│ LINEAR   │
│          │
│ ░░░░░░░░ │  ← cruise (0.3)
│ ▓▓▓▓▓▓▓▓ │  ← actual
│ ▓▓▓▓▓▓▓▓ │
│ ────────  │  ← min (0.05)
│          │
│  0.18    │
│  m/s     │
└──────────┘
```

- Rango: 0 (abajo) a `cruise_linear_m_s` (arriba)
- Fill: gradient cyan `#06b6d4` → azul `#3b82f6`, de abajo hacia arriba
- Marker horizontal en `min_linear_m_s`: linea punteada
- Marker horizontal en `cruise_linear_m_s`: linea solida blanca
- Valor numerico debajo: font 16px bold + "m/s" font 11px gris
- Width ~60px, height ~120px

### Angular (abajo)

```
┌──────────┐
│ ANGULAR  │
│ ░░░░░░░░ │  ← +max (0.5)
│ ▓▓▓▓▓▓▓▓ │  ← actual
│ ════════ │  ← 0 (centro)
│          │
│ ░░░░░░░░ │  ← -max (-0.5)
│  0.15    │
│  rad/s   │
└──────────┘
```

- Rango: -`max_angular_rad_s` (abajo) a +`max_angular_rad_s` (arriba), centro en 0
- Positivo (giro derecha): cyan `#06b6d4`, crece desde centro hacia arriba
- Negativo (giro izquierda): rojo `#ef4444`, crece desde centro hacia abajo
- Linea central en 0: blanca solida
- Width ~60px, height ~120px

---

## 7. Widget: Mapa 2D (center)

Ocupa todo el espacio central. Es el elemento mas grande, foco principal de la vista.

### Datos
- Posicion: `bueyuy/telemetry/json` → `x`, `y`
- Heading: `heading_zed` (grados, 0=este, 90=norte)
- Waypoints: `bueyuy/waypoints_xy` (retained)

### Renderizado
- Canvas, fondo `#0a0e14` con grid sutil cada 1m en `rgba(255,255,255,0.04)`
- **Robot**: triangulo (base 12px, alto 18px) en direccion heading
  - Fill cyan `#00d4ff`, borde blanco 1px, glow `0 0 8px #00d4ff`
- **Trail**: polyline ultimas ~200 posiciones
  - Color cyan `rgba(0,212,255,0.3)`, grosor 2px, fade gradual
- **Waypoints**: circulos de radio 10px, numerados (1, 2, 3...)
  - Completados: fill verde `#22c55e`, opacidad 70%
  - Actual: fill amarillo `#eab308`, borde pulsante
  - Pendientes: fill gris `#4b5563`, borde `#6b7280`
  - Numero blanco centrado, font 10px bold
- **Linea robot→target**: punteada blanca `rgba(255,255,255,0.25)`
- **Escala**: ruler "1m" esquina inferior-derecha
- **Coordenadas**: `x: 1.23  y: 5.67` esquina superior-izquierda, font 11px gris
- **Auto-zoom**: viewport se ajusta a bounding box (waypoints + robot) + 20% margen
- Coordenadas: x=este (derecha), y=norte (arriba)

---

## 8. Widget: Motor Bars (right)

Dos barras verticales L y R. Datos de `bueyuy/navigation/motors`.
Simulan visualmente los sticks del control DJI.

```
┌─────────────┐
│  MOTORS     │
│   L     R   │
│ ┌───┐ ┌───┐│
│ │   │ │   ││  +100
│ │   │ │   ││
│ │===│ │===││  0
│ │▓▓▓│ │▓▓▓││
│ │▓▓▓│ │▓▓▓││
│ │▓▓▓│ │   ││
│ └───┘ └───┘│  -100
│ -42.0 -58.0│
└─────────────┘
```

- Dos barras de 30px ancho, ~160px alto, separadas 12px
- Rango -100 a +100, centro en 0
- Linea central en 0: blanca solida 1px
- Positivo (adelante): verde `#22c55e`, crece desde centro hacia arriba
- Negativo (atras): rojo `#ef4444`, crece desde centro hacia abajo
- Valor numerico debajo: rojo si negativo, verde si positivo, gris si 0
- Si ambos = 0 por >1s: "STOPPED" gris encima de las barras
- Labels "L" y "R" arriba

---

## 9. Widget: Compass + Headings (bottom-left)

```
  ╭───╮
  │ ↗ │   ZED  45.2°
  ╰───╯   IMU  44.8°
```

- **Brujula**: circulo 40px, borde blanco fino
  - Ticks en N, S, E, W
  - Flecha interior rota con `heading_zed`, color cyan
  - N siempre arriba
- **Headings**: al lado
  - "ZED 45.2°" en cyan `#06b6d4`
  - "IMU 44.8°" en gris `#9ca3af`
  - Si diferencia ZED-IMU > 5°: warning amarillo

---

## 10. Widget: Deadzone Gauge (bottom-right)

Mini barras horizontales L y R. Parametros del pipeline stage "soft_deadzone".

```
┌───────────────────────────────┐
│ DZ  L [▓▓░░░████████████████] │
│     R [▓▓░░░████████████████] │
│        0  5  20           100  │
└───────────────────────────────┘
```

- Dos barras de ~200px ancho, ~8px alto
- 3 zonas:
  - 0 a `low`: rojo `rgba(220,38,38,0.3)`
  - `low` a `high`: amarillo `rgba(234,179,8,0.3)`
  - `high` a `max_output`: verde `rgba(34,197,94,0.3)`
- Marcador vertical en `|valor actual|`, color segun zona
- Si en zona muerta: marcador pulsante
- Ticks: 0, low, high, max_output

---

## 11. Widget: Connection + Clock (bottom-center)

```
12:34:56    ⚡ MQTT
```

- Reloj HH:MM:SS, font 11px gris, actualizado cada segundo
- Indicador MQTT:
  - Conectado: `⚡` verde `#22c55e`
  - Desconectado: `⚡` rojo pulsante
  - Reconectando: `⚡` amarillo

---

## 12. Modal: Parametros (⚙ icon)

Icono ⚙ en bottom-left, al lado de la brujula. Click abre modal overlay.

Generar dinamicamente del pipeline retained: iterar stages, mostrar params agrupados por nodo.

```
┌─────────────────────────────────────────────┐
│  ⚙ Parameters                          [✕]  │
│                                              │
│  ▾ Controller (10Hz)                         │
│    Camera Offset                             │
│      offset_x_m ............ 0.875    m      │
│    Position Filter                           │
│      window_size ........... 5               │
│    Distance to WP                            │
│      goal_tolerance_m ...... 0.50     m      │
│    Decel Zone                                │
│      decel_distance_m ...... 2.00     m      │
│      cruise_linear_m_s ..... 0.30     m/s    │
│    Ramp Profile                              │
│      accel_rate ............ 0.03     /cyc   │
│      decel_rate ............ 0.06     /cyc   │
│    ...                                       │
│                                              │
│  ▸ Motor Gateway (20Hz)                      │
│                                              │
└─────────────────────────────────────────────┘
```

- Fondo: `rgba(10,14,20,0.95)` con blur, centrado, max-width 500px
- Secciones colapsables por nodo
- Stages como subtitulos dentro de cada nodo
- Keys monospace gris, valores blancos alineados a la derecha
- Unidades gris oscuro
- Cerrar con ✕, click fuera, o Escape

---

## 13. Responsive: mobile / tablet

En pantallas < 768px:

- Mapa ocupa 60% del alto (arriba)
- Status bar simplificado: `● FOLLOWING  WP 3/8  1.45m`
- Velocity y Motor bars debajo del mapa, horizontales
- Brujula como icono en status bar
- Frequencies como badges compactos en status bar
- Deadzone gauge oculto (disponible en modal params)

Prioridad: **status > mapa > motores > velocidades**

---

## 14. Notas tecnicas

1. **Fullscreen**: `width: 100vw; height: 100vh; overflow: hidden`. Cero scroll.

2. **Pipeline retained para config**: la vista HUD no dibuja el diagrama de pipeline,
   pero usa `bueyuy/pipeline` para leer parametros de config (rangos de velocidad,
   deadzone low/high, max_output, etc). Asi no hardcodea nada.

3. **Sin GPS**: no mostrar GPS, satelites, fix quality, HDOP. Solo ZED.

4. **Performance**: topics llegan a 10-20Hz. Throttlear canvas redraw a 30fps
   con requestAnimationFrame. Solo redibujar si cambiaron los datos.

5. **Stack sugerido**: React/Preact + Canvas API para mapa + CSS para widgets.
   O vanilla JS + lit-html.
