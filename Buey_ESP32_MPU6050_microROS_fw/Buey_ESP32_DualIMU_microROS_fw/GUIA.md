# Guía rápida — MPU6050 IMU por micro-ROS + Web

Firmware en un solo ESP32 (nodo `esp32_imu_node`) que publica:

| Sensor | Tópico | Contenido |
|--------|--------|-----------|
| MPU6050 | `/mpu6050/imu/data` | acelerómetro + giroscopio |

Pines: MPU6050 → `Wire1` (SDA 33 / SCL 25, shield).

> El LSM303 (magnetómetro/brújula) fue **removido**: la brújula quedó inservible y
> el heading pasó a ser gyro + COG GPS (nodo `mpu6050_gyro` → `/heading/fused`).

---

## Levantar todo (4 terminales)

Cada terminal es independiente y hay que **dejarla abierta**.

### Terminal 1 — micro-ROS agent (puente ESP32 ↔ ROS)
```bash
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```
> **No hace falta resetear el ESP32.** El firmware tiene reconexión automática: hace
> ping al agente y crea las entidades solo cuando aparece; si el agente se cae (o lo
> relanzás), las destruye y reconecta solo. Podés arrancar el agente antes o después
> del ESP32, y relanzarlo cuantas veces quieras. Deberías ver `create_client`,
> `establish_session`, `create_publisher` en cuanto el firmware detecte el agente.

### Terminal 2 — rosbridge (puente ROS ↔ WebSocket)
```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
> Queda escuchando en `ws://localhost:9090`.

### Terminal 3 — servidor web
```bash
cd ~/Escritorio/Buey_ESP32_MPU6050_microROS_fw/web
python3 -m http.server 8000
```

### Navegador
- Gráficas crudas:  http://localhost:8000/index.html
- Avión 3D + brújula:  http://localhost:8000/avion.html

Pulsa **Conectar**. Tópico por defecto: `/mpu6050/imu/data`.

### Terminal 4 (opcional) — comprobar por consola
```bash
source /opt/ros/humble/setup.bash
ros2 topic list                       # ver los tópicos
ros2 topic echo /mpu6050/imu/data     # ver los datos del MPU6050
```

---

## Orden recomendado de arranque
1. Terminal 1 (agent) → confirmar handshake (el ESP32 reconecta solo, sin reset).
2. Terminal 2 (rosbridge).
3. Terminal 3 (web) → abrir navegador → Conectar.

El orden es indistinto: el firmware espera al agente y reconecta solo si se cae.
Si recargas la web o reconectas, no hace falta reiniciar el agent.
**Solo un programa puede usar `/dev/ttyUSB0`**: cierra cualquier monitor serie / PlatformIO Monitor antes de lanzar el agent.

---

## Calibración (avión 3D)

### Por qué hace falta
El giroscopio quieto **no marca 0**, sino un pequeño valor constante: el **bias** (zero-rate offset).
El rumbo (yaw) se calcula **integrando** la velocidad angular (`yaw += giro_z · dt`), así que ese
error constante se va **acumulando** y el avión "gira solo" aunque el sensor esté inmóvil. Es la
deriva típica de un IMU sin magnetómetro.

### Qué hace la calibración
Mide el **promedio del giroscopio estando quieto** (~10 s) y se lo **resta** a cada lectura.
Tras calibrar, con el sensor inmóvil el yaw queda prácticamente clavado.

### Cómo calibrar
1. Deja el sensor **totalmente quieto** sobre la mesa.
2. Al **Conectar**, la web calibra sola: verás *"Calibrando… mantén quieto"*. **No lo toques** esos segundos.
3. Para recalibrar cuando quieras: botón **"Calibrar (quieto)"** (útil si la deriva vuelve tras un rato,
   p. ej. al calentarse el sensor).

### Botón "Cero (yaw)"
Distinto de calibrar: **no** mide el bias, solo **pone el rumbo actual a 0°** al instante.
Úsalo para fijar tu "norte" de referencia sin esperar la calibración.

### Lo que NO se arregla calibrando
Roll y Pitch son absolutos (salen del acelerómetro = gravedad) y no derivan.
El **yaw siempre tendrá una deriva residual lenta** porque el bias cambia con la temperatura/tiempo y
no hay referencia absoluta de rumbo. Para eliminarla de verdad haría falta un **magnetómetro**
(el LSM303 lo tiene → se fusionaría aguas abajo en ROS).
