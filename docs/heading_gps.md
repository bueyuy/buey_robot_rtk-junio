# Analisis de rtk.py — Odometria RTK

## Que hace rtk.py

Es el nodo de odometria RTK (~300 lineas) que convierte `/gps/fix` en `/odom_filtered`. Tiene 5 bloques principales:

1. **Parametros** (L29-57): declara 14 params ROS2 sin defaults (fail-fast desde YAML) — origen GPS, calidad, filtros, fusion IMU.

2. **GPS callback** (L129-176): valida satelites y RTK fix, establece origen ENU (auto o via BASE MQTT), convierte lat/lon a coordenadas locales con `GPSConverter`, aplica filtro de media movil opcional, y calcula heading.

3. **IMU callback**: adopta el heading fusionado gyro+COG GPS (`/heading/fused`, ya en yaw ENU) que publica el nodo `mpu6050_gyro`. No hay magnetometro: rtk no re-transforma ni re-fusiona, solo lo toma como `current_heading` cuando `use_imu_heading=true`.

4. **Fusion heading** (L232-254): filtro complementario — parado usa solo IMU, sin IMU usa solo GPS, en movimiento mezcla con `alpha * IMU + (1-alpha) * GPS`.

5. **Publicacion** (L256-282): publica `Odometry` en `/odom_filtered` con pose (x, y, heading como quaternion) y velocidad aproximada — misma interfaz que `zed.py` para que el controller sea agnostico.

---

## Como calcula el heading de GPS

El heading GPS se calcula en `_update_heading()` (L213-230):

1. **Delta de posicion**: toma la diferencia entre la posicion actual y la anterior:

   ```python
   dx = self.current_x - self.prev_x
   dy = self.current_y - self.prev_y
   ```

2. **Velocidad aproximada** (L220): estima m/s asumiendo 10 Hz de GPS:

   ```python
   self.current_speed = math.sqrt(dx**2 + dy**2) * 10.0
   ```

3. **Heading solo si hay movimiento** (L222-228): si la velocidad supera `min_movement_threshold`, calcula el angulo de la direccion de movimiento con `atan2(dy, dx)` — esto da yaw ENU (0 = Este, antihorario). Luego lo suaviza con un `ExponentialFilter` y publica en `/heading/gps` (en grados).

En resumen: no usa ningun dato de orientacion del GPS. Infiere el heading de **la direccion en que se mueve** el robot entre dos fixes consecutivos. Por eso necesita un umbral minimo de movimiento — parado, el `atan2` seria puro ruido.

---

## La velocidad no viene en los datos NMEA?

Las sentencias NMEA (VTG y RMC) traen velocidad sobre el suelo (SOG), pero aca no se usa. El `NavSatFix` de ROS2 solo trae lat, lon, altitud y covarianza — no incluye velocidad ni heading.

Para acceder a SOG/COG del GPS habria que usar otro mensaje ROS2 como `TwistStamped` o `GPSFix` (del paquete `gps_msgs`), que si incluyen esos campos. Dependeria de que el driver NMEA los publique en un topic aparte.

Tal como esta, la velocidad se estima geometricamente (distancia entre fixes * frecuencia asumida de 10 Hz), que es una aproximacion razonable pero tiene dos debilidades:

- **Depende de asumir la frecuencia** — si el GPS publica a otra tasa, el calculo se desfasa.
- **Ruido de posicion se amplifica al derivar** — el filtro de media movil en x/y ayuda, pero no lo elimina.

---

## Como sabe la posicion anterior

En `gps_callback()` (L170-173), justo antes de actualizar la posicion actual, guarda la vigente como "anterior":

```python
self.prev_x = self.current_x
self.prev_y = self.current_y
self.current_x = x
self.current_y = y
```

Ambas variables arrancan en `None` (L66-67), asi que en el **primer fix GPS** no hay posicion anterior y `_update_heading()` sale sin hacer nada (L215-216). Recien a partir del **segundo fix** hay un par de posiciones para calcular el delta.

---

## Cada cuanto se ejecuta rtk.py

No tiene timer propio. Se ejecuta **cada vez que llega un mensaje en `/gps/fix`** — es puramente reactivo al callback (L83):

```python
self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
```

La frecuencia depende de a que tasa publique el driver GPS (`gps_nmea_driver` en `mapper/`).

---

## Cada cuanto publica realmente el driver

La frecuencia real la controla el parametro `gps.publish_rate_hz` en el driver (`gps_nmea.py`, L32). El driver hace throttle en L105-106:

```python
if now - self._last_publish_time < (1.0 / self._publish_rate):
    return
```

Esta configurado a **10 Hz** en `sensors.yaml`:

```yaml
publish_rate_hz: 10.0  # rate de /gps/fix ROS
```

Asi que el `* 10.0` hardcodeado en rtk.py coincide con la config actual. Pero es una **coincidencia fragil** — si alguien cambia `publish_rate_hz` en el YAML, la estimacion de velocidad en rtk.py se rompe silenciosamente porque el `10.0` esta hardcodeado en el codigo.

---

## nmea.py — el adapter de adquisicion GPS

`adapters/serial/inputs/nmea.py` es la clase que lee el puerto serial en un thread separado, parsea las sentencias NMEA y llama al callback `on_fix` del driver.

- **Thread con reconexion** (L93-118): loop que conecta al puerto serial, lee lineas y reconecta automaticamente si se pierde la conexion.
- **Parsea dos sentencias NMEA**:
  - **GGA**: posicion, calidad (0-5), satelites, HDOP, altitud. Emite fix siempre.
  - **RMC**: posicion, pero solo emite fix si status='A' (valido) y quality > 0.
- **`_emit_fix()`**: llama al callback con un dict `{lat, lon, alt, quality, satellites, hdop}`.
- **`_parse_coordinate()`**: convierte formato NMEA `ddmm.mmmm` a grados decimales.
- **`_verify_checksum()`**: valida el checksum XOR estandar NMEA antes de procesar.

Dato clave: la sentencia RMC trae velocidad (SOG) y course (COG) en sus campos, pero este adapter **los ignora** — solo extrae posicion.

---

## Que es COG (Course Over Ground)

**COG** es la direccion en la que se mueve el receptor GPS sobre la superficie terrestre, medida en grados desde el Norte (0-360, sentido horario). Lo calcula el propio chip GPS internamente.

Es basicamente lo mismo que rtk.py calcula manualmente con `atan2(dy, dx)` — la direccion de movimiento — pero el GPS lo entrega ya calculado en la sentencia RMC, con la ventaja de que:

- No depende de asumir una frecuencia fija (el `* 10.0`).
- No amplifica ruido de posicion al derivar entre dos fixes.
- Lo calcula el hardware con acceso a datos internos mas precisos.

La diferencia con heading: COG es hacia donde te **moves**, no hacia donde **apuntas**. Si el robot derrapa o va marcha atras, COG y heading real del chasis no coinciden. Pero para un robot que siempre avanza hacia adelante (como Buey), son practicamente lo mismo — que es exactamente la asuncion que ya hace rtk.py con el `atan2`.
