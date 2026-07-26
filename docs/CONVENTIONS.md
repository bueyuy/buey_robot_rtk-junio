# Convenciones — buey_robot

Directrices para cualquier dev (humano o agente) que toque este repo. El mapa de
capas/nodos/topics vive en [architecture.md](architecture.md); aca van las REGLAS, no el mapa.

## 1. Arquitectura por capas

- **Cada capa habla UN vocabulario.** Los drivers hablan del mundo fisico (lat/lon, yaw
  crudo); navegacion habla x/y metrico. Un dato no cambia de nomenclatura dentro de una capa.
- **UN solo punto bilingue.** La conversion lat/lon <-> x/y ocurre en un unico nodo
  (`OdometryGps`, dueño del datum). Ningun otro archivo mezcla lat/lon con x/y. Arriba de esa
  frontera, nadie dice lat/lon.
- **SRP con criterio, no a rajatabla.** Un nodo puede hacer varias cosas si son la MISMA
  preocupacion. No dividir por micro-tarea. Regla para separar en un nodo nuevo: solo si
  necesita su propia frecuencia, su propio ciclo de vida, o es un punto de reuso/test. Si no,
  es un modulo (clase) dentro de un nodo.
- **Alinear con ROS2/Nav2 en nombres y patrones, SIN su maquinaria.** Se toman conceptos
  (velocity_smoother, twist_mux, staleness/transform-tolerance, controller/plugins) pero
  implementados como nodos simples topic-based. NO se trae costmaps, TF tree, BT, pluginlib
  ni lifecycle (overkill para este robot).

## 2. Nomenclatura

- **Clase de nodo = PascalCase(carpeta + archivo).** `navigation/controller.py` -> `NavigationController`;
  `motor/gateway.py` -> `MotorGateway`; `odometry/gps.py` -> `OdometryGps`. Excepcion: en `drivers/`
  la carpeta es una categoria, el archivo va solo (`GpsNmea`, `ImuMpu6050`).
- **Nombre de nodo ROS** = snake_case de la clase (`navigation_controller`, `odometry_gps`).
- **Topics: explicitos sobre QUE es el dato y en QUE frame**, no la tecnologia ni el metodo.
  `/gps/course` (no `/gps/heading`), `/heading/fused`, `/odom`, `/local/route`. Fuente unica de
  nombres de topic: `contracts.py`. Nada de strings de topic sueltos en los nodos.
- Comentarios y docs en espanol SIN acentos. Nombres (variables, clases) en ingles.

## 3. Patrones de datos

- **Ausencia = invalido.** El productor publica SOLO cuando el dato vale (fix confiable, heading
  convergido, COG util). El consumidor infiere invalidez por ausencia/antiguedad (staleness con
  timeout, estilo transform tolerance de Nav2). NO usar NaN, sentinels ni un topic `*_ready`
  paralelo (segundo source of truth que se desincroniza).
- **La validez del dato la decide el PRODUCTOR.** La confiabilidad (ej RTK Fixed/Float) se
  chequea donde se conoce la calidad (la capa geografica), no aguas arriba.
- **Reusar el timer existente para detectar ausencias** (un no-evento necesita un tick temporal);
  no meter un watchdog aparte si ya hay un loop.

## 4. Config

- 100% en YAML. Cada nodo declara sus params con `declare_parameter` SIN default (fail-fast: si
  falta una key, el nodo no arranca). Ver `utils/params.py` (`load_params`).
- `config/` **espeja** la estructura de `buey_robot/` (un yaml por nodo, mismas subcarpetas).
- Top-key del yaml = nombre del nodo (o `/**` si el bloque es compartido). Si se renombra un
  nodo, se renombra su top-key.
- Solo `mqtt.yaml` se lee con `load_config()` (no son params ROS2, es la config del cliente MQTT).

## 5. MQTT / adapters

- El unico lugar que toca `paho` es `adapters/mqtt/client.py` (singleton). Los nodos de
  `navigation/`, `motor/`, `odometry/`, `drivers/` NO importan paho.
- **Los bridges** (`command_bridge` MQTT->ROS, `telemetry_bridge` ROS->MQTT) viven en
  `adapters/mqtt/` y usan el `client` DIRECTO (subscribe/publish + parse inline).
- Una primitiva `Mqtt*Output` inyectable existe SOLO si un nodo NO-adapter necesita MQTT sin
  importar paho (hoy: solo `MqttMotorOutput`, que inyecta el gateway). Todo lo demas, el bridge
  lo hace directo. No envolver cada topic en una clase.

## 6. Comentarios y logs

- **Comentarios minimos (KISS).** Header de 1 linea por archivo (que es / que hace) SIEMPRE.
  En codigo: comentar solo lo NO inferible (unidades, magic number con su razon, condicion no
  obvia). Nada de header meta de 2da linea ni comentarios que repiten el nombre del atributo.
- **Self-contained.** El comentario describe lo que hace ESE archivo, sin nombrar otros nodos ni
  actores externos (dashboard, operador). Si moves el archivo a otro proyecto, el comentario
  sigue siendo verdad. Los topics propios de I/O si van (son su interfaz).
- **Configs YAML**: un comentario corto por linea, con el `#` alineado en columna.
- **Logs = excepcion a "minimo".** Van EXPLICITOS y auto-descriptivos: sin jerga ambigua,
  con NUMEROS (valor medido + umbral) y la CONSECUENCIA. Ej malo: "fix no confiable". Ej bueno:
  "GPS poco preciso: accuracy 0.05m > 0.035m (RTK Fixed). No publico odometria."
- **Observabilidad = logs**, NO topics `/status` de navegacion (un adapter futuro reenvia
  `/rosout` a MQTT). En loops que corren a N Hz: loguear el estado SOLO en transicion (guardar
  el ultimo y comparar) o con `throttle_duration_sec`; nunca una linea por tick.

## 7. Estilo de codigo

- Compacto: pocos saltos de linea, sin expandir una clave por renglon.
- Codigo autoexplicativo: si un archivo esta bien hecho no necesita explicacion.

## 8. Mensajes / rutas

- Preferir tipos estandar (`std_msgs`, `nav_msgs`, `geometry_msgs`). NO crear un msg custom
  (rosidl = paquete nuevo ament_cmake, blast radius alto) salvo que realmente haga falta.
- La ruta de waypoints viaja como `std_msgs/String` con JSON `{waypoints:[...], loop}` (igual
  criterio que el flujo viejo, sin msg custom).

## 9. Capa de velocidad (cmd_vel)

- Los productores (`controller`, `joystick`, `initializer`) emiten su velocidad objetivo CRUDA
  a `/*/cmd_vel`. El suavizado (rampa) NO va en el productor ni en el gateway final: va en el
  `MotorGateway`, que hace mux por prioridad (joy>init>nav) + rampa con el perfil de la fuente
  activa + cinematica + clamp de rueda. No hay `/cmd_vel` final como contrato.
