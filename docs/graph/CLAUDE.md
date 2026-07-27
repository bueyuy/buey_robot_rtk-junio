# CLAUDE.md — diagrama de nodos de buey_robot

Genera el diagrama de arquitectura ROS 2 de buey_robot: cada nodo es una caja
con sus tópicos de entrada y salida, y los cables van de socket a socket.

## Regla principal

**Solo se edita `graph.spec.json`.** Nunca se editan los `.svg` ni se tocan
coordenadas a mano. El layout (posición de cajas y ruteo de cables) lo calcula
ELK; los `render*.mjs` solo dibujan lo que ELK devuelve.

Si el resultado no gusta, se ajusta una opción de ELK en el `render*.mjs`, no
el SVG generado.

## Donde vive

```
docs/
  ARCHITECTURE.md          <- documento final, con marcadores graph:start/end
  graph/
    graph.spec.json        <- LO UNICO QUE SE EDITA A MANO
    render.mjs             <- spec -> SVG (layout con ELK, nada a mano)
    sync-docs.mjs          <- vuelca el SVG y las tablas en ARCHITECTURE.md
    buey_graph.svg         <- generado, SE COMMITEA (GitHub lo sirve del repo)
    package.json
```

El `.svg` va versionado: GitHub lo renderiza desde el repo, no lo genera.

## Build

```bash
npm install     # una sola vez
npm run build   # graph.spec.json -> buey_graph.svg + tablas en ARCHITECTURE.md
```

El build **falla con exit 1** si el spec tiene errores o si algún cable no
termina exactamente en el centro de su socket. Si sale `ok: todos los cables
alineados con su socket`, está bien. Un fallo hay que arreglarlo, no ignorarlo.


## Estructura del spec

```jsonc
{
  "layers": ["entradas", "estimacion", ...],   // una fila por capa, en orden
  "roles":  { "sensor": { "color": "#5AC8A8", "desc": "..." }, ... },
  "nodes":  [ { "id", "layer", "role", "in": [], "out": [] } ],
  "edges":  [ ["NodoOrigen", "/topico", "NodoDestino", "/topico"] ]
}
```

- `layer` es el índice dentro de `layers`. Define en qué fila cae el nodo.
- `role` define el color de la caja y de los cables que salen de ella.
- `in` / `out` son los tópicos ROS. El orden dentro del array es el orden en
  que se dibujan las pills, izquierda a derecha.
- Un cable es `[origen, topico_out, destino, topico_in]`. Los dos tópicos
  tienen que existir en el `out` del origen y en el `in` del destino.
- Quinto elemento opcional `"feedback"` para un cable que va contra el flujo
  (de una fila de abajo hacia una de arriba). Se dibuja punteado y por afuera.

## Cómo agregar un nodo

Ejemplo real: agregar `TelemetryBridge`, que escucha `/odom` y `/heading/fused`
y los espeja a MQTT.

1. Agregarlo a `nodes`. Va en una fila **por debajo** de quienes le publican:

```json
{ "id": "TelemetryBridge", "layer": 5, "role": "comando",
  "in": ["/odom", "/heading/fused"], "out": ["MQTT"],
  "file": "adapters/mqtt/telemetry_bridge.py",
  "desc": "espeja el estado del stack a la web" }
```

2. Agregar sus cables a `edges`:

```json
["OdometryGps", "/odom", "TelemetryBridge", "/odom"],
["FusionHeading", "/heading/fused", "TelemetryBridge", "/heading/fused"]
```

3. `npm run build` y verificar que no haya errores.

## Errores que reporta el build

- `no existe el nodo origen "X"` — typo en el nombre, o el nodo no está en `nodes`.
- `"X" no publica "/t"` — falta el tópico en el `out` de ese nodo.
- `"X" e "Y" estan en la misma fila` — un cable no puede ir entre dos nodos de
  la misma capa. Hay que mover uno de fila.
- `aviso: "X" publica "/t" y nadie lo escucha` — no rompe nada, es un tópico
  suelto. Normal para salidas hacia afuera del grafo.

## Lo que NO hay que hacer

- No editar los `.svg`: se pisan en el próximo build.
- No editar lo que hay entre `<!-- graph:start -->` y `<!-- graph:end -->` en
  `ARCHITECTURE.md`: también se pisa. La prosa va fuera de los marcadores.
- No agregar un nodo sin `file` y `desc`: son las columnas de la tabla generada.
- No agregar coordenadas al spec. No existen y no se van a respetar.
- No poner un cable entre dos nodos de la misma fila. Si dos nodos se hablan
  entre sí, uno está mal ubicado.
- No cambiar `elk.portConstraints`: `FIXED_POS` es lo que hace que las pills
  coincidan con los puertos. Tocarlo desalinea todo.

## Como llega al ARCHITECTURE.md

`sync-docs.mjs` reemplaza lo que haya entre estos dos marcadores de
`docs/ARCHITECTURE.md`:

```markdown
<!-- graph:start -->
<!-- graph:end -->
```

Ahi mete la imagen, la tabla de nodos y la tabla de topicos, todo derivado del
spec. El resto del documento no se toca: la prosa que escribas alrededor
sobrevive a cualquier build. Es idempotente, correrlo dos veces no cambia nada.

El link a la imagen es **relativo** (`graph/buey_graph.svg`). Asi
funciona en cualquier branch, fork o PR. Un link absoluto a
`raw.githubusercontent.com` apunta siempre a una branch fija y se rompe en los
forks; usalo solo si necesitas embeber la imagen fuera de GitHub.

**No** pegar el `<svg>` inline en markdown: GitHub lo sanitiza y no se ve.

El SVG está preparado para eso: fondo propio (se ve igual en tema claro y
oscuro), sin `<script>` ni handlers que el sanitizador borre, y cada texto
lleva `textLength` declarado, así el navegador lo ajusta al ancho que asumió
el layout aunque el lector no tenga la misma fuente.

Si agregás un nodo con un tópico muy largo, revisá que el margen texto->pill
siga siendo positivo: el ancho de caja se calcula con `W.pill = 6.15` px por
carácter en `render*.mjs`.

## Notas de diseño (por qué está así)

- **Filas por capa**: `elk.partitioning.activate` fuerza cada nodo a su fila.
- **Las filas no están centradas entre sí, a propósito.** Se probó centrarlas y
  cuesta caro: un cable que iba recto entre dos filas desplazadas distinto tiene
  que ganar un escalón. Medido sobre este grafo, centrar pasa de 8 cables rectos
  a 0 y de 20 quiebres a 38. Se prefirió el trazado limpio y se aceptan hasta
  ~120px de descentrado entre filas.
- Cada `<path>` del SVG lleva `data-edge="Origen:/topico -> Destino"`, útil
  para inspeccionar o para estilar un cable puntual desde CSS.
