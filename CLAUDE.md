# buey_robot — guia para agentes

Sistema de navegacion autonoma para skid-steer agricola (Buey V), ROS2 Humble.

Antes de tocar codigo, leer:
- [README.md](README.md) — que es, build, uso.
- [docs/architecture.md](docs/architecture.md) — capas, nodos, topics, diagramas.
- [docs/CONVENTIONS.md](docs/CONVENTIONS.md) — reglas de diseño. **Respetarlas.**

Reglas rapidas (detalle en CONVENTIONS.md):
- Un solo punto bilingue lat/lon <-> x/y: `OdometryGps`. Arriba de esa frontera, todo x/y.
- Contratos de topic: fuente unica en `buey_robot/contracts.py` (nada de strings sueltos).
- Config 100% YAML, fail-fast (params sin default). `config/` espeja `buey_robot/`.
- MQTT solo en `adapters/mqtt/`; los demas nodos no importan paho.
- Ausencia = invalido (el productor publica solo si el dato vale; el consumidor infiere por staleness).
- Comentarios KISS y self-contained; logs explicitos con numeros + consecuencia.
- Clase de nodo = PascalCase(carpeta+archivo). SRP con criterio, no sobre-dividir.
