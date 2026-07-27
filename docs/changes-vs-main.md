# Cambios vs `main` (original)

Comparacion funcional entre el `main` original y el stack refactorizado. La logica de
navegacion se preservo (mejor estructurada); estos son los cambios de comportamiento e
interfaz a tener en cuenta (sobre todo para el dashboard web).

## Igual (compatible)

- **Navegacion**: stop_and_turn, waypoint following, lever-arm, fusion de heading, odom RTK.
- **Salida a motores**: identica — `velL&velR` a `bueyuy/navigation/motors` + `v&w` debug a
  `bueyuy/navigation/cmd_vel`.
- **Entradas**: joystick (`bueyuy/navigation/joystick`), GO (`bueyuy/navigation/start`), ruta
  lat/lon (`bueyuy/waypoints`).

## Cambios de comportamiento

1. **No hay auto-navegar al launch.** `main` tenia `goal_source: waypoints_file` +
   `auto_load_waypoints: <archivo>` -> cargaba el YAML y arrancaba solo (sin GO). El actual solo
   acepta ruta por MQTT + GO. Mas seguro (el robot no se mueve al arrancar), pero se pierde la
   conveniencia. Reponer = un modo "cargar waypoints de archivo + auto-GO".
2. **Modo `arc` fuera.** `main` tenia `stop_and_turn` + `arc` (ALIGN->CRUISE->ARC->FINAL_APPROACH,
   curva sin frenar). Estaba en `no-arc` por default. El actual solo tiene `stop_and_turn`, con
   `navigation/control/base.py` (interface) listo para reponer `arc` como otro `Control`.

## Interfaz MQTT (dashboard)

| Concepto | main | actual |
|----------|------|--------|
| Estado de nav | `bueyuy/navigation/status` | logs -> `bueyuy/logs` |
| Origen del frame | web publica `positions_base`/`positions_start` | datum interno (primer fix) |
| Recalibracion gyro | `bueyuy/sensors/imu/calibration` (externa) | interna (initializer) |
| Posicion del robot (mapa) | `rtk/location/json` (lat/lon) | no se publica (hasta `/geo/position`) |
| Echo de ruta / config | `waypoints_xy`, `config`, `config/motors` | no se publican |

**El dashboard necesita adaptarse**: leer el estado desde `bueyuy/logs` (no
`bueyuy/navigation/status`); el mapa pierde la posicion del robot hasta reponer la conversion
inversa `/geo/position` (ver pendientes). Las entradas se simplificaron (ya no manda BASE ni
dispara la calibracion).
