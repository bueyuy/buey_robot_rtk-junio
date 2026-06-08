# Motor Filters — Guia de activacion y tuning

El motor gateway tiene un pipeline de filtros para suavizar los comandos a los motores.
Actualmente solo **SoftDeadzone** esta activo. EMA y SlewRate estan como stubs listos para activar.

## Pipeline actual

```
cmd_vel -> gains -> kinematics -> clamp -> [SoftDeadzone] -> MQTT
```

## SoftDeadzone (ACTIVO)

Reemplaza el snap duro que causaba sacudones en la deadzone del motor.

**Comportamiento:**
- `|val| < low` (5) -> 0 (zona muerta real, motor no se mueve)
- `|val| in [low, high]` (5-20) -> rampa lineal suave
- `|val| > high` (20) -> pass through sin modificar

**Parametros en `config/motor.yaml`:**
```yaml
soft_deadzone:
  low: 5.0    # Por debajo de esto, motor apagado
  high: 20.0  # Por encima de esto, valor directo
```

**Es una transformacion estatica** (mapeo input->output), NO una rampa temporal.
No agrega latencia ni interactua con la RampProfile del controlador de navegacion.

---

## EMAFilter (FUTURO — pass-through)

Filtro de media movil exponencial. Suaviza picos en los comandos de motor.

**Cuando activar:** Si cmd_vel tiene picos (ej: Nav2 con planificador agresivo, joystick con ruido).

**Parametro:** `ema_alpha` (0-1). Menor = mas suavizado.
- `0.3` — suavizado fuerte, respuesta lenta
- `0.5` — balance tipico
- `0.7` — suavizado ligero, respuesta rapida

**Formula:** `output = alpha * input + (1 - alpha) * output_anterior`

**Para activar:** Implementar el metodo `apply()` en `motor/motor_filters.py:EMAFilter`
y agregar al pipeline en `motor_gateway.py` despues del clamp.

---

## SlewRateLimiter (FUTURO — pass-through)

Limita el cambio maximo (delta) de comando entre ciclos consecutivos.

**Cuando activar:** Si los motores vibran por cambios bruscos, incluso despues de activar EMA.

**Parametros:**
- `slew_rate` — delta maximo por ciclo en movimiento (tipico: 10-15 a 20Hz)
- `slew_rate_stop` — delta maximo al frenar (tipico: 20-30, mas agresivo para frenar rapido)

**Para activar:** Implementar el metodo `apply()` en `motor/motor_filters.py:SlewRateLimiter`
y agregar al pipeline en `motor_gateway.py` despues de EMA.

---

## Guia de tuning

1. **Robot sacude al arrancar/frenar:** Verificar que SoftDeadzone esta activa y `low`/`high` son correctos.
2. **Robot sacude durante navegacion:** Activar EMA con `alpha=0.5` primero.
3. **Sigue sacudiendo con EMA:** Agregar SlewRate con `slew_rate=10`.
4. **Robot responde lento:** Subir `alpha` (ej: 0.7) o subir `slew_rate`.
5. **Motores vibran a baja velocidad:** Bajar `soft_deadzone.high` para que la rampa sea mas corta.

## Referencia

Ver `motor_gateway_spec.md` para el analisis original del pipeline y las metricas.
