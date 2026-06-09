#!/usr/bin/env python3
"""Calibracion hard/soft-iron en vivo: captura mag, calcula y escribe imu.yaml.

Reemplaza el flujo de 2 pasos (imu_cal_logger.py -> CSV -> fit_ellipse.py ->
copiar a mano). Se suscribe a /imu/mag (sensor_msgs/MagneticField, campo crudo
del firmware micro-ROS), captura durante SECS segundos mientras se gira el robot,
ajusta una elipse (misma matematica que fit_ellipse.py) y escribe los parametros
hard_iron / soft_iron directamente en config/imu.yaml (con backup).

ARRANQUE: por defecto NO arranca al ejecutar; queda esperando un publish en el
topic MQTT 'imu_calibration' (bueyuy/sensors/imu/calibration, configurable en
mqtt.yaml) para que el dashboard web dispare la calibracion. El payload puede
traer {"secs": N} para override de la duracion; cualquier mensaje dispara. Con
--now arranca de una sin esperar MQTT (util para probar a mano).

IMPORTANTE: hay que GIRAR el robot 4-5 vueltas lentas durante la captura para
cubrir los 360 grados; si no, la cobertura angular queda baja y el ajuste sale
malo. Ver docs/imu-microros-calibracion.md.

Uso:
    python3 tools/imu_cal/imu_cal_live.py [--secs 60] [--topic /imu/mag] [--now]
"""
import argparse
import os
import re
import shutil
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

from fit_ellipse import compute_calibration

GLITCH = 1500.0  # descartar lecturas mag con |componente| mayor (glitch I2C)
MIN_POINTS = 30  # mismo umbral que fit_ellipse.main()
MIN_COVERAGE = 80.0  # % de 360 cubiertos minimo para escribir el YAML

# tools/imu_cal/ -> ../../config/imu.yaml (mismo dev path que utils/config.py)
IMU_YAML = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'imu.yaml'))


class LiveCalibrator(Node):
    def __init__(self, topic):
        super().__init__('imu_cal_live')
        self.mag_x = []
        self.mag_y = []
        self.n_glitch = 0
        self.collecting = False  # se enciende al disparar la captura
        self.create_subscription(MagneticField, topic, self._mag, 10)
        print(f'[imu_cal_live] suscrito a {topic}', flush=True)

    def _mag(self, m):
        # Ignorar datos hasta que arranque la ventana de captura.
        if not self.collecting:
            return
        mx = m.magnetic_field.x
        my = m.magnetic_field.y
        mz = m.magnetic_field.z
        if abs(mx) > GLITCH or abs(my) > GLITCH or abs(mz) > GLITCH:
            self.n_glitch += 1
            return
        self.mag_x.append(mx)
        self.mag_y.append(my)


def _read_old_params(path):
    """Lee hard_iron/soft_iron actuales del YAML (para comparar). dict o {}."""
    try:
        import yaml
        with open(path) as f:
            cfg = yaml.safe_load(f)
        p = cfg['imu_compass']['ros__parameters']
        return {'hard_iron': p.get('hard_iron'), 'soft_iron': p.get('soft_iron')}
    except Exception:
        return {}


def _fmt_list(vals, prec):
    return '[' + ', '.join(f'{v:.{prec}f}' for v in vals) + ']'


def write_yaml(path, hard_iron, soft_iron, note):
    """Escribe hard_iron/soft_iron por reemplazo de linea (preserva comentarios).

    Hace backup path.bak-<timestamp> antes de tocar el archivo.
    """
    stamp = time.strftime('%Y%m%d-%H%M%S')
    backup = f'{path}.bak-{stamp}'
    shutil.copy2(path, backup)
    print(f'[imu_cal_live] backup -> {backup}', flush=True)

    with open(path) as f:
        text = f.read()

    hi_str = _fmt_list(hard_iron, 2)
    si_str = _fmt_list(soft_iron, 4)

    # Reemplazar solo el array, preservando indentacion y comentario inline.
    text, n_hi = re.subn(r'(?m)^(\s*hard_iron:\s*)\[[^\]]*\]',
                         lambda mo: mo.group(1) + hi_str, text)
    text, n_si = re.subn(r'(?m)^(\s*soft_iron:\s*)\[[^\]]*\]',
                         lambda mo: mo.group(1) + si_str, text)
    if n_hi != 1 or n_si != 1:
        raise RuntimeError(
            f'no pude ubicar hard_iron/soft_iron en {path} '
            f'(hard={n_hi}, soft={n_si}); revisa el formato del YAML')

    # Nota de recaptura: insertar como linea de comentario propia encima de
    # hard_iron, SIN tocar el bloque historico de comentarios (es contexto
    # valioso). Primero borrar una nota previa de imu_cal_live para no acumular.
    fecha = time.strftime('%Y-%m-%d')
    text = re.sub(r'(?m)^\s*#\s*Recapturado en vivo \(imu_cal_live\)[^\n]*\n',
                  '', text)
    text, n_note = re.subn(
        r'(?m)^(\s*)(hard_iron:)',
        lambda mo: f'{mo.group(1)}# {note.format(fecha=fecha)}\n'
                   f'{mo.group(1)}{mo.group(2)}', text)
    if n_note != 1:
        raise RuntimeError(f'no pude insertar la nota en {path}')

    with open(path, 'w') as f:
        f.write(text)
    print(f'[imu_cal_live] escrito -> {path}', flush=True)


def wait_for_mqtt_trigger(node, secs):
    """Bloquea hasta recibir un publish en el topic MQTT imu_calibration.

    Usa el cliente MQTT singleton del proyecto (get_client). El payload puede
    traer un JSON con {"secs": N} para override de la duracion; cualquier mensaje
    dispara el arranque. Devuelve los segundos de captura a usar.

    Crash con mensaje claro si no se puede importar buey_robot (env sin sourcear).
    """
    try:
        from buey_robot.utils.config import load_config, require_key
        from buey_robot.adapters.mqtt.client import get_client
    except ImportError as e:
        raise SystemExit(
            f'[imu_cal_live] no pude importar buey_robot ({e}).\n'
            '  Sourcea el workspace (source install/setup.bash) o corre con --now '
            'para arrancar sin esperar MQTT.')

    cfg = load_config('mqtt.yaml')
    topic = require_key(cfg, 'topics', 'imu_calibration')

    trigger = threading.Event()
    box = {'secs': secs}

    def _on_trigger(client, userdata, msg):
        try:
            import json
            payload = json.loads(msg.payload.decode())
            if isinstance(payload, dict) and 'secs' in payload:
                box['secs'] = float(payload['secs'])
        except Exception:
            pass  # cualquier mensaje (aunque no sea JSON) dispara igual
        trigger.set()

    mqtt = get_client(cfg, logger=node.get_logger())
    mqtt.subscribe(topic, _on_trigger)
    print(f'[imu_cal_live] esperando trigger MQTT en "{topic}" '
          f'(Ctrl+C para abortar)...', flush=True)

    waited = 0.0
    while not trigger.is_set():
        rclpy.spin_once(node, timeout_sec=0.2)
        waited += 0.2
        if waited % 5.0 < 0.2:
            estado = 'conectado' if mqtt.is_connected else 'SIN conexion al broker'
            print(f'[imu_cal_live] ...esperando ({estado})', flush=True)
    print(f'[imu_cal_live] TRIGGER recibido -> arrancando captura', flush=True)
    return box['secs']


def main():
    ap = argparse.ArgumentParser(description='Calibracion IMU hard/soft-iron en vivo')
    ap.add_argument('--secs', type=float, default=60.0, help='duracion captura (s)')
    ap.add_argument('--topic', default='/imu/mag', help='topic MagneticField crudo')
    ap.add_argument('--now', action='store_true',
                    help='arrancar ya, sin esperar el trigger MQTT (para pruebas)')
    args = ap.parse_args()

    rclpy.init()
    node = LiveCalibrator(args.topic)

    # Por defecto espera el publish del dashboard en bueyuy/sensors/imu/calibration.
    secs = args.secs
    if not args.now:
        secs = wait_for_mqtt_trigger(node, args.secs)

    node.collecting = True
    t0 = node.get_clock().now()
    next_report = 0.0
    print(f'[imu_cal_live] capturando {secs:.0f}s -> GIRA EL ROBOT 4-5 vueltas lentas',
          flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = (node.get_clock().now() - t0).nanoseconds * 1e-9
            if elapsed >= secs:
                break
            if elapsed >= next_report:
                next_report += 2.0
                n = len(node.mag_x)
                cov = ''
                if n >= 5:
                    import numpy as np
                    ang = np.degrees(np.arctan2(node.mag_y, node.mag_x))
                    hist, _ = np.histogram(ang % 360, bins=36, range=(0, 360))
                    cov = f' cobertura~{np.sum(hist > 0) / 36 * 100:.0f}%'
                print(f'[t={elapsed:5.1f}/{secs:.0f}] n={n} '
                      f'glitch={node.n_glitch}{cov}  (sigue girando)', flush=True)
    except KeyboardInterrupt:
        print('\n[imu_cal_live] interrumpido', flush=True)

    x, y = list(node.mag_x), list(node.mag_y)
    old = _read_old_params(IMU_YAML)
    node.destroy_node()
    rclpy.shutdown()

    print(f'\n=== CAPTURA TERMINADA: {len(x)} puntos validos '
          f'({node.n_glitch} glitches) ===', flush=True)
    if len(x) < MIN_POINTS:
        print(f'ABORTADO: muy pocos puntos (<{MIN_POINTS}). No se escribe el YAML.')
        return

    r = compute_calibration(x, y)
    print(f'centro (hard_iron) = ({r["hard_iron"][0]:.1f}, {r["hard_iron"][1]:.1f})')
    print(f'ratio soft-iron    = {r["ratio"]:.3f}   residual = {r["residual"]:.3f}')
    print(f'cobertura CRUDA={r["coverage_raw"]:.0f}%  '
          f'CALIBRADA={r["coverage_cal"]:.0f}% (deberia ~100%)')

    print('\n=== NUEVO vs VIEJO ===')
    print(f'hard_iron  nuevo={_fmt_list(r["hard_iron"], 2)}  '
          f'viejo={old.get("hard_iron")}')
    print(f'soft_iron  nuevo={_fmt_list(r["soft_iron"], 4)}  '
          f'viejo={old.get("soft_iron")}')

    if r['coverage_cal'] < MIN_COVERAGE:
        print(f'\nABORTADO: cobertura calibrada {r["coverage_cal"]:.0f}% < '
              f'{MIN_COVERAGE:.0f}% (no giraste lo suficiente). No se escribe el YAML.')
        return

    note = ('Recapturado en vivo (imu_cal_live) {fecha}. '
            f'n={r["n"]}, residual={r["residual"]:.2f}, '
            f'cobertura_cal={r["coverage_cal"]:.0f}%.')
    write_yaml(IMU_YAML, r['hard_iron'], r['soft_iron'], note)
    print('\nListo. Rebuild + relanzar imu_compass para aplicar. '
          'Si en campo empeora, restaura desde el .bak-*')


if __name__ == '__main__':
    main()
