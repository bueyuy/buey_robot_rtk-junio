#!/usr/bin/env python3
"""Ajuste de elipse al scatter mag (x,y) para calibracion hard/soft-iron.

Lee el CSV del logger, filtra glitches, ajusta una elipse por minimos cuadrados,
extrae centro (hard-iron) y la matriz de correccion (soft-iron), y reporta:
  - calidad del ajuste (residual RMS relativo)
  - cobertura angular del heading CRUDO vs CALIBRADO (debe barrer ~360)
"""
import csv
import math
import numpy as np

CSV = '/home/nexlab/imu_cal/imu_cal_log.csv'
GLITCH = 1500.0


def load():
    xs, ys, vs = [], [], []
    with open(CSV, errors='ignore') as f:
        lines = (ln.replace('\x00', '') for ln in f)
        for row in csv.DictReader(lines):
            try:
                mx = float(row['mag_x']); my = float(row['mag_y'])
                v = float(row['v']) if row['v'] else 0.0
            except (ValueError, KeyError):
                continue
            if abs(mx) > GLITCH or abs(my) > GLITCH:
                continue
            xs.append(mx); ys.append(my); vs.append(v)
    return np.array(xs), np.array(ys), np.array(vs)


def fit_ellipse(x, y):
    """Ajuste algebraico general de conica: ax^2+bxy+cy^2+dx+ey+f=0 (SVD)."""
    D = np.column_stack([x*x, x*y, y*y, x, y, np.ones_like(x)])
    _, _, Vt = np.linalg.svd(D)
    a, b, c, d, e, f = Vt[-1]
    # centro
    M = np.array([[2*a, b], [b, 2*c]])
    cx, cy = np.linalg.solve(M, [-d, -e])
    # forma: ejes y rotacion desde la submatriz cuadratica
    A = np.array([[a, b/2], [b/2, c]])
    evals, evecs = np.linalg.eigh(A)
    # constante en el centro
    fc = a*cx*cx + b*cx*cy + c*cy*cy + d*cx + e*cy + f
    axes = np.sqrt(-fc / evals)  # semi-ejes
    return (cx, cy), axes, evecs


def _coverage(ang):
    """% de los 360 cubiertos (36 bins de 10 grados)."""
    hist, _ = np.histogram(ang % 360, bins=36, range=(0, 360))
    return np.sum(hist > 0) / 36 * 100


def compute_calibration(x, y):
    """Ajusta la elipse a los puntos mag (x,y) ya filtrados de glitch y devuelve
    los parametros hard/soft-iron listos para config/imu.yaml + metricas de calidad.

    Devuelve dict:
      hard_iron     [cx, cy]                      offset a restar
      soft_iron     [a, b, c, d]                  matriz W 2x2 fila-mayor [[a,b],[c,d]]
                                                  que consume imu_compass:
                                                  cal_x=a*dx+b*dy, cal_y=c*dx+d*dy
      ratio         mayor/menor                   1.0 = circulo perfecto
      residual      RMS (0 = perfecto)
      coverage_raw  % de 360 cubiertos sin calibrar
      coverage_cal  % de 360 cubiertos calibrado (deberia ~100 si se giro completo)
      n             cantidad de puntos
      center, axes  para reporte
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    (cx, cy), axes, evecs = fit_ellipse(x, y)
    ax_major, ax_minor = max(axes), min(axes)

    # residual: distancia normalizada de cada punto a la elipse
    xc, yc = x - cx, y - cy
    P = evecs.T @ np.vstack([xc, yc])
    rr = (P[0]/axes[0])**2 + (P[1]/axes[1])**2  # =1 sobre la elipse
    resid = float(np.sqrt(np.mean((rr - 1.0)**2)))

    # matriz de correccion: rotar a ejes, escalar a circulo, rotar de vuelta
    W = evecs @ np.diag([ax_major/axes[0], ax_major/axes[1]]) @ evecs.T
    cal = W @ np.vstack([xc, yc])

    return {
        'hard_iron': [float(cx), float(cy)],
        'soft_iron': [float(W[0, 0]), float(W[0, 1]),
                      float(W[1, 0]), float(W[1, 1])],
        'ratio': float(ax_major / ax_minor),
        'residual': resid,
        'coverage_raw': float(_coverage(np.degrees(np.arctan2(y, x)))),
        'coverage_cal': float(_coverage(np.degrees(np.arctan2(cal[1], cal[0])))),
        'n': int(len(x)),
        'center': (float(cx), float(cy)),
        'axes': (float(ax_major), float(ax_minor)),
    }


def main():
    x, y, v = load()
    print(f'puntos validos (sin glitch): {len(x)}')
    if len(x) < 30:
        print('muy pocos puntos'); return

    r = compute_calibration(x, y)
    cx, cy = r['center']
    ax_major, ax_minor = r['axes']
    w = r['soft_iron']
    print(f'\n=== ELIPSE AJUSTADA ===')
    print(f'centro (hard-iron offset) = ({cx:.1f}, {cy:.1f})')
    print(f'semi-ejes mayor={ax_major:.1f}, menor={ax_minor:.1f}')
    print(f'ratio mayor/menor (soft-iron) = {r["ratio"]:.3f}   (1.0 = circulo perfecto)')
    print(f'residual RMS (0=perfecto) = {r["residual"]:.3f}')

    print(f'\n=== COBERTURA ANGULAR (girando deberia ser ~100%) ===')
    print(f'heading CRUDO:     {r["coverage_raw"]:.0f}% de los 360 cubiertos')
    print(f'heading CALIBRADO: {r["coverage_cal"]:.0f}% de los 360 cubiertos')

    # matriz de calibracion para el nodo ROS:  m_cal = W @ (m_raw - offset)
    print(f'\n=== PARAMS PARA config/imu.yaml ===')
    print(f'hard_iron: [{cx:.2f}, {cy:.2f}]')
    print(f'soft_iron (2x2): [[{w[0]:.4f}, {w[1]:.4f}], [{w[2]:.4f}, {w[3]:.4f}]]')


if __name__ == '__main__':
    main()
