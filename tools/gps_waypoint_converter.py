#!/usr/bin/env python3
"""
GPS Waypoint Converter: Herramienta para convertir waypoints entre formatos GPS y local.

Este script permite:
1. Convertir waypoints locales (x, y en metros) a coordenadas GPS (lat, lon)
2. Convertir waypoints GPS a coordenadas locales
3. Generar archivos YAML en ambos formatos
"""

import argparse
import yaml
import math
from typing import List, Tuple


class GPSConverter:
    """Conversor GPS simple (copiado de gps_utils.py)."""

    def __init__(self):
        self.origin_lat = None
        self.origin_lon = None
        self.origin_set = False
        self.EARTH_RADIUS = 6378137.0

    def set_origin(self, latitude: float, longitude: float):
        self.origin_lat = latitude
        self.origin_lon = longitude
        self.origin_set = True

    def gps_to_local(self, latitude: float, longitude: float) -> Tuple[float, float]:
        if not self.origin_set:
            raise ValueError("Origen no establecido")

        lat_rad = math.radians(latitude)
        lon_rad = math.radians(longitude)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)

        delta_lat = lat_rad - origin_lat_rad
        delta_lon = lon_rad - origin_lon_rad

        x = self.EARTH_RADIUS * delta_lon * math.cos(origin_lat_rad)
        y = self.EARTH_RADIUS * delta_lat

        return x, y

    def local_to_gps(self, x: float, y: float) -> Tuple[float, float]:
        if not self.origin_set:
            raise ValueError("Origen no establecido")

        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)

        delta_lat = y / self.EARTH_RADIUS
        delta_lon = x / (self.EARTH_RADIUS * math.cos(origin_lat_rad))

        lat_rad = origin_lat_rad + delta_lat
        lon_rad = origin_lon_rad + delta_lon

        latitude = math.degrees(lat_rad)
        longitude = math.degrees(lon_rad)

        return latitude, longitude


def local_to_gps_file(input_file: str, output_file: str, origin_lat: float, origin_lon: float):
    """
    Convierte archivo de waypoints locales (x, y) a GPS (lat, lon).

    Args:
        input_file: Archivo YAML con waypoints locales
        output_file: Archivo YAML de salida con waypoints GPS
        origin_lat: Latitud del origen
        origin_lon: Longitud del origen
    """
    # Leer waypoints locales
    with open(input_file, 'r') as f:
        data = yaml.safe_load(f)

    if 'waypoints' not in data:
        print(f"Error: Archivo {input_file} no contiene campo 'waypoints'")
        return False

    # Configurar conversor
    converter = GPSConverter()
    converter.set_origin(origin_lat, origin_lon)

    # Convertir waypoints
    waypoints_gps = []
    print(f"\nOrigen GPS: ({origin_lat:.8f}, {origin_lon:.8f})")
    print(f"\nConvirtiendo {len(data['waypoints'])} waypoints locales → GPS:\n")
    print(f"{'#':<4} {'X (m)':<10} {'Y (m)':<10} {'→':<3} {'Latitude':<15} {'Longitude':<15}")
    print("-" * 70)

    for i, wp in enumerate(data['waypoints']):
        if 'x' in wp and 'y' in wp:
            x = float(wp['x'])
            y = float(wp['y'])

            lat, lon = converter.local_to_gps(x, y)
            waypoints_gps.append({'latitude': lat, 'longitude': lon})

            print(f"{i:<4} {x:<10.2f} {y:<10.2f} {'→':<3} {lat:<15.8f} {lon:<15.8f}")

    # Guardar archivo GPS
    output_data = {'waypoints': waypoints_gps}
    with open(output_file, 'w') as f:
        yaml.dump(output_data, f, default_flow_style=False, sort_keys=False)

    print(f"\n✓ Waypoints GPS guardados en: {output_file}")
    return True


def gps_to_local_file(input_file: str, output_file: str, origin_lat: float, origin_lon: float):
    """
    Convierte archivo de waypoints GPS (lat, lon) a locales (x, y).

    Args:
        input_file: Archivo YAML con waypoints GPS
        output_file: Archivo YAML de salida con waypoints locales
        origin_lat: Latitud del origen
        origin_lon: Longitud del origen
    """
    # Leer waypoints GPS
    with open(input_file, 'r') as f:
        data = yaml.safe_load(f)

    if 'waypoints' not in data:
        print(f"Error: Archivo {input_file} no contiene campo 'waypoints'")
        return False

    # Configurar conversor
    converter = GPSConverter()
    converter.set_origin(origin_lat, origin_lon)

    # Convertir waypoints
    waypoints_local = []
    print(f"\nOrigen GPS: ({origin_lat:.8f}, {origin_lon:.8f})")
    print(f"\nConvirtiendo {len(data['waypoints'])} waypoints GPS → locales:\n")
    print(f"{'#':<4} {'Latitude':<15} {'Longitude':<15} {'→':<3} {'X (m)':<10} {'Y (m)':<10}")
    print("-" * 70)

    for i, wp in enumerate(data['waypoints']):
        if 'latitude' in wp and 'longitude' in wp:
            lat = float(wp['latitude'])
            lon = float(wp['longitude'])

            x, y = converter.gps_to_local(lat, lon)
            waypoints_local.append({'x': x, 'y': y})

            print(f"{i:<4} {lat:<15.8f} {lon:<15.8f} {'→':<3} {x:<10.2f} {y:<10.2f}")

    # Guardar archivo local
    output_data = {'waypoints': waypoints_local}
    with open(output_file, 'w') as f:
        yaml.dump(output_data, f, default_flow_style=False, sort_keys=False)

    print(f"\n✓ Waypoints locales guardados en: {output_file}")
    return True


def main():
    parser = argparse.ArgumentParser(
        description='Convierte waypoints entre formatos GPS y local',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos de uso:

  # Convertir waypoints locales a GPS
  python3 gps_waypoint_converter.py \\
      --local-to-gps \\
      --input waypoints/rectangulo_local.yaml \\
      --output waypoints/rectangulo_gps.yaml \\
      --origin-lat -34.603722 \\
      --origin-lon -58.381592

  # Convertir waypoints GPS a locales
  python3 gps_waypoint_converter.py \\
      --gps-to-local \\
      --input waypoints/rectangulo_gps.yaml \\
      --output waypoints/rectangulo_local.yaml \\
      --origin-lat -34.603722 \\
      --origin-lon -58.381592

Notas:
  - El origen debe ser el mismo punto donde el robot inicia
  - Para GPS RTK, usar coordenadas con 8 decimales de precisión
  - El formato local usa metros (X = Este, Y = Norte)
        """
    )

    parser.add_argument('--local-to-gps', action='store_true',
                        help='Convertir de coordenadas locales a GPS')
    parser.add_argument('--gps-to-local', action='store_true',
                        help='Convertir de coordenadas GPS a locales')
    parser.add_argument('-i', '--input', required=True,
                        help='Archivo YAML de entrada')
    parser.add_argument('-o', '--output', required=True,
                        help='Archivo YAML de salida')
    parser.add_argument('--origin-lat', type=float, required=True,
                        help='Latitud del origen en grados')
    parser.add_argument('--origin-lon', type=float, required=True,
                        help='Longitud del origen en grados')

    args = parser.parse_args()

    # Validar que se especificó una dirección
    if not args.local_to_gps and not args.gps_to_local:
        print("Error: Debes especificar --local-to-gps o --gps-to-local")
        return 1

    if args.local_to_gps and args.gps_to_local:
        print("Error: Solo puedes especificar una dirección de conversión")
        return 1

    # Ejecutar conversión
    try:
        if args.local_to_gps:
            success = local_to_gps_file(
                args.input,
                args.output,
                args.origin_lat,
                args.origin_lon
            )
        else:
            success = gps_to_local_file(
                args.input,
                args.output,
                args.origin_lat,
                args.origin_lon
            )

        return 0 if success else 1

    except FileNotFoundError as e:
        print(f"Error: Archivo no encontrado: {e}")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    import sys
    sys.exit(main())
