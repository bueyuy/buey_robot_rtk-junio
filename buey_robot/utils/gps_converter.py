"""Conversion de coordenadas GPS a sistema local ENU.

Proyeccion ENU (East-North-Up) simple, sin dependencias externas.
"""

import math
from typing import Tuple, Optional


class GPSConverter:
    """Convierte coordenadas GPS (lat/lon) a coordenadas locales Cartesianas (X, Y)
    usando proyeccion ENU (East-North-Up).
    """

    def __init__(self):
        self.origin_lat: Optional[float] = None
        self.origin_lon: Optional[float] = None
        self.origin_set = False
        self.EARTH_RADIUS = 6378137.0

    def set_origin(self, latitude: float, longitude: float):
        """Establece el origen del sistema de coordenadas locales."""
        self.origin_lat = latitude
        self.origin_lon = longitude
        self.origin_set = True

    def gps_to_local(self, latitude: float, longitude: float) -> Tuple[float, float]:
        """Convierte GPS a coordenadas locales (X=Este, Y=Norte) en metros.

        Raises:
            ValueError: Si el origen no ha sido establecido.
        """
        if not self.origin_set:
            raise ValueError("Origen no establecido. Llama a set_origin() primero.")

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
        """Convierte coordenadas locales (X, Y) en metros a GPS (lat, lon)."""
        if not self.origin_set:
            raise ValueError("Origen no establecido. Llama a set_origin() primero.")

        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)

        delta_lat = y / self.EARTH_RADIUS
        delta_lon = x / (self.EARTH_RADIUS * math.cos(origin_lat_rad))

        lat_rad = origin_lat_rad + delta_lat
        lon_rad = origin_lon_rad + delta_lon

        return math.degrees(lat_rad), math.degrees(lon_rad)

    def distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Distancia en metros entre dos puntos GPS (formula de Haversine)."""
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad

        a = math.sin(dlat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return self.EARTH_RADIUS * c

    def bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Rumbo (bearing) desde el punto 1 al punto 2, en radianes."""
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        dlon = lon2_rad - lon1_rad
        x = math.sin(dlon) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)

        return math.atan2(x, y)
