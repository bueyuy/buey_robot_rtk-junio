"""Conversion GPS (lat/lon) -> coordenadas locales x/y (proyeccion ENU, sin deps externas)."""

import math

_EARTH_RADIUS = 6378137.0


class GPSConverter:
    def __init__(self):
        self.origin_lat = None
        self.origin_lon = None
        self.origin_set = False

    def set_origin(self, latitude: float, longitude: float):
        self.origin_lat = latitude
        self.origin_lon = longitude
        self.origin_set = True

    def gps_to_local(self, latitude: float, longitude: float) -> tuple:
        """(x=Este, y=Norte) en metros respecto al origen. Requiere set_origin() antes."""
        if not self.origin_set:
            raise ValueError("Origen no establecido")
        origin_lat_rad = math.radians(self.origin_lat)
        x = _EARTH_RADIUS * math.radians(longitude - self.origin_lon) * math.cos(origin_lat_rad)
        y = _EARTH_RADIUS * math.radians(latitude - self.origin_lat)
        return x, y

    def local_to_gps(self, x: float, y: float) -> tuple:
        """(lat, lon) desde x/y locales (inversa de gps_to_local). Requiere set_origin() antes."""
        if not self.origin_set:
            raise ValueError("Origen no establecido")
        origin_lat_rad = math.radians(self.origin_lat)
        lat = self.origin_lat + math.degrees(y / _EARTH_RADIUS)
        lon = self.origin_lon + math.degrees(x / (_EARTH_RADIUS * math.cos(origin_lat_rad)))
        return lat, lon
