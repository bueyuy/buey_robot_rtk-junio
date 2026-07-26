"""Interfaz de un control: dada la pose y el waypoint objetivo, calcula la velocidad."""

from abc import ABC, abstractmethod


class Control(ABC):
    @abstractmethod
    def reset(self):
        """Estado interno a cero. El controller lo llama al arrancar y en cada goal nuevo."""

    @abstractmethod
    def compute(self, pose, goal):
        """Un tick de control. pose=(x, y, heading), goal=(x, y).
        Devuelve (linear, angular, reached, status)."""
