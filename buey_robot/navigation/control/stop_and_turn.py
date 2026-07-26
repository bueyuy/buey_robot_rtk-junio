"""Modo stop_and_turn: encara el waypoint girando en el lugar, avanza, y repite."""

import math
from dataclasses import dataclass

from buey_robot.utils.math import angle_diff
from buey_robot.navigation.control.base import Control


@dataclass
class StopAndTurnParams:
    cruise_linear: float
    cruise_angular: float
    angular_gain: float
    max_angular: float
    min_linear: float
    goal_tolerance: float
    alignment_tolerance: float
    decel_distance: float


class StopAndTurnControl(Control):
    def __init__(self, params: StopAndTurnParams):
        self._p = params
        self._aligning = True

    def reset(self):
        self._aligning = True

    def compute(self, pose, goal):
        x, y, heading = pose
        gx, gy = goal
        heading_err = angle_diff(math.atan2(gy - y, gx - x), heading)

        if self._aligning:                        # encara el WP antes de avanzar
            if abs(heading_err) < self._p.alignment_tolerance:
                self._aligning = False
            else:
                return 0.0, math.copysign(self._p.cruise_angular, heading_err), False, 'encarando WP'

        dist = math.hypot(gx - x, gy - y)
        if dist < self._p.goal_tolerance:
            return 0.0, 0.0, True, 'WP alcanzado'

        lin = (max(self._p.cruise_linear * dist / self._p.decel_distance, self._p.min_linear)
               if dist < self._p.decel_distance else self._p.cruise_linear)
        ang = max(-self._p.max_angular, min(self._p.max_angular, heading_err * self._p.angular_gain))
        return lin, ang, False, f'yendo a WP (dist={dist:.2f}m)'
