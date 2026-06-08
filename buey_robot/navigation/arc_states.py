"""Mixin con la maquina de estados ARC para el TrajectoryController.

Estados:
  ALIGN         -> girar en el lugar hasta alinear heading al WP
  CRUISE        -> avanzar recto con micro-correccion angular
  ARC           -> girar con velocidad angular constante mientras avanza
  FINAL_APPROACH-> stop_and_turn al ultimo waypoint

Se usa en navigation/controller.py via herencia:
  class TrajectoryController(Node, ArcStateMixin):
"""

import math

from buey_robot.utils.math import angle_diff


class ArcStateMixin:
    """Mixin de estados ARC. Requiere que la clase que lo use tenga los atributos
    del TrajectoryController (ramp_linear, ramp_angular, wp_manager, etc.).
    """

    def _handle_wp_reached(self):
        """Logica comun al alcanzar un waypoint: reset ramps + advance."""
        self.ramp_linear.reset()
        self.ramp_angular.reset()
        self.get_logger().info(f'WP {self.wp_manager.progress_string()} alcanzado')
        self.wp_manager.advance()

    def _control_arc(self, goal_x: float, goal_y: float):
        """Modo arc: mantiene movimiento lineal durante giros para preservar visual odometry."""
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        heading_error = angle_diff(angle_to_goal, self.current_heading)

        if self._arc_state == 'ALIGN':
            self._handle_arc_align(heading_error)
            if self._arc_state == 'ALIGN':
                return

        if self._arc_state == 'CRUISE':
            done = self._handle_arc_cruise(distance, heading_error)
            if done:
                return

        if self._arc_state == 'ARC':
            self._handle_arc_turn()
            return

        if self._arc_state == 'FINAL_APPROACH':
            self._handle_arc_final(distance, heading_error)

    def _handle_arc_align(self, heading_error: float):
        self.ramp_linear.reset()
        if abs(heading_error) < self.alignment_tolerance:
            self._arc_state = 'CRUISE'
            self.get_logger().info(
                f'ArcAlign completado al WP {self.wp_manager.progress_string()} '
                f'(error={math.degrees(heading_error):.1f} deg) - CRUISE')
        else:
            angular_vel = math.copysign(self.cruise_angular, heading_error)
            self._publish_velocity(0.0, angular_vel)
            self._publish_status(
                f'ArcAlign WP {self.wp_manager.progress_string()}, '
                f'heading_err={math.degrees(heading_error):.1f} deg')

    def _handle_arc_cruise(self, distance: float, heading_error: float) -> bool:
        """Retorna True si hay que hacer return en el caller."""
        if distance < self.goal_tolerance:
            self._handle_wp_reached()
            self._arc_state = 'ALIGN'
            return True

        if distance < self.arc_start_distance:
            next_wp = self.wp_manager.peek_next_goal()
            if next_wp is not None:
                self._arc_state = 'ARC'
                self.get_logger().info(
                    f'ArcCruise -> ARC a dist={distance:.2f}m del '
                    f'WP {self.wp_manager.progress_string()}')
            else:
                self._arc_state = 'FINAL_APPROACH'
                self._final_aligning = False
                self.get_logger().info('ArcCruise -> FINAL_APPROACH (ultimo WP)')

        if self._arc_state == 'CRUISE':
            linear_vel = self.ramp_linear.apply(self.cruise_linear)
            target_angular = heading_error * self.angular_gain
            target_angular = max(-self.max_angular, min(self.max_angular, target_angular))
            angular_vel = self.ramp_angular.apply(target_angular)
            self._publish_velocity(linear_vel, angular_vel)
            self._publish_status(
                f'ArcCruise WP {self.wp_manager.progress_string()}, '
                f'dist={distance:.2f}m, heading_err={math.degrees(heading_error):.1f} deg')
            return True

        return False  # state changed, continue in caller

    def _handle_arc_turn(self):
        next_wp = self.wp_manager.peek_next_goal()
        if next_wp is None:
            self._arc_state = 'FINAL_APPROACH'
            self._final_aligning = False
            return

        next_dx = next_wp[0] - self.current_x
        next_dy = next_wp[1] - self.current_y
        angle_to_next = math.atan2(next_dy, next_dx)
        heading_error_next = angle_diff(angle_to_next, self.current_heading)

        if abs(heading_error_next) < self.alignment_tolerance:
            self.get_logger().info(f'WP {self.wp_manager.progress_string()} alcanzado')
            self.wp_manager.advance()
            if self.wp_manager.is_complete():
                self._arc_state = 'ALIGN'
                return
            self._arc_state = 'CRUISE'
            self.ramp_linear._current = self.arc_linear
            self.ramp_angular._current = 0.0
            self.get_logger().info(
                f'ARC completado -> CRUISE al WP {self.wp_manager.progress_string()}')
            return

        w_target = math.copysign(self.arc_angular, heading_error_next)
        linear_vel = self.ramp_linear.apply(self.arc_linear)
        angular_vel = self.ramp_angular.apply(w_target)
        self._publish_velocity(linear_vel, angular_vel)
        self._publish_status(
            f'ArcTurn WP {self.wp_manager.progress_string()}, '
            f'heading_err_next={math.degrees(heading_error_next):.1f} deg')

    def _handle_arc_final(self, distance: float, heading_error: float):
        if self._final_aligning:
            self.ramp_linear.reset()
            if abs(heading_error) < self.alignment_tolerance:
                self._final_aligning = False
                self.get_logger().info('ArcFinal alineado - avanzando al ultimo WP')
            else:
                angular_vel = math.copysign(self.cruise_angular, heading_error)
                self._publish_velocity(0.0, angular_vel)
                self._publish_status(
                    f'ArcFinal aligning, '
                    f'heading_err={math.degrees(heading_error):.1f} deg')
                return

        if distance < self.goal_tolerance:
            self._handle_wp_reached()
            self._arc_state = 'ALIGN'
            return

        if abs(heading_error) > self.alignment_tolerance * 2:
            self._final_aligning = True
            self._publish_stop()
            return

        if distance < self.decel_distance:
            target_vel = self.cruise_linear * (distance / self.decel_distance)
            target_vel = max(target_vel, self.min_linear)
        else:
            target_vel = self.cruise_linear

        linear_vel = self.ramp_linear.apply(target_vel)
        target_angular = heading_error * self.angular_gain
        target_angular = max(-self.max_angular, min(self.max_angular, target_angular))
        angular_vel = self.ramp_angular.apply(target_angular)

        self._publish_velocity(linear_vel, angular_vel)
        self._publish_status(
            f'ArcFinal WP {self.wp_manager.progress_string()}, '
            f'dist={distance:.2f}m, heading_err={math.degrees(heading_error):.1f} deg')
