"""Controlador de navegacion: lleva el robot por una ruta de waypoints (en coordenadas
locales x/y) usando la odometria, y genera /cmd_vel. Navega en modo stop_and_turn
(frena, gira, avanza) waypoint por waypoint.
"""


class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        # (imports y parametros: se asumen cargados)

        self._current_x = None
        self._current_y = None
        self._current_heading = None
        self._last_odom_time = None
        self._route_loaded = False
        self._loop_route = False
        self._navigating = False
        self._wp_manager = WaypointManager()
        self._control = StopAndTurnControl(StopAndTurnParams(
            cruise_linear=self.cruise_linear, cruise_angular=self.cruise_angular,
            angular_gain=self.angular_gain, max_angular=self.max_angular,
            min_linear=self.min_linear, goal_tolerance=self.goal_tolerance,
            alignment_tolerance=self.alignment_tolerance, decel_distance=self.decel_distance))

        # Entradas
        self.create_subscription(Odometry, ODOM_FILTERED, self._on_odom, 10)   # pose local (ausencia = no navegar)
        self.create_subscription(NavRoute, NAV_ROUTE, self._on_route, 10)      # ruta x/y (vacia = parar)
        self.create_subscription(Empty, NAV_START, self._on_start, 10)         # GO

        # Salidas
        self._cmd_vel_pub = self.create_publisher(Twist, CMD_VEL, 10)
        self._status_pub = self.create_publisher(String, NAV_STATUS, 10)

        # Timers
        self.create_timer(1.0 / self.frequency, self._control_loop)

    def _control_loop(self):
        if not self._odom_fresh():           # sin odom (arranque) o se corto (rtk dejo de publicar) -> freno
            self._publish_stop()
            self._publish_status('sin odometria fresca, detenido')
            return
        if not self._navigating:
            self._publish_stop()
            return

        if self._wp_manager.is_complete():
            if self._loop_route:
                self._wp_manager.restart()
                self._control.reset()
            else:
                self._navigating = False
                self._publish_stop()
                self._publish_status('COMPLETED')
                return

        pose = (self._current_x, self._current_y, self._current_heading)
        linear, angular, reached, status = self._control.compute(pose, self._wp_manager.current_goal())
        
        if reached:
            self._wp_manager.advance()
            if not self._wp_manager.is_complete():
                self._control.reset()
            self._publish_stop()
            return
        
        self._publish_velocity(linear, angular)
        self._publish_status(status)

    def _on_odom(self, msg: Odometry):
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._current_heading = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)  # yaw del quaternion
        self._last_odom_time = self.get_clock().now()

    def _odom_fresh(self):
        # odom vale solo si llego hace menos de odom_timeout; mas viejo = no confiable, no navego.
        if self._last_odom_time is None:                                  # nunca llego odom
            return False
        age = (self.get_clock().now() - self._last_odom_time).nanoseconds * 1e-9
        return age <= self.odom_timeout

    def _on_route(self, msg: NavRoute):
        if not msg.waypoints:                      # ruta vacia = descartar la ruta y quedar en idle
            self._clear_route()
            return
        self._wp_manager.set_waypoints([(p.x, p.y) for p in msg.waypoints])
        self._loop_route = msg.loop
        self._route_loaded = True
        self._navigating = False                   # ruta lista pero NO arranca: espera el GO
        self._control.reset()
        self._publish_status(f'ruta cargada: {len(msg.waypoints)} waypoints (loop={msg.loop}), esperando GO')

    def _clear_route(self):
        self._route_loaded = False
        self._navigating = False
        self._loop_route = False
        self._publish_stop()
        self._publish_status('idle, sin ruta')

    def _on_start(self, msg: Empty):
        if not self._route_loaded:                 # GO sin ruta -> ignorar
            self._publish_status('GO ignorado: no hay ruta cargada')
            return
        self._wp_manager.restart()
        self._control.reset()
        self._navigating = True
        self._publish_status('navegando')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
