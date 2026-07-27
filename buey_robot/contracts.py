"""Contratos de topics entre capas. Comentario = tipo del mensaje."""

# GPS driver
GPS_FIX = '/gps/fix'                            # GPSFix, pos lat/lon + calidad RTK
GPS_COURSE = '/gps/course'                      # Float32, deg ENU (COG); solo cuando es confiable
GPS_STATUS = '/gps/status'                      # String, JSON

# IMU driver
IMU_YAW = '/imu/yaw'                            # Float32, deg, yaw relativo del gyro
IMU_RATE = '/imu/rate'                          # Float32, rad/s
IMU_STATUS = '/imu/status'                      # String, JSON
IMU_CALIBRATE = '/imu/calibrate'                # Empty, trigger de recalibracion

# Fusion de heading
HEADING_FUSED = '/heading/fused'                # Float32, deg ENU, yaw absoluto; sale solo al converger

# Odometria
ODOM = '/odom'                                  # Odometry, pose x/y + twist en frame local
GEO_POSITION = '/geo/position'                  # String JSON {lat, lon} del robot (para el mapa)

# Rutas (String con JSON: {waypoints:[...], loop})
GEO_ROUTE = '/geo/route'                        # String JSON {waypoints:[{lat,lon}], loop}
LOCAL_ROUTE = '/local/route'                    # String JSON {waypoints:[{x,y}], loop}

# Navegacion
NAV_START = '/nav/start'                        # Empty, GO
NAV_STATUS = '/nav/status'                      # String JSON, estado operativo (state, wp, distancia, pose, v/w)
NAV_CMD_VEL = '/nav/cmd_vel'                    # Twist, velocidad objetivo de navegacion
JOY_CMD_VEL = '/joy/cmd_vel'                    # Twist, velocidad de teleoperacion
INIT_CMD_VEL = '/init/cmd_vel'                  # Twist, velocidad de la maniobra de arranque
