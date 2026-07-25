"""Contratos de topics entre capas. Comentario = tipo del mensaje."""

# IMU driver
IMU_HEADING = '/imu/heading'          # Float32, deg, yaw relativo
IMU_RATE = '/imu/rate'                # Float32, rad/s
IMU_STATUS = '/imu/status'            # String, JSON
IMU_CALIBRATE = '/imu/calibrate'      # Empty, trigger de recalibracion

# GPS driver
GPS_FIX = '/gps/fix'                  # GPSFix, pos + course + quality
GPS_HEADING = '/gps/heading'          # Float32, deg ENU; solo cuando el COG es confiable
GPS_STATUS = '/gps/status'            # String, JSON

# Fusion
HEADING_FUSED = '/heading/fused'              # Float32, deg ENU
HEADING_FUSED_READY = '/heading/fused_ready'  # Bool, latched
FUSION_STATUS = '/fusion/status'              # String, JSON
