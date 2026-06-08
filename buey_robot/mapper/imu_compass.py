"""imu_compass: calcula el heading de brujula desde el magnetometro CRUDO.

Reemplaza el heading que publica el firmware ESP32 (que sale con un offset
hard-iron sin corregir, por eso al girar 90 la brujula casi no cambia: el
origen queda fuera de la nube de mag y atan2 barre solo ~20% de los 360).

Este nodo se suscribe a /imu/mag (sensor_msgs/MagneticField, campo crudo),
filtra glitches del I2C, aplica calibracion hard/soft-iron y publica un
heading de brujula LINEAL (barre los 360 completos):

    m_cal = soft_iron @ (m_raw - hard_iron)      # centrar + de-sesgar elipse
    heading = atan2(m_cal_y, m_cal_x)            # grados 0-360

La calibracion (hard_iron, soft_iron) vive en config/imu.yaml -> versionable
y NO se pierde en reboots (a diferencia de la calibracion en RAM del firmware).
Se obtiene con la herramienta imu_cal/fit_ellipse.py sobre una captura de
varias vueltas.

El heading sale en la MISMA convencion que usaba el firmware (0 = referencia,
sentido segun 'invert'), asi rtk.py lo consume sin cambios: convierte a ENU
con yaw_enu = 90 - heading - declinacion, y la declinacion absorbe el offset
constante. El sentido de giro (horario/antihorario) se ajusta con 'invert'.

NOTA: sin tilt-compensation (el accel del firmware esta mal escalado). Asume
robot nivelado, igual que el firmware.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import MagneticField
from std_msgs.msg import Float32


class ImuCompass(Node):
    def __init__(self):
        super().__init__('imu_compass')

        # --- Parametros (sin defaults: fail-fast si falta en el YAML) ---
        self.declare_parameter('mag_topic', Parameter.Type.STRING)
        self.declare_parameter('heading_topic', Parameter.Type.STRING)
        self.declare_parameter('glitch_threshold', Parameter.Type.DOUBLE)
        self.declare_parameter('filter_alpha', Parameter.Type.DOUBLE)
        self.declare_parameter('invert', Parameter.Type.BOOL)
        # hard_iron: [offset_x, offset_y]
        self.declare_parameter('hard_iron', Parameter.Type.DOUBLE_ARRAY)
        # soft_iron: matriz 2x2 fila-mayor [a, b, c, d] -> [[a,b],[c,d]]
        self.declare_parameter('soft_iron', Parameter.Type.DOUBLE_ARRAY)

        self.mag_topic = self.get_parameter('mag_topic').value
        self.heading_topic = self.get_parameter('heading_topic').value
        self.glitch = self.get_parameter('glitch_threshold').value
        self.alpha = self.get_parameter('filter_alpha').value
        self.invert = self.get_parameter('invert').value
        hi = self.get_parameter('hard_iron').value
        si = self.get_parameter('soft_iron').value
        if len(hi) != 2 or len(si) != 4:
            raise ValueError(
                f'hard_iron debe tener 2 elementos (tiene {len(hi)}) y '
                f'soft_iron 4 (tiene {len(si)})')
        self.ox, self.oy = hi[0], hi[1]
        self.w = si  # [a, b, c, d]

        # Estado del filtro de heading (media circular suavizada)
        self._sin = None
        self._cos = None
        self._last_good = None
        self._glitch_count = 0
        self._msg_count = 0

        self.pub = self.create_publisher(Float32, self.heading_topic, 10)
        self.create_subscription(
            MagneticField, self.mag_topic, self._mag_callback, 10)

        self.get_logger().info(
            f'imu_compass: {self.mag_topic} -> {self.heading_topic}')
        self.get_logger().info(
            f'  hard_iron=({self.ox:.1f}, {self.oy:.1f}) '
            f'soft_iron={self.w} invert={self.invert} alpha={self.alpha}')

    def _mag_callback(self, msg: MagneticField):
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z

        # Filtrar glitches del I2C (valores de saturacion/error tipo +-4096)
        if (abs(mx) > self.glitch or abs(my) > self.glitch
                or abs(mz) > self.glitch):
            self._glitch_count += 1
            return

        # Calibracion: centrar (hard-iron) + de-sesgar elipse (soft-iron 2x2)
        dx = mx - self.ox
        dy = my - self.oy
        cal_x = self.w[0] * dx + self.w[1] * dy
        cal_y = self.w[2] * dx + self.w[3] * dy

        heading = math.atan2(cal_y, cal_x)  # rad
        if self.invert:
            heading = -heading

        # Suavizado por media circular (evita el salto 359->0)
        s, c = math.sin(heading), math.cos(heading)
        if self._sin is None:
            self._sin, self._cos = s, c
        else:
            self._sin = self.alpha * s + (1 - self.alpha) * self._sin
            self._cos = self.alpha * c + (1 - self.alpha) * self._cos
        deg = math.degrees(math.atan2(self._sin, self._cos)) % 360.0

        out = Float32()
        out.data = float(deg)
        self.pub.publish(out)
        self._last_good = deg

        self._msg_count += 1
        if self._msg_count == 1 or self._msg_count % 100 == 0:
            self.get_logger().info(
                f'heading={deg:.1f} (mag_cal=({cal_x:.0f},{cal_y:.0f}) '
                f'glitches={self._glitch_count})')


def main(args=None):
    rclpy.init(args=args)
    node = ImuCompass()
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
