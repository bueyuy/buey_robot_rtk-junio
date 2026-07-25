/*
 * ESP32 - MPU6050 (GY-521) IMU con micro-ROS Serial
 * Publica sensor_msgs/Imu (acelerometro + giroscopio) por USB serial.
 *
 * Es la UNICA IMU del robot: el MPU6050 no tiene magnetometro, asi que se
 * publica un Imu con linear_acceleration y angular_velocity (sin campo mag).
 * El heading se resuelve aguas abajo en ROS (gyro integrado + COG GPS).
 *
 * Lectura del sensor basada en la libreria MPU6050 de Jeff Rowberg (I2Cdevlib).
 */

#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

// --- Configuracion ---
// Pines I2C del shield (igual que en la prueba de banco): SDA=GPIO33, SCL=GPIO25
#define I2C_SDA  33
#define I2C_SCL  25

#define PUBLISH_INTERVAL_MS  50   // 20 Hz
#define NODE_NAME            "esp32_mpu6050_node"
#define IMU_TOPIC            "imu/data"
#define FRAME_ID             "imu_link"

// Conversiones MPU6050 (rango por defecto: +-2g y +-250 deg/s)
// Acelerometro: 16384 LSB/g  ->  m/s^2 = raw / 16384 * 9.80665
#define ACCEL_SCALE  (9.80665 / 16384.0)
// Giroscopio: 131 LSB/(deg/s) ->  rad/s = raw / 131 * (PI/180)
#define GYRO_SCALE   ((1.0 / 131.0) * (PI / 180.0))

// Macros para verificar errores
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { errorLoop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

// --- Objetos ---
MPU6050 accelgyro;

rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

bool sensorAvailable = false;
static char imu_frame[] = FRAME_ID;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// Error loop
void errorLoop()
{
  while (true) {
    delay(100);
  }
}

// Timer callback: lee sensor y publica
void timerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer == NULL) return;

  if (sensorAvailable) {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Acelerometro en m/s^2
    imu_msg.linear_acceleration.x = ax * ACCEL_SCALE;
    imu_msg.linear_acceleration.y = ay * ACCEL_SCALE;
    imu_msg.linear_acceleration.z = az * ACCEL_SCALE;

    // Giroscopio (velocidad angular) en rad/s
    imu_msg.angular_velocity.x = gx * GYRO_SCALE;
    imu_msg.angular_velocity.y = gy * GYRO_SCALE;
    imu_msg.angular_velocity.z = gz * GYRO_SCALE;
  }

  // Timestamp
  int64_t time_ns = rmw_uros_epoch_nanos();
  imu_msg.header.stamp.sec = (int32_t)(time_ns / 1000000000LL);
  imu_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);

  // Publicar
  RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
}

void setup()
{
  Serial.begin(115200);

  // Inicializar I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // Inicializar sensor MPU6050
  accelgyro.initialize();
  if (accelgyro.testConnection()) {
    sensorAvailable = true;
    Serial.println("MPU6050 inicializado OK");
  } else {
    Serial.println("MPU6050 no detectado");
  }

  // Configurar transporte serial micro-ROS
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // Esperar agente micro-ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Crear nodo
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // Publisher IMU
  RCCHECK(rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    IMU_TOPIC));

  // Timer 20 Hz
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(PUBLISH_INTERVAL_MS),
    timerCallback));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Inicializar mensaje
  memset(&imu_msg, 0, sizeof(imu_msg));

  // Frame ID
  imu_msg.header.frame_id.data = imu_frame;
  imu_msg.header.frame_id.size = strlen(imu_frame);
  imu_msg.header.frame_id.capacity = sizeof(imu_frame);

  // Covarianza: sin orientacion disponible (REP-145)
  imu_msg.orientation_covariance[0] = -1.0;

  // Covarianza velocidad angular (diagonal)
  imu_msg.angular_velocity_covariance[0] = 0.001;
  imu_msg.angular_velocity_covariance[4] = 0.001;
  imu_msg.angular_velocity_covariance[8] = 0.001;

  // Covarianza acelerometro (diagonal)
  imu_msg.linear_acceleration_covariance[0] = 0.01;
  imu_msg.linear_acceleration_covariance[4] = 0.01;
  imu_msg.linear_acceleration_covariance[8] = 0.01;

  // Sincronizar tiempo
  rmw_uros_sync_session(1000);
}

void loop()
{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  // Intentar reconectar sensor
  if (!sensorAvailable) {
    accelgyro.initialize();
    if (accelgyro.testConnection()) {
      sensorAvailable = true;
    }
  }
}
