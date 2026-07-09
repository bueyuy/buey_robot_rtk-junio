/*
 * ESP32 - MPU6050 IMU con micro-ROS Serial
 *
 * Publica el MPU6050 (acelerometro + giroscopio) por micro-ROS:
 *   - MPU6050 -> mpu6050/imu/data (sensor_msgs/Imu, accel + gyro real)
 *
 * El giroscopio alimenta el heading fused (nodo mpu6050_gyro -> /heading/fused).
 * El LSM303 (magnetometro/brujula) fue removido: la brujula quedo inservible y el
 * heading paso a ser gyro + COG GPS, asi que ya no se publica /imu/data ni /imu/mag.
 *
 * Lectura MPU6050: libreria Jeff Rowberg (I2Cdevlib) sobre Wire1 (bus 1).
 *
 * RECONEXION AUTOMATICA (no reset manual):
 *   El firmware NO crea las entidades micro-ROS en setup(). Usa la maquina de
 *   estados estandar de micro-ROS: hace ping al agente y crea las entidades solo
 *   cuando aparece; si el agente se cae (p.ej. al relanzar outdoor_rtk), las
 *   destruye y vuelve a esperar. Asi el ESP32 puede bootear antes o despues del
 *   agente, y reconecta solo cada vez que el agente se reinicia -> nunca hay que
 *   pulsar EN/RST. (setup() solo inicia el sensor y el transporte serial.)
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
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/imu.h>

// --- Configuracion ---
// MPU6050 en Wire1 (bus 1), pines del shield.
#define MPU_SDA  33
#define MPU_SCL  25

#define PUBLISH_INTERVAL_MS  50   // 20 Hz
#define NODE_NAME            "esp32_imu_node"

#define MPU_IMU_TOPIC        "mpu6050/imu/data"
#define MPU_FRAME            "mpu6050_link"

// Conversiones MPU6050 (rango por defecto +-2g y +-250 deg/s)
#define MPU_ACCEL_SCALE  (9.80665 / 16384.0)               // m/s^2
#define MPU_GYRO_SCALE   ((1.0 / 131.0) * (PI / 180.0))    // rad/s

// Macros de error: en la creacion de entidades NO colgamos el firmware (return
// false y reintentar), a diferencia del errorLoop() one-shot anterior.
#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { return false; } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

// Ejecuta X cada MS milisegundos sin bloquear (para el ping periodico).
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = millis(); } \
  if (millis() - init > (MS)) { X; init = millis(); } \
} while (0)

// --- Objeto sensor ---
MPU6050 accelgyro(MPU6050_DEFAULT_ADDRESS, &Wire1); // usa Wire1 (bus 1)

// --- micro-ROS ---
rcl_publisher_t mpu_imu_pub;
sensor_msgs__msg__Imu mpu_imu_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Maquina de estados de conexion con el agente
enum AgentState {
  WAITING_AGENT,      // sin agente: ping periodico hasta que responda
  AGENT_AVAILABLE,    // agente detectado: crear entidades
  AGENT_CONNECTED,    // entidades vivas: publicar + vigilar que el agente siga
  AGENT_DISCONNECTED  // agente perdido: destruir entidades y volver a esperar
} agentState = WAITING_AGENT;

bool mpuAvailable = false;

static char mpu_imu_frame[] = MPU_FRAME;

int16_t ax, ay, az, gx, gy, gz;   // buffers MPU6050

// Timer callback: lee el sensor y publica
void timerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;
  if (timer == NULL) return;
  if (!mpuAvailable) return;

  // Timestamp
  int64_t time_ns = rmw_uros_epoch_nanos();
  int32_t  t_sec  = (int32_t)(time_ns / 1000000000LL);
  uint32_t t_nsec = (uint32_t)(time_ns % 1000000000LL);

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  mpu_imu_msg.linear_acceleration.x = ax * MPU_ACCEL_SCALE;
  mpu_imu_msg.linear_acceleration.y = ay * MPU_ACCEL_SCALE;
  mpu_imu_msg.linear_acceleration.z = az * MPU_ACCEL_SCALE;
  mpu_imu_msg.angular_velocity.x = gx * MPU_GYRO_SCALE;
  mpu_imu_msg.angular_velocity.y = gy * MPU_GYRO_SCALE;
  mpu_imu_msg.angular_velocity.z = gz * MPU_GYRO_SCALE;

  mpu_imu_msg.header.stamp.sec = t_sec;  mpu_imu_msg.header.stamp.nanosec = t_nsec;

  RCSOFTCHECK(rcl_publish(&mpu_imu_pub, &mpu_imu_msg, NULL));
}

// Configura los campos fijos del mensaje (frame_id, covarianzas). Independiente
// del agente: se hace una sola vez en setup(), no en cada reconexion.
void initMessages()
{
  memset(&mpu_imu_msg, 0, sizeof(mpu_imu_msg));

  mpu_imu_msg.header.frame_id.data = mpu_imu_frame;
  mpu_imu_msg.header.frame_id.size = strlen(mpu_imu_frame);
  mpu_imu_msg.header.frame_id.capacity = sizeof(mpu_imu_frame);

  // Covarianzas MPU6050 (sin orientacion; accel + gyro disponibles)
  mpu_imu_msg.orientation_covariance[0] = -1.0;
  mpu_imu_msg.angular_velocity_covariance[0] = 0.001;
  mpu_imu_msg.angular_velocity_covariance[4] = 0.001;
  mpu_imu_msg.angular_velocity_covariance[8] = 0.001;
  mpu_imu_msg.linear_acceleration_covariance[0] = 0.01;
  mpu_imu_msg.linear_acceleration_covariance[4] = 0.01;
  mpu_imu_msg.linear_acceleration_covariance[8] = 0.01;
}

// Crea node + publisher + timer + executor. Devuelve false si algo falla (para
// reintentar). Se llama cuando el agente esta disponible.
bool createEntities()
{
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  RCCHECK(rclc_publisher_init_default(
    &mpu_imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), MPU_IMU_TOPIC));

  RCCHECK(rclc_timer_init_default(
    &timer, &support, RCL_MS_TO_NS(PUBLISH_INTERVAL_MS), timerCallback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // Sincronizar reloj con el agente (para los timestamps de los mensajes).
  rmw_uros_sync_session(1000);
  return true;
}

// Libera todas las entidades micro-ROS. Timeout de sesion 0 para no bloquear
// cuando el agente ya se fue (destruccion "en frio").
void destroyEntities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&mpu_imu_pub, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup()
{
  Serial.begin(115200);

  // MPU6050 en su propio bus I2C (Wire1)
  Wire1.begin(MPU_SDA, MPU_SCL);
  Wire1.setClock(100000);

  accelgyro.initialize();
  if (accelgyro.testConnection()) {
    mpuAvailable = true;
    Serial.println("MPU6050 inicializado OK");
  } else {
    Serial.println("MPU6050 no detectado");
  }

  // Transporte serial micro-ROS. A partir de aqui NO usar Serial.print (corromperia
  // el protocolo). Las entidades se crean en loop() cuando el agente aparezca.
  set_microros_serial_transports(Serial);

  initMessages();
  agentState = WAITING_AGENT;
}

void loop()
{
  // Reintento de reconexion del sensor (I2C, independiente del agente)
  if (!mpuAvailable) {
    accelgyro.initialize();
    if (accelgyro.testConnection()) { mpuAvailable = true; }
  }

  // Maquina de estados de conexion con el agente micro-ROS
  switch (agentState) {
    case WAITING_AGENT:
      // Ping cada 500ms; cuando el agente responde, pasar a crear entidades.
      EXECUTE_EVERY_N_MS(500,
        agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;

    case AGENT_AVAILABLE:
      // Crear entidades; si falla, destruir lo que haya y volver a esperar.
      agentState = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (agentState == WAITING_AGENT) { destroyEntities(); }
      break;

    case AGENT_CONNECTED:
      // Vigilar que el agente siga vivo (ping cada 200ms) y publicar.
      EXECUTE_EVERY_N_MS(200,
        agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (agentState == AGENT_CONNECTED) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
      }
      break;

    case AGENT_DISCONNECTED:
      // Agente perdido (p.ej. relanzaron outdoor_rtk): liberar y reesperar.
      destroyEntities();
      agentState = WAITING_AGENT;
      break;
  }
}
