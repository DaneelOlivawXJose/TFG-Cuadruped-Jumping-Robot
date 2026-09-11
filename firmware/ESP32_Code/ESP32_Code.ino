#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <SCServo.h>

// --- CONFIGURACIÓN DE SERVOS ---
SCSCL sc; 
#define RX_PIN 16
#define TX_PIN 17

#define LED_PIN 2

const int n_servos = 10; 
u8 ids[n_servos] = {1, 8, 3, 2, 7, 6, 4, 5, 9, 10}; 

// --- OBJETOS DE MICRO-ROS ---
rcl_subscription_t subscriber;
rcl_subscription_t subscriber_salto;
rcl_publisher_t publisher_listo; 

std_msgs__msg__Int32MultiArray msg;
std_msgs__msg__Int32 msg_salto; 
std_msgs__msg__Int32 msg_listo; 

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// --- VARIABLES DE TEMPORIZACIÓN PARA EL SERVO 9 ---
bool ejecutando_salto = false;
unsigned long tiempo_inicio_salto = 0;
unsigned long duracion_salto_ms = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop()
{
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500); 
  }
}

// CALLBACK DE POSICIONES GENERALES
void subscription_callback(const void * msgin)
{
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 

  const std_msgs__msg__Int32MultiArray * array_msg = (const std_msgs__msg__Int32MultiArray *)msgin;

  if (array_msg->data.size >= n_servos)
  {
    for(int i = 0; i < n_servos; i++)
    {
      int pos_objetivo = array_msg->data.data[i];

      if (ids[i] == 9)
      {
        // Ignorar órdenes del array general si estamos ejecutando el temporizador de salto
        if (!ejecutando_salto) {
          if (pos_objetivo < 0) pos_objetivo = 0;
          if (pos_objetivo > 2047) pos_objetivo = 2047;
          sc.WritePWM(9, pos_objetivo);
        }
      } 
      else 
      {
        // El servo 10 entrará automáticamente por aquí, en modo posición normal
        if (pos_objetivo < 0) pos_objetivo = 0;
        sc.WritePos(ids[i], (u16)pos_objetivo, 0, 0);
      }
    }
  }
}

// CALLBACK DE LA PETICION DE SALTO POR TIEMPO (ID 9)
void subscription_salto_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  long tiempo_pedido = msg->data; // Milisegundos que debe estar girando
  
  // Solo aceptamos la orden si NO estamos ya haciendo un salto
  if (!ejecutando_salto && tiempo_pedido != 0) {
    
    ejecutando_salto = true;
    duracion_salto_ms = abs(tiempo_pedido);
    tiempo_inicio_salto = millis(); // Disparamos el cronómetro interno del ESP32
    
    if (tiempo_pedido > 0) {
      sc.WritePWM(9, 600);  // Hacia delante
    } else {
      sc.WritePWM(9, 2000); // Hacia atrás
    }
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  set_microros_transports();
  delay(100); 

  Serial2.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN); 
  sc.pSerial = &Serial2; 
  delay(500); 

  // Freno de emergencia en el arranque
  sc.WritePWM(9, 0);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "nodo_esp32_esclavo", "", &support));

  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "servo_poses"));
  RCCHECK(rclc_subscription_init_default(&subscriber_salto, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "cmd_meta_salto"));
  RCCHECK(rclc_publisher_init_default(&publisher_listo, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "estado_salto_listo"));

  // La capacidad ahora será automáticamente de 10
  msg.data.capacity = n_servos;
  msg.data.data = (int32_t*) malloc(msg.data.capacity * sizeof(int32_t));
  msg.data.size = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_salto, &msg_salto, &subscription_salto_callback, ON_NEW_DATA));
}

void loop() {
  
  // --- MÁQUINA DE ESTADO DEL TEMPORIZADOR ---
  if (ejecutando_salto) {
    // Comprobamos si el cronómetro ya ha alcanzado el tiempo objetivo
    if (millis() - tiempo_inicio_salto >= duracion_salto_ms) {
      
      sc.WritePWM(9, 0); // FRENO DE EMERGENCI
      ejecutando_salto = false;

      // Avisamos a ROS 2 de que el tiempo se ha cumplido
      msg_listo.data = 1;
      RCSOFTCHECK(rcl_publish(&publisher_listo, &msg_listo, NULL));
    }
  }

  // --- PROCESAR MICRO-ROS ---
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}