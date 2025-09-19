#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

// ROS
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Bool.h>

// ---------------- CONFIGURACIÓN ----------------
#define LED_PIN     6
#define LED_COUNT   1

#define SENSOR_RX   8   // RX del Arduino <- TX del sensor
#define SENSOR_TX   9   // TX del Arduino -> RX del sensor

#define NUM_SENSORS 8
#define BYTES_PER_DISTANCE 2
#define PACKET_HEADER1 0x4D
#define PACKET_HEADER2 0x46
#define PACKET_SIZE (2 + NUM_SENSORS * BYTES_PER_DISTANCE)

// ---------------- HARDWARE ----------------
SoftwareSerial sensorSerial(SENSOR_RX, SENSOR_TX);
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------------- ROS ----------------
ros::NodeHandle nh;

// Mensaje de distancias
std_msgs::UInt16MultiArray sensor_data_msg;
ros::Publisher sensor_pub("/sensor_distances", &sensor_data_msg);

// Callback del LED (alerta desde ROS C++)
void alertaCallback(const std_msgs::Bool& msg) {
  bool alerta = msg.data;
  if (alerta) {
    // ROJO
    for (int i = 0; i < LED_COUNT; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    }
  } else {
    // VERDE
    for (int i = 0; i < LED_COUNT; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 255, 0));
    }
  }
  pixels.show();
}
ros::Subscriber<std_msgs::Bool> alerta_sub("/alerta_led", &alertaCallback);


// ---------------- VARIABLES ----------------
uint8_t buffer[PACKET_SIZE];
uint16_t distances[NUM_SENSORS];
bool sync = false;
uint8_t index = 0;

// ---------------- FUNCIONES ----------------
void procesarPaquete(uint8_t *buf) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int hi = buf[2 + i * 2];
    int lo = buf[2 + i * 2 + 1];
    uint16_t val = (hi << 8) | lo;

    // El sensor devuelve 65535 cuando no detecta nada → lo convertimos a 0
    if (val == 65535) {
      val = 0;
    }

    distances[i] = val;
  }

  // Publicar en ROS
  sensor_data_msg.data_length = NUM_SENSORS;
  sensor_data_msg.data = distances;
  sensor_pub.publish(&sensor_data_msg);
}


void setup() {
  // LEDs
  pixels.begin();
  pixels.show();

  // Serial del sensor
  Serial.begin(115200);
  sensorSerial.begin(115200);

  // ROS
  nh.initNode();
  nh.advertise(sensor_pub);
  nh.subscribe(alerta_sub);
}

void loop() {
  // Leer datos del sensor
  while (sensorSerial.available()) {
    uint8_t byte = sensorSerial.read();

    if (!sync) {
      if (index == 0 && byte == PACKET_HEADER1) {
        buffer[index++] = byte;
      } else if (index == 1 && byte == PACKET_HEADER2) {
        buffer[index++] = byte;
        sync = true;
      } else {
        index = 0;
      }
    } else {
      buffer[index++] = byte;

      if (index >= PACKET_SIZE) {
        procesarPaquete(buffer);
        index = 0;
        sync = false;
      }
    }
  }

  nh.spinOnce();
}
