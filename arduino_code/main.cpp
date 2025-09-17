#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>

// ROS
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Bool.h>

// --------------------- CONFIG ---------------------
#define LED_PIN       6
#define LED_COUNT     10

#define SENSOR_RX     8
#define SENSOR_TX     9

#define PACKET_HEADER1 0x4D  // 'M'
#define PACKET_HEADER2 0x46  // 'F'
#define NUM_SENSORS     8
#define BYTES_PER_DISTANCE 2
#define PACKET_SIZE (2 + NUM_SENSORS * BYTES_PER_DISTANCE)

// --------------------------------------------------

SoftwareSerial sensorSerial(SENSOR_RX, SENSOR_TX);
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ROS
ros::NodeHandle nh;

// Publisher distancias
std_msgs::UInt16MultiArray sensor_data_msg;
ros::Publisher sensor_pub("/sensor_distances", &sensor_data_msg);

// Buffer paquete
uint8_t buffer[64];
uint8_t index = 0;
bool sync = false;

// ----------- LEDs -----------
void setAll(uint32_t color){
  for (int i=0;i<LED_COUNT;i++) pixels.setPixelColor(i, color);
  pixels.show();
}
void setAlert(bool alert){ // true=rojo, false=verde
  setAll(alert ? pixels.Color(255,0,0) : pixels.Color(0,255,0));
}

// Subscriber /alerta_led (Bool)
void alertaCb(const std_msgs::Bool &msg){
  setAlert(msg.data);
}
ros::Subscriber<std_msgs::Bool> alerta_sub("/alerta_led", &alertaCb);

// ----------- Procesado -----------
void procesarPaquete(uint8_t* data) {
  static uint16_t sensor_values[NUM_SENSORS];
  sensor_data_msg.data = sensor_values;
  sensor_data_msg.data_length = NUM_SENSORS;

  for (int i=0;i<NUM_SENSORS;i++){
    uint16_t d = ((uint16_t)data[2 + i*2] << 8) | data[3 + i*2];
    if (d == 0xFFFF) d = 0; // sin dato válido -> 0
    sensor_values[i] = d;
  }
  sensor_pub.publish(&sensor_data_msg);
}

void setup() {
  Serial.begin(115200);
  sensorSerial.begin(115200);
  pixels.begin();
  pixels.show();
  setAlert(false); // verde al iniciar

  nh.initNode();
  nh.advertise(sensor_pub);
  nh.subscribe(alerta_sub);
}

void loop() {
  while (sensorSerial.available()) {
    uint8_t byte = sensorSerial.read();

    if (!sync) {
      if (index == 0 && byte == PACKET_HEADER1)      { buffer[index++] = byte; }
      else if (index == 1 && byte == PACKET_HEADER2) { buffer[index++] = byte; sync = true; }
      else                                            { index = 0; }
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
