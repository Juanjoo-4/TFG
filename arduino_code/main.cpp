// Resumen: Se publican las distancias de los arrays de sensores continuamente. Según la distancia detectada se está en un estado
// de alerta u otro y se encenderan los leds del color correspondiente al estado, en el código se explican los diferentes estados y sus colores y cómo se organizan.
// Además, se ha implementado un modo manual para darle la utilidad que se desee, en este caso los leds dejan de funcionar de forma automtica
// y enciende los leds verdes o rojos según el estado en el que se encuentre, igual se explica más claramente en el código.
//
// Topics:
//   Pub: /sensor_distances_1  (UInt16MultiArray) : pub de las distancias del array de sensores 1
//        /sensor_distances_2  (UInt16MultiArray) : pub de las distancias del array de sensores 2
//        /alerta_estado       (Int8)             : pub del estado de alerta: 0 -> OK (verde), 1 -> BAJO (azul),  2 -> ALTO (rojo)
//        /alerta_led          (Bool              : true = "ALTO" o "BAJO" y false = "OK"
//   Sub: /modo_manual         (Bool)             : true = obedece a /alerta_forzada, false = modo automático
//        /alerta_forzada      (Bool)             : utilidad que se desee en el modo manual

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

// Configuración LEDs
#define PIN_LED 6
#define NUM_LEDS 1 // Solo uso uno como prueba porque lo estaba alimentando con el arduino
Adafruit_NeoPixel pixels(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

// Puertos Sensores
#define PUERTO_SENSOR_A Serial1 // D18 TX1, D19 RX1
#define PUERTO_SENSOR_B Serial2 // D16 TX2, D17 RX2
static const unsigned long VELOCIDAD_SERIE_SENSORES = 115200;

// Protocolo Sensores: es el encabezado de los paquetes de los sensores
#define CABECERA1 0x4D
#define CABECERA2 0x46
#define NUM_SENSORES 8 // Sensores por array

// Umbrales
static uint16_t UMBRAL_BAJO = 150;
static uint16_t UMBRAL_ALTO = 500;
static const uint32_t TIEMPO_MAXIMO_SIN_DATOS_MS = 250; // si uno de los arrays de sensores no envía datos en este tiempo,
                                                        // se considera que faltan datos y, por tanto, cambia el estado de /alerta_estado
                                                        // como precaución

// ROS
ros::NodeHandle nodo_ros;

std_msgs::UInt16MultiArray mensaje_distancias_1, mensaje_distancias_2;
ros::Publisher publicador_dist_1("/sensor_distances_1", &mensaje_distancias_1); // pub de distancias del array de sensores 1
ros::Publisher publicador_dist_2("/sensor_distances_2", &mensaje_distancias_2); // pub de distancias del array de sensores 2

std_msgs::Int8 mensaje_alerta_estado;
ros::Publisher publicador_estado("/alerta_estado", &mensaje_alerta_estado); // pub del estado de alerta, que puede ser 0/1/2

std_msgs::Bool mensaje_alerta_led;
ros::Publisher publicador_led("/alerta_led", &mensaje_alerta_led); // pub del estado de alerta en modo manual

// Variables de estado global
static uint16_t distancias_array1[NUM_SENSORES], distancias_array2[NUM_SENSORES]; // últimas lecturas de cada array de sensores
static bool hay_datos_array1 = false, hay_datos_array2 = false;                   // bool que indica que ya tiene datos válidos
static uint32_t tiempo_ultimo_array1 = 0, tiempo_ultimo_array2 = 0;               // tiempo en millis del último paquete recibido

// Modo manual
static volatile bool modo_manual = false;    // modo manual en el que deja de funcionar de forma automática para obedecer a /alerta_forzada
static volatile bool alerta_forzada = false; // valor manual de lo que se desee, en este caso solo cambia de estado
                                             // de forma que "true = leds rojos" y "false = leds verdes"

// Últimos valores publicados, se usan para que se muestren los valores en los topics solo cuando cambien de estado
static int8_t ultimo_estado_publicado = -1;
static bool ultimo_led_publicado = false;
static bool hay_publicacion_led = false;
static bool hay_publicacion_estado = false;

// Función básica de LEDs Neopixel
static inline void pintarColor(uint8_t r, uint8_t g, uint8_t b)
{
  for (int i = 0; i < NUM_LEDS; i++)
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  pixels.show();
}
static inline void pintarVerde() { pintarColor(0, 255, 0); }
static inline void pintarAzul() { pintarColor(0, 0, 255); }
static inline void pintarRojo() { pintarColor(255, 0, 0); }

// Callbacks ROS

// Al llegar /modo_manual:
// - Si cambia el modo, se actualiza modo_manual.
// - Si entra en manual, pintas ya en rojo/verde según alerta_forzada.
// - Si se sale a automático, lo pintará el ciclo periódico.
void callbackModoManual(const std_msgs::Bool &msg)
{
  bool nuevo_modo = msg.data;
  if (nuevo_modo != modo_manual)
  {
    modo_manual = nuevo_modo;
    if (modo_manual)
    {
      if (alerta_forzada)
        pintarRojo();
      else
        pintarVerde();
    }
  }
}
ros::Subscriber<std_msgs::Bool> subscriptor_modo("/modo_manual", &callbackModoManual);

// Al llegar /alerta_forzada, actualiza los leds.
void callbackAlertaForzada(const std_msgs::Bool &msg)
{
  alerta_forzada = msg.data;
  if (modo_manual)
  {
    if (alerta_forzada)
      pintarRojo();
    else
      pintarVerde();
  }
}
ros::Subscriber<std_msgs::Bool> subscriptor_alerta("/alerta_forzada", &callbackAlertaForzada);

// Estado de lectura de paquetes
struct EstadoLectura
{
  uint8_t etapa = 0;                        // se usa para ver en qué fase del switch está: 
                                            // 0 = busca "cabecera1", 1 = busca "cabecera2", 2 = lee los datos
  uint8_t indice_payload = 0;               // posición dentro del payload, para ver cuantos datos lleva
  uint8_t buffer_payload[NUM_SENSORES * 2]; // bytes de payload
};

// Un estado de lectura para cada array
EstadoLectura estado_array1;
EstadoLectura estado_array2;

// Función de lectura 
// Lee del puerto serie (Serial1/Serial2 sirven porque heredan de Stream), usando su estado, y rellena el array distancias 
// cuando logra un paquete completo. Devuelve true si lo completó.
bool leerPaquete(Stream &puerto, EstadoLectura &estado, uint16_t distancias[NUM_SENSORES])
{
  while (puerto.available())
  {
    int byte_entrada = puerto.read();
    if (byte_entrada < 0)
      break;

    switch (estado.etapa)
    {
    case 0: // Etapa 0: se busca cabecera1 (0x4D, ‘M’). Si se ve → se pasa a 1.
      if (byte_entrada == CABECERA1)
        estado.etapa = 1;
      break;

    case 1: // Etapa 1: se espera CABECERA2 (0x46, ‘F’).
            // - Si llega, se pasa a 2 y se pone indice_payload=0.
            // - Si no, pero llegó otra vez ‘M’, mantiene en 1 por si ahora sí llega ‘F’ pegado. Si no, vuelve a 0.
      if (byte_entrada == CABECERA2)
      {
        estado.etapa = 2;
        estado.indice_payload = 0;
      }
      else
      {
        estado.etapa = (byte_entrada == CABECERA1) ? 1 : 0;
      }
      break;

    case 2: // Leyendo payload hasta los 16 bytes. Al completarlo:
            // - Se reconstruye cada lectura big-endian.
            // - Si el valor es 65535 (0xFFFF), se convierte a 0 para más claridad.
            // - Se guarda en distancias[], se resetea a etapa 0 y se devuelve true
      estado.buffer_payload[estado.indice_payload++] = (uint8_t)byte_entrada;
      if (estado.indice_payload >= NUM_SENSORES * 2)
      {
        // Decodificar distancias
        for (int i = 0; i < NUM_SENSORES; i++)
        {
          uint16_t valor = (uint16_t(estado.buffer_payload[2 * i]) << 8) | estado.buffer_payload[2 * i + 1];
          distancias[i] = (valor == 65535) ? 0 : valor; // 65535 => 0
        }
        estado.etapa = 0;
        return true;
      }
      break;
    }
  }
  return false;
}

// Evaluación de estado 
// Para una distancia:
// 0 → se interpreta como vacío, es decir, ALTO.
// distancia > UMBRAL_ALTO → ALTO.
// distancia < UMBRAL_BAJO → BAJO.
// Resto → OK
static inline int8_t estadoDeValor(uint16_t valor)
{
  if (valor == 0)
    return 2; // ALTO
  if (valor > UMBRAL_ALTO)
    return 2; // ALTO
  if (valor < UMBRAL_BAJO)
    return 1; // BAJO
  return 0;   // OK
}

// Se combinan los arrays para comprobar estado de ambos, por ejemplo, para los casos en que uno detecte una distancia > UMBRAL_ALTO 
// y el otro distancia < UMBRAL_BAJO a la vez, se tenga en cuenta la prioridad de forma que ALTO > BAJO > OK.
static inline int8_t combinarPrioridad(int8_t a, int8_t b)
{
  if (a == 2 || b == 2)
    return 2;
  if (a == 1 || b == 1)
    return 1;
  return 0;
}

// Se recorre las 8 distancias:
// - Si hay ALTO, se devuelve ALTO al instante.
// - Si hubo algún BAJO, al final se devuelve BAJO.
// - Si todas OK, se devuelve OK.
static int8_t estadoArray(const uint16_t distancias[NUM_SENSORES])
{
  int8_t resultado = 0;
  for (int i = 0; i < NUM_SENSORES; i++)
  {
    int8_t estado = estadoDeValor(distancias[i]);
    if (estado == 2)
      return 2; // atajo
    if (estado == 1)
      resultado = 1;
  }
  return resultado;
}

// Función que se usa para marcar el estado como BAJO cuando un array no ha mandado nada en 250 ms, para los casos donde dejen de funcionar por cualquier motivo.
// Se combina A y B con prioridad.
static int8_t calcularEstadoGlobal()
{
  const uint32_t ahora = millis();
  const bool faltan_datos1 = (!hay_datos_array1) || (ahora - tiempo_ultimo_array1 > TIEMPO_MAXIMO_SIN_DATOS_MS);
  const bool faltan_datos2 = (!hay_datos_array2) || (ahora - tiempo_ultimo_array2 > TIEMPO_MAXIMO_SIN_DATOS_MS);

  int8_t estado1 = faltan_datos1 ? 1 : estadoArray(distancias_array1);
  int8_t estado2 = faltan_datos2 ? 1 : estadoArray(distancias_array2);

  return combinarPrioridad(estado1, estado2);
}


// Publicación y pintado
static void publicarYPintar(int8_t estado)
{
  // Se publica en /alerta_estado solo cuando cambia de estado
  if (!hay_publicacion_estado || estado != ultimo_estado_publicado)
  {
    mensaje_alerta_estado.data = estado;
    publicador_estado.publish(&mensaje_alerta_estado);
    ultimo_estado_publicado = estado;
    hay_publicacion_estado = true;
  }

  // Realmente este es redundante y se podría eliminar, solo muestra si hay alerta o no, 
  // pero lo usé durante el desarrollo del código para hacer /alerta_estado y ver si estaba funcionando bien 
  // Se publica en /alerta_led según el estado:
  // - En manual funciona igual a /alerta_forzada
  // - En automático funciona de forma que true = "ALTO" o "BAJO" y false = "OK"
  // También se publica solo si cambia de estado
  bool valor_led = modo_manual ? alerta_forzada : (estado != 0);
  if (!hay_publicacion_led || valor_led != ultimo_led_publicado)
  {
    mensaje_alerta_led.data = valor_led;
    publicador_led.publish(&mensaje_alerta_led);
    ultimo_led_publicado = valor_led;
    hay_publicacion_led = true;
  }

  // Pintar LEDs de forma que:
  // En manual: rojo/verde según alerta_forzada
  // En automático: verde = OK, azul = BAJO, rojo = ALTO
  if (modo_manual)
  {
    if (alerta_forzada)
      pintarRojo();
    else
      pintarVerde();
  }
  else
  {
    switch (estado)
    {
    case 0:
      pintarVerde();
      break;
    case 1:
      pintarAzul();
      break;
    case 2:
      pintarRojo();
      break;
    default:
      pintarVerde();
      break;
    }
  }
}


void setup()
{
  pixels.begin();
  pixels.show();

  Serial.begin(115200); // Para rosserial
  PUERTO_SENSOR_A.begin(VELOCIDAD_SERIE_SENSORES);
  PUERTO_SENSOR_B.begin(VELOCIDAD_SERIE_SENSORES);

  nodo_ros.initNode();

  mensaje_distancias_1.data_length = NUM_SENSORES; // Tamaño del array
  mensaje_distancias_2.data_length = NUM_SENSORES;
  mensaje_distancias_1.data = distancias_array1; // Puntero al array de enteros de 16 bits
  mensaje_distancias_2.data = distancias_array2;

  nodo_ros.advertise(publicador_dist_1);
  nodo_ros.advertise(publicador_dist_2);
  nodo_ros.advertise(publicador_estado);
  nodo_ros.advertise(publicador_led);

  nodo_ros.subscribe(subscriptor_modo);
  nodo_ros.subscribe(subscriptor_alerta);

  // Estado inicial
  modo_manual = false;
  alerta_forzada = false;
  ultimo_estado_publicado = -1;
  hay_publicacion_estado = false;
  hay_publicacion_led = false;

  // Flash de arranque
  pintarAzul();
  delay(200);
  pintarVerde();
}

void loop()
{
  // Se leen los paquetes
  // Se intenta leer un paquete del array A, si se consigue, marca que hay datos, se actualiza el instante y publica /sensor_distances_1
  if (leerPaquete(PUERTO_SENSOR_A, estado_array1, distancias_array1))
  {
    hay_datos_array1 = true;
    tiempo_ultimo_array1 = millis();
    publicador_dist_1.publish(&mensaje_distancias_1);
  }

  // Se intenta leer un paquete del array B, si se consigue, marca que hay datos, se actualiza el instante y publica /sensor_distances_2
  if (leerPaquete(PUERTO_SENSOR_B, estado_array2, distancias_array2))
  {
    hay_datos_array2 = true;
    tiempo_ultimo_array2 = millis();
    publicador_dist_2.publish(&mensaje_distancias_2);
  }

  // Evaluación periódica (~20 Hz)
  // Se comprueba con la función calcularEstadoGlobal() si siguen funcionando correctamente los sensores, calcula el estado de cada array y los combina en un solo estado
  // Luego publica y pinta los leds
  static uint32_t tiempo_ultima_eval = 0;
  const uint32_t ahora = millis();
  if (ahora - tiempo_ultima_eval >= 50)
  {
    tiempo_ultima_eval = ahora;
    int8_t estado = calcularEstadoGlobal();
    publicarYPintar(estado);
  }

  nodo_ros.spinOnce();
}
