# TFG: TeraRanger Multiflex + Arduino + ROS (Nodo C++)

## Objetivo
Arduino publica distancias; un nodo C++ evalúa si todas están dentro del rango [LOW, HIGH]. 

Si alguna está fuera o 0 (vacío) → publica alerta → Arduino pone LEDs en rojo; si no, verde.

## Requisitos
- Ubuntu 20.04 + ROS Noetic
- rosserial_python, std_msgs
- PlatformIO (para Arduino)

## Conexiones de hardware

### Arduino Nano (ATmega328P - bootloader nuevo)

- Sensor TeraRanger Multiflex:
  - RX (sensor) → pin 8 (Arduino)
  - TX (sensor) → pin 9 (Arduino) (no es necesario)
  - Alimentación: 5V y GND desde el Arduino o fuente externa

- Tira de LEDs NeoPixel:
  - Din → pin 6 (Arduino)
  - 5V → 5V Arduino (se recomienda usar fuente externa si se alimentan más de 2 LEDs)
  - GND → GND Arduino

### Notas

- Los pines 8 y 9 se usan con `SoftwareSerial` para la comunicación con el sensor.
- El pin 6 se usa con la librería `Adafruit NeoPixel`.
- El sensor y los LEDs pueden compartir alimentación si el consumo lo permite.


## Ejecución rápida
1) Arduino
```bash
cd arduino
pio run -t upload
```
3) ROS (en 3 terminales distintos)
```bash
roscore
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600 
rosrun sensor_alerta alerta_automatica
```
4) Comandos útiles:

  **Modo manual ON:**
```bash
rostopic pub /modo_manual std_msgs/Bool "data: true" -1
```
   **Forzar alerta a true:** 
```bash
rostopic pub /alerta_forzada std_msgs/Bool "data: true" -1
```
   **Forzar alerta a false:**
```bash
rostopic pub /alerta_forzada std_msgs/Bool "data: false" -1
```
   **Volver a modo automático:**
```bash
rostopic pub /modo_manual std_msgs/Bool "data: false" -1
```
## Temas ROS
- /sensor_distances — std_msgs/UInt16MultiArray (mm)
- /alerta_led — std_msgs/Bool (true = alerta)

## Parámetros
- LOW (int, mm)  – umbral inferior 
- HIGH (int, mm) – umbral superior 


