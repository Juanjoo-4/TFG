# TFG: TeraRanger Multiflex + Arduino + ROS (Nodo C++)

## Objetivo
Arduino publica distancias; un nodo C++ evalúa si todas están dentro del rango [LOW, HIGH]. 
Si alguna está fuera o 0 (vacío) → publica alerta → Arduino pone LEDs en rojo; si no, verde.

## Requisitos
- Ubuntu 20.04 + ROS Noetic
- rosserial_python, std_msgs
- PlatformIO (para Arduino)

## Ejecución rápida
1) Arduino
   cd arduino
   pio run -t upload

2) ROS (en 2–3 terminales)
   roscore
   rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
   rosrun sensor_alerta alerta_automatica

## Temas ROS
- /sensor_distances — std_msgs/UInt16MultiArray (mm)
- /alerta_led — std_msgs/Bool (true = alerta)

## Parámetros
- LOW (int, mm)  – umbral inferior (por defecto 300)
- HIGH (int, mm) – umbral superior (por defecto 1200)


