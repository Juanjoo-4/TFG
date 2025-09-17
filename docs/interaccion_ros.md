# Interacción con ROS

## Comunicación desde Arduino
-Archivo: `arduino_code/main.cpp`
- **Publica** en el topic:
  - `/sensor_distances`
  - Tipo: `std_msgs/UInt16MultiArray`
  - Contenido: `data[i]` representa la distancia (en milímetros) del sensor i (de 0 a 7)

- **Se suscribe** al topic:
  - `/alerta_led`
  - Tipo: `std_msgs/Bool`
  - Acción: si `true` → todos los LEDs en rojo; si `false` → todos en verde

---

## Nodo ROS en C++ (`sensor_alerta`)

- Archivo: `ros_ws/alerta_automatica.cpp`
- Se **suscribe** a:
  - `/sensor_distances` (`UInt16MultiArray`)

- Evalúa la alerta:
  - Define un **rango válido** `[UMBRAL_MIN, UMBRAL_MAX]` (por ejemplo: 300–1200 mm)
  - Recorre todas las distancias:
    - Si alguna está **fuera del rango** o es `0`, se considera una alerta
    - Solo si **todas** las distancias están dentro del rango, se considera estado “OK”

- Publica en:
  - `/alerta_led` (`std_msgs/Bool`)
    - `true` → alerta activada (Arduino enciende LEDs en rojo)
    - `false` → estado normal (LEDs en verde)

