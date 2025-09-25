# Interacción con ROS

## Comunicación desde Arduino
- Archivo: `arduino_code/main.cpp`
  
- **Publica** en los topics:
  - `/sensor_distances_1`
    - Tipo: `std_msgs/UInt16MultiArray`
    - Contenido: `data[i]` es la distancia (en mm) del sensor *i* (0..7) del **array A**.
    - Se publica cuando llega un paquete completo del array A.
  - `/sensor_distances_2`
    - Tipo: `std_msgs/UInt16MultiArray`
    - Contenido: `data[i]` es la distancia (en mm) del sensor *i* (0..7) del **array B**.
    - Se publica cuando llega un paquete completo del array B.
  - `/alerta_estado`
    - Tipo: `std_msgs/Int8`
    - Estados: `0 = OK (verde)`, `1 = BAJO (azul)`, `2 = ALTO (rojo)`.
    - Solo se publica cuando cambia el estado respecto al último enviado.
  - `/alerta_led`
    - Tipo: `std_msgs/Bool`
    - Automático: `true` si `estado != 0`, `false` si `estado == 0`.
    - Manual: refleja el valor de `/alerta_forzada`.
    - Solo se publica cuando cambia respecto al último enviado.

- **Se suscribe** a los topics:
  - `/modo_manual`
    - Tipo: `std_msgs/Bool`
    - `true` → entra en modo manual (pinta inmediatamente rojo/verde según `/alerta_forzada`).  
      `false` → vuelve a automático (pintado según `/alerta_estado`).
  - `/alerta_forzada`
    - Tipo: `std_msgs/Bool`
    - En manual: `true` → LEDs rojos, `false` → LEDs verdes.  
      En automático: no afecta.
