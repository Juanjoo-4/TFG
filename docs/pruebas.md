# ✅ Prueba 1: Publicación de distancias desde Arduino

**Objetivo:**  
Verificar que el Arduino publica correctamente las distancias en `/sensor_distances`.

**Procedimiento:**  
1. Conectar Arduino y sensor al PC.
2. Abrir una terminal y lanzar `roscore`.
3. En otra terminal, ejecutar:
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
```
4. Abrir una tercera terminal y ejecutar:
```bash
rosrun sensor_alerta alerta_automatica
```
5. En otra terminal ejecutar:
```bash
rostopic echo /sensor_distances
```

**Resultado esperado:**
- Se muestran arrays de 8 valores numéricos (uno por sensor).
- Los valores están dentro del rango aproximado de medición (por ejemplo, 300–1500 mm).
- No deben ser todos 0 a menos que el sensor esté en vacío.

**Resultado real:**

# ✅ Prueba 2: Evaluación de alerta con nodo ROS

**Objetivo:**
Confirmar que el nodo ROS alerta_automatica.cpp evalúa correctamente los datos y publica en `/alerta_led`.

**Procedimiento:**  
1. Conectar Arduino y sensor al PC.
2. Abrir una terminal y lanzar `roscore`.
3. En otra terminal, ejecutar:
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
```
4. Abrir una tercera terminal y ejecutar:
```bash
rosrun sensor_alerta alerta_automatica
```
5. En otra terminal ejecutar:
```bash
rostopic echo /alerta_led
```
6. Simular diferentes situaciones con los sensores:
- Todos los sensores apuntando a una superficie dentro del rango.
- Uno o más sensores apuntando a una superficie fuera del rango.
- Uno o más sensores apuntando al vacío.

**Resultado esperado:**
- `false` si todos los sensores están dentro del rango válido.
- `true` si alguno está fuera del rango o da 0.

**Resultado real:**


