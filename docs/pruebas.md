# ✅ Prueba 1: Publicación de distancias desde Arduino

**Objetivo:**  
Verificar que el Arduino publica correctamente las distancias en `/sensor_distances`.

**Procedimiento:**  
1. Conectar Arduino y sensor al PC.
2. Abrir una terminal y lanzar `roscore`.
3. En otra terminal, ejecutar:
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
