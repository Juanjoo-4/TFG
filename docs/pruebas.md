# Prueba 1: Publicación continua de distancias desde ambos arrays de sensores

**Objetivo:** 

Verificar que el Arduino publica correctamente las distancias en `/sensor_distances_1` y `/sensor_distances_2`.

**Procedimiento:**  
1. Conectar Arduino Mega 2560 y los arrays de sensores al PC.
2. Abrir una terminal y lanzar `roscore`.
3. Terminal 2, ejecutar (ajustar el puerto):
```bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 
```
4. Terminal 3:
```bash
rostopic echo /sensor_distances_1
```
5. Terminal 4:
```bash
rostopic echo /sensor_distances_2
```

**Resultado esperado:**
- En cada tópico se muestran arrays de 8 valores (mm) correspondientes a su array.
- Los valores están dentro del rango aproximado de medición (por ejemplo, 150–500 mm).
- Se publica cuando llega un paquete completo por cada array.

**Resultado real:**

La comunicación entre el Arduino Mega 2560 Pro y ROS se mantiene estable durante toda la prueba.  
Los tópicos `/sensor_distances_1` y `/sensor_distances_2` publican correctamente los arrays de 8 valores por sensor, con lecturas coherentes y sin interrupciones.  

Las distancias se mantienen dentro del rango de detección previsto (aproximadamente entre 150 mm y 400 mm), y se observa una actualización continua y fluida de los datos en ROS.  

<img width="1755" height="617" alt="Captura_Prueba1" src="https://github.com/user-attachments/assets/0750284b-09d8-43f8-9ae7-da8bd1d209a4" />

[![Descripción del video](https://img.youtube.com/vi/KGx-cfAc6IM/0.jpg)](https://youtu.be/KGx-cfAc6IM)

Para verificar la tasa de envío de mensajes, se ejecutó el comando:
```bash
rostopic hz /sensor_distances_1
```
 se observa una frecuencia promedio de aproximadamente 6,33 Hz, con una desviación estándar muy baja, lo que indica una transmisión estable y regular de los paquetes de
 datos por parte del Arduino.
 
<img width="735" height="433" alt="Hz_Prueba1" src="https://github.com/user-attachments/assets/567bc842-9226-462c-9517-dca8f670b2e8" />

El siguiente comando:
```bash
rostopic hz /sensor_distances_1
```
se aprecia un ancho de banda promedio de ≈ 180 B/s, con un tamaño de mensaje constante de 28 bytes. Esto confirma que cada publicación contiene únicamente los valores numéricos esperados (8 distancias en formato entero) y que no existen sobrecargas innecesarias en la transmisión.

Por último, se utilizó la herramienta rqt_plot para representar la evolución temporal de los valores recibidos en los sensores 4 y 5 de ambos arrays. En el siguiente video se muestra la estabilidad de las lecturas, con variaciones suaves y consistentes entre los sensores, lo que evidencia un comportamiento correcto y libre de ruido significativo.

[![rqt plot - Demostración](https://img.youtube.com/vi/3h9d-gzrbG8/0.jpg)](https://youtu.be/3h9d-gzrbG8)

# Prueba 2: Evaluación de alerta (modo automático)

**Objetivo:**

Confirmar que el Arduino evalúa correctamente el estado global y publica en `/alerta_estado` y `/alerta_led` solo cuando hay cambios.

**Procedimiento:**  
1. Dejar el sistema corriendo como en la Prueba 1.
2. Abrir nuevos terminales para observar:
```bash
rostopic echo /alerta_estado
rostopic echo /alerta_led
```
3. Simular diferentes situaciones con los sensores:
  - OK (0): Mantener todas las lecturas en el rango [150, 500] (ambos umbrales incluidos).
  - BAJO (1): Hacer que alguna lectura sea < 150.
  - ALTO (2): Hacer que alguna lectura sea 0 (vacío) o > 500.

**Resultado esperado:**
- Observar los diferentes estados de `/alerta_estado` y comprobar el funcionamiento de los LEDs también:
  - 0 → LEDs verdes
  - 1 → LEDs azules
  - 2 → LEDs rojos
- `/alerta_led` en automático es true si estado != 0, false si estado == 0.
- Tanto `/alerta_estado` como `/alerta_led` solo emiten cuando cambia el valor.

Prioridad de combinación entre arrays: ALTO (2) > BAJO (1) > OK (0).

**Resultado real:**

Durante la ejecución de la prueba se comprobó que el sistema detectaba correctamente los cambios de estado en función de las distancias medidas por los sensores. En el primer vídeo se puede observar el cambio de estado al situar una caja dentro del rango inferior de detección, el valor publicado en el tópico /alerta_estado cambió de 0 a 1, mientras que el mensaje /alerta_led alternó entre True y False en correspondencia con las variaciones de los sensores.

[![Prueba 2 - Frontal](https://img.youtube.com/vi/-78qRNNndis/maxresdefault.jpg)](https://youtu.be/-78qRNNndis)

En el segundo video se observa el mismo funcionamiento, pero en este caso desde los sensores laterales.

[![Prueba 2 - Lateral](https://img.youtube.com/vi/GVIAIHRoIYA/0.jpg)](https://youtu.be/GVIAIHRoIYA)

En el último vídeo, se observa como se encienden los LEDs igualmente pero debido a la proximidad de la caja por la esquina trasera.

[![Prueba 2 - Esquina trasera](https://img.youtube.com/vi/p5DWbBj6l8M/0.jpg)](https://youtu.be/p5DWbBj6l8M)

# Prueba 3: Evaluación de modo manual 

**Objetivo:**

Verificar que en modo manual el sistema deja de pintar automáticamente y obedece a /alerta_forzada.

**Procedimiento:**  
1. Activar modo manual:
```bash
rostopic pub /modo_manual std_msgs/Bool "data: true" -1
```
2. Forzar `/alerta_forzada` a `true` (LEDs rojos):
```bash
rostopic pub /alerta_forzada std_msgs/Bool "data: true" -1
```
3. Forzar `/alerta_forzada` a `false` (LEDs verdes):
```bash
rostopic pub /alerta_forzada std_msgs/Bool "data: false" -1
```
4. Volver a automático:
```bash
rostopic pub /modo_manual std_msgs/Bool "data: false" -1
```
**Resultado esperado:**
- En manual, el color depende solo de `/alerta_forzada`:
  - true → rojo
  - false → verde
- `/alerta_led` refleja el valor de `/alerta_forzada` mientras el modo sea manual.
- Al volver a automático, los LEDs vuelven a depender de `/alerta_estado`.

**Resultado real:**

En el primer vídeo se muestra la activación del modo manual. Inicialmente, el valor de /alerta_forzada es False y los LEDs permanecen en color verde. Al cambiar el valor de /alerta_forzada a True, los LEDs pasan inmediatamente a rojo, confirmando que el sistema obedece la orden forzada enviada desde ROS. Posteriormente, al acercar un objeto a los sensores, se comprueba que el modo automático no interviene, ya que los LEDs permanecen rojos pese a la variación de distancia.

[![Prueba 3 - Modo manual TRUE](https://img.youtube.com/vi/cZ9v-GQI4_I/0.jpg)](https://youtu.be/cZ9v-GQI4_I)

En el segundo vídeo se observa se acercó un objeto a los sensores para comprobar que el sistema no respondía automáticamente mientras el modo manual está activo. A continuación, al volver /alerta_forzada a False, los LEDs cambian nuevamente a verde. Finalmente, se desactiva el modo manual y el sistema recupera la lógica automática: los LEDs pasan a azul al detectar un objeto cercano, demostrando que el comportamiento vuelve a depender de las medidas reales de los sensores.

[![Prueba 3 - Modo manual FALSE](https://img.youtube.com/vi/VqqzFcgsXv8/0.jpg)](https://youtu.be/VqqzFcgsXv8)


# Prueba 4: Tolerancia a pérdida de datos de un array

**Objetivo:**

Verificar que si un array deja de enviar datos, el sistema lo trata como BAJO (1) tras 250 ms.

**Procedimiento:**  
1. Con el sistema en marcha, desconectar temporalmente una de las dos líneas TX de los arrays o apagarlo.
2. Observar:
```bash
rostopic echo /alerta_estado
rostopic echo /alerta_led
```
**Resultado esperado:**
- A los ~250 ms (TIEMPO_MAXIMO_SIN_DATOS_MS) el estado del array faltante se considera BAJO (1).
- El estado global respeta la prioridad:
  - Si el otro array está en ALTO (2) → global 2.
  - Si no hay ALTO, pero falta un array → global 1.
- LEDs se ponen azules (o rojos si el otro array está en ALTO).
- `/alerta_led` en automático será true mientras estado != 0.

**Resultado real:**

En el vídeo correspondiente se observa que, inicialmente, los sensores actualizan sus datos de forma continua en función de las lecturas de los sensores, mientras el valor del tópico /alerta_estado permanece en 0, indicando un funcionamiento normal.

A continuación, al desconectar la alimentación de los sensores, los LEDs cambian inmediatamente a color azul, señalando la detección de una condición anómala. Simultáneamente, en la terminal se aprecia cómo el valor de /alerta_estado pasa a 1 y las lecturas de los sensores dejan de actualizarse, confirmando que el sistema identifica correctamente la pérdida de comunicación y responde de manera visual mediante el color azul.

[![Prueba 4](https://img.youtube.com/vi/vwm33qtRRzA/0.jpg)](https://youtu.be/vwm33qtRRzA)

