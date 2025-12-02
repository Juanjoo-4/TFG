# Estructura

Para la integración de todos los elementos del sistema se requirió del diseño de una estructura específica que permitiera fijar los sensores y los indicadores luminosos de manera estable y organizada. El objetivo principal fue disponer de un soporte que garantizara la correcta orientación de los módulos demedida,facilitara su conexión al microcontrolador y ofreciera una base compacta sobre la que desarrollar las pruebas.
 
Para ello se diseñaron varias piezas independientes que, una vez ensambladas, conforman la estructura completa del robot. Para facilitar la interpretación, las piezas se han clasificado en diferentes grupos:

- Las piezas identificadas como A, A1, A2 y A3 corresponden a los soportes situados en las esquinas, cuya función principal es aportar rigidez al conjunto y servir de base para la fijación de la mayoría delos sensores.

- Las piezas B, B1, B2, B3, B4 y B5 se ubican en el interior de la estructura y están destinadas a reforzar la base y mantener la geometría general. Cabe señalar que, debido a condicionantes geométricos del robot que se detallarán más adelante, una de estas piezas se empleó además como soporte para dos de los sensores, uno de los módulos y el Arduino.

- Las piezas S1 y S2 actúan como soportes específicos para los módulos de los sensores, asegurando su orientación adecuada hacia la parte frontal del robot. Cabe destacar que estas no son piezas independientes, sino que forman parte de las piezas A3 y B3. Además, cuentan con dos piezas adicionales, C1 y C2, que sirven como refuerzo para evitar que los sensores se desplacen fuera de su cubículo.

![Croquis](docs/Croquis-estructura.png) 

## Piezas tipo A

### Pieza A

La pieza A constituye el elemento base a partir del cual se derivan el resto de piezas del mismo tipo. Se trata de la más sencilla del conjunto, pero al mismo tiempo sirve como referencia dimensional para el diseño de las demás esquinas.

En su diseño se tuvieron en cuenta las restricciones de espacio impuestas por la estructura general del robot, que obligaban a limitar la extensión de la pieza a unos 25 mm en dirección horizontal desde el borde de la base y a 23 mm en altura respecto a la misma. Estas dimensiones aseguran que las piezas encajen en el espacio disponible sin interferir con otros elementos.

Desde la vista inferior de la pieza puede observarse un rebaje que responde al cono de detección
del sensor. Este detalle evita obstrucciones en el campo de medida, garantizando que la geometría de la pieza no afecte a la lectura de distancias. En esta misma vista se aprecia además varios taladros pasantes que permitirían fijar la pieza mediante tornillería, en caso de que se requiera una sujeción más robusta.
 
La vista lateral muestra por su parte la inclinación de 45º en la disposición de los sensores, lo que permite una cobertura óptima del entorno. En esta cara y en la opuesta, se ubican también dos taladros adicionales, destinados a la unión entre piezas, que aseguran la continuidad y rigidez de la estructura completa.

![Pieza A](docs/Pieza_A_1-Photoroom.png)
![Pieza A 1](docs/Pieza_A_2-Photoroom.png)
![Pieza A 2](docs/Pieza_A_3-Photoroom.png)

### Pieza A1

La pieza A1 mantiene la misma base de diseño que la pieza A, aunque incorpora una modificación importante para adaptarse a las necesidades del sistema de movilidad del robot. En este caso, se tuvo en cuenta que el eje delantero de las ruedas es pivotante, lo que implica que las ruedas pueden variar su inclinación al superar obstáculos.
 
Con el fin de evitar posibles interferencias entre la barra de sensores y el movimiento de las ruedas, se decidió eliminar la parte lateral situada justo encima de las ruedas. De esta forma se garantiza que no se produzca colisión en ningún escenario de funcionamiento, manteniendo al  mismotiempolarigidez y la función estructural de la pieza.
 
En el resto de aspectos, la pieza A1 conserva las mismas características geométricas y de montaje que la pieza A.

![Pieza A1](docs/Pieza_A1_1-Photoroom.png)

### Pieza A2

La pieza A2 reproduce el mismo diseño que la A1, ya que cumple la misma función de evitar interferencias con las ruedas. La única diferencia radica en su ubicación dentro de la estructura, situada en la esquina opuesta para mantener la simetría del conjunto.
 
De esta forma, se asegura que tanto en el lado izquierdo como en el derecho del robot se dispone del espacio necesario para el movimiento de las ruedas, sin riesgo de colisiones con la barra de sensores.

![Pieza A2](docs/Pieza_A2_1-Photoroom.png)

### Pieza A3

La pieza A3 comparte la misma base geométrica que la pieza A, ya que se ubica en la misma zona lateral de la estructura, pero en la parte trasera. La diferencia principal radica en la incorporación de un cubículo diseñado específicamente para alojar el módulo de los sensores. Este espacio se dimensionó de acuerdo conlas medidasdel TeraRanger Multiflex, asegurando un
encaje correcto y estable del dispositivo.

Para evitar que el sensor pueda desplazarse fuera de su posición, la pieza A3 trabaja conjuntamente con la pieza C1, que actúa como elemento de retención. De este modo, se garantiza que el módulo permanezca fijo incluso en caso de vibraciones o movimientos bruscos durante el funcionamiento del robot.

![Pieza A3](docs/Pieza_A3_1-Photoroom.png)
![Pieza A3 1](docs/Pieza_A3_2-Photoroom.png)
![Pieza C1](docs/Pieza_C1_1-Photoroom.png)

## Piezas tipo B

### Pieza B

La pieza B se encuentra situada entre las piezas A1 y A2, funcionando como elemento de unión y refuerzo en la parte frontal de la estructura. Al igual que ocurría con la pieza A en el grupo de esquinas, la pieza B actúa como base de referencia para el resto de elementos del mismo tipo, de manera que a partir de ella se diseñaron las variantes B1, B2, B3, B4 y B5.
 
Su función principal es garantizar la continuidad estructural entre las piezas de las esquinas y aportar la rigidez necesaria al conjunto, manteniendo la geometría de la base.

![Pieza B](docs/Pieza_B_1-Photoroom.png)

### Pieza B1

La pieza B1 se sitúa en el lateral de la estructura, en contacto directo con la pieza A. Su diseño mantiene la misma geometría base que el resto del grupo B, aportando rigidez al conjunto y actuando como elemento de unión entre los extremos. 

Como particularidad, se incorporó un taladro adicional que coincide con la zona central del lateral de la base, necesario para permitir el paso de un tornillo ubicado en esa posición. Este ajuste responde a un condicionante geométrico del chasis, garantizando así que la pieza pueda encajar sin interferencias y manteniendo la continuidad estructural del lateral. Además, se le introdujo un hueco lateral destinado a permitir la salida de los pines de conexión de la tira de LEDs.

![Pieza B1](docs/Pieza_B1-Photoroom.png)

### Pieza B2

La pieza B2 se encuentra situada en el lateral de la estructura, junto a la pieza A2. A diferencia de B1, en este caso se incorporó un espacio específico para alojar un sensor, aprovechando la geometría de la zona y evitando interferencias con la rueda.

Además, al igual que en la pieza B1, se mantuvo el taladro adicional en la parte central del lateral para permitir el paso del tornillo de fijación de la base. De esta manera, la pieza B2 cumple un doble papel: reforzar la estructura lateral y servir como soporte seguro para uno de los sensores del sistema.

![Pieza B2](docs/Pieza_B2_1-Photoroom.png)

### Pieza B3

La pieza B3 se ubica en el lateral de la estructura, adyacente a la pieza A1. Dado que en la propia A1 no fue posible colocar ni un sensor adicional ni el cubículo destinado al módulo correspondiente, se optó por integrarlo en B3.
 
Para asegurar la fijación del módulo en su posición y evitar desplazamientos durante el funcionamiento, esta pieza trabaja conjuntamente con la pieza C2, que actúa como elemento de retención adicional, de manera equivalente a la función que cumple la C1 en la pieza A3.
 
Además de esta modificación, se ajustó el sistema de unión entre piezas: en este lateral se eliminó uno de los taladros previstos para la fijación, manteniéndose únicamente el que realmente se emplea en conjunto con la pieza A1.

![Pieza B3](docs/Pieza_B3_1-Photoroom.png)
![Pieza C2](docs/Pieza_C2_1-Photoroom.png)

### Pieza B4

La pieza B4 se ubica en el lateral de la estructura, justo al lado de la pieza A3. Aunque mantiene la misma geometría base que el resto del grupo B, se incorporó una plataforma donde se colocará el Arduino y se le introdujo un hueco lateral, como se hizo en la pieza B1.

![Pieza B4](docs/Pieza_B4-Photoroom.png)

### Pieza B5

La pieza B5 se ubica en la parte trasera de la estructura, entre las piezas A3 y A. Su geometría es similar a la del resto del grupo B. La única particularidad en este caso es que en su parte central incorpora un hueco para el paso de un tornillo situado en la parte central de la parte trasera de la base. Este detalle responde a un condicionante de montaje, garantizando que la pieza pueda encajar sin interferencias y manteniendo la solidez del conjunto.

![Pieza B5](docs/Pieza_B5_1-Photoroom.png)

## Montaje de la estructura

Una vez diseñadas y fabricadas todas las piezas, se procedió al montaje completo de la estructura, la siguiente imagen muestra la disposición completa de ambas secciones una vez montados los sensores, las tiras de LEDs y el Arduino. En esta fase se verificó el correcto encaje de las piezas y la orientación de los sensores, así como el espacio disponible para el cableado y la conexión de los módulos electrónicos antes de la integración final en el robot Rafi.

![Montaje](docs/Ambas_partes.jpg)


