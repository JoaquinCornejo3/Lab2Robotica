# Laboratorio 2 - Navegación reactiva con filtrado y fusión de sensores en Webots

## Objetivo
Implementar un sistema básico de navegación reactiva en Webots para un
robot móvil diferencial, utilizando sensores de distancia y encoders de rueda,
aplicando filtrado sobre las mediciones y empleando un filtro de Kalman para
estimar la distancia frontal a obstáculos y mejorar la toma de decisiones.

## Datos 
**Universidad:** Pontificia Universidad Católica de Valparaíso

### Integrantes:
1. Joaquín Cornejo Fernández.
2. Vicente Martinez Estay.
3. Darío Fuentes Ponce.

**Curso:** (ICI450-2) Robotica y sistemas autonomos.

**Paralelo:** 2

## Descripción del Robot y Sensores
Se utiliza el robot diferencial **E-puck** equipado con:
* **Sensores de Proximidad:** 8 sensores infrarrojos (`ps0` a `ps7`) para percibir obstáculos. Se agrupan lógicamente en cuadrantes (frontal, diagonal, lateral, trasero) para evaluar amenazas.
* **Encoders:** Sensores habilitados en las ruedas izquierda (`left wheel sensor`) y derecha (`right wheel sensor`) para registrar la rotación de los motores.

## Frecuencia de Muestreo
Durante la simulación, las lecturas de los sensores de distancia y de los encoders de las ruedas se registraron de forma estrictamente síncrona con el paso de simulación del entorno Webots.

* **Tiempo de muestreo ($T_{s}$):** 0.032 s (32 ms), extraído dinámicamente mediante la función `getBasicTimeStep()`
* **Frecuencia de muestreo ($f_{s}$):** 31.25 Hz, calculada a partir de la relación $f_{s}=\frac{1}{T_{s}}$
* **Cantidad de muestras registradas:** El experimento registró un total de 1031 muestras, lo que equivale a una ventana de observación de 33.0 segundos de navegación ininterrumpida.

## Análisis de Señales y Estimación de Avance
Los sensores frontales (ps0, ps7) muestran un valor de ruido de fondo de entre 60 y 80 unidades cuando no hay obstáculos. Cuando el robot se aproxima a una pared, los valores aumentan rápidamente, superando las 700 unidades en colisiones inminentes. Debido a que la respuesta del sensor IR no es lineal, interpretar directamente los valores crudos puede llevar a decisiones poco precisas. Por eso, en el código esos valores se transforman a metros (con un límite util de 0.10 m).

#### Odometría con encoders
El desplazamiento lineal y el giro angular se calculan a partir de la diferencia entre lecturas consecutivas de los encoders:

    DeltaSIzq = RadioRueda * DeltaThetaIzq
    DeltaSDer = RadioRueda * DeltaThetaDer
    AvanceLineal = (DeltaSIzq + DeltaSDer) / 2.0
    GiroAngular  = (DeltaSDer - DeltaSIzq) / DistanciaRuedas

Con RadioRueda = 0.0205 m y DistanciaRuedas = 0.052 m. Este modelo cinemático diferencial se utiliza para actualizar la predicción del filtro de Kalman en cada paso (sabiendo exactamente cuánto avanzó físicamente el robot).

## Filtrado y Fusión Sensorial (Filtro de Kalman)

<img width="1389" height="989" alt="image" src="https://github.com/user-attachments/assets/0c5c6b47-109b-42cd-b68c-8a5e8584dd1b" />

### Filtro EMA (Media Móvil Exponencial)
Antes de ingresar al filtro de Kalman, los valores crudos de los sensores frontales se suavizan con un filtro EMA para mitigar el ruido:

`y[k] = α · x[k] + (1 − α) · y[k−1]`

Con `α = 0.3`. Al utilizar este valor de α, el sistema da mayor importancia a las mediciones anteriores que a los cambios bruscos instantáneos. De esta manera, se pueden eliminar fluctuaciones espurias en los datos sin provocar un retraso significativo en la respuesta del sensor.

### Filtro de Kalman 1D
El filtro de Kalman estima la distancia frontal al obstáculo más cercano, fusionando el modelo de movimiento (odometría) con la medición del sensor filtrado.

- Estado: `d_kalman` = distancia estimada al obstáculo (metros).

- Medición: `z_k = min(DistFD_ema, DistFI_ema)` que seria la menor distancia detectada por los sensores frontales filtrados.

#### Etapa de Predicción (Actualización de Tiempo):
En esta etapa, el filtro "predice" cómo cambió la distancia basándose únicamente en el movimiento del robot. Como el robot avanza, se asume que el obstáculo se acerca en esa misma proporción. Simultáneamente, la incertidumbre del sistema (`P_pred`) aumenta debido al ruido natural del movimiento. La distancia se limita estrictamente entre 0 y 0.10 m.

```
DeltaD_k = -AvanceLineal
d_pred = d_kalman + DeltaD_k
d_pred = max(0.0, min(d_pred, 0.10))
P_pred = P_kalman + Q_kalman
```
Luego se calcula la ganancia de kalman, que se define como `K_k = P_pred / (P_pred + R_kalman)`.

#### Etapa de Corrección (Actualización de Medición):
En esta etapa, la predicción anterior se corrige utilizando la lectura real del sensor . Si el sensor es muy ruidoso, el filtro confiará más en la predicción; si la predicción es incierta, confiará más en el sensor. Finalmente, la incertidumbre se reduce tras incorporar el nuevo dato.
```
d_kalman = d_pred + K_k * (z_k - d_pred)
d_kalman = max(0.0, min(d_kalman, 0.10))
P_kalman = (1.0 - K_k) * P_pred
```
Para el ajuste y sintonización de este filtro, se definieron los siguientes parámetros fijos de inicialización e incertidumbre:

| Parámetro | Valor | Significado|
| :--- | :---: | ---: |
Q_kalman | 0.0001| Ruido de proceso (muy baja incertidumbre en la odometría).
R_kalman | 0.0015 | Ruido de medición (incertidumbre del sensor IR convertido a metros).
d_kalman (Inicial) | 0.10 m | Estado inicial (se asume 10 cm libres al frente).
P_kalman (Inicial) | 1.0 | Covarianza inicial (alta incertidumbre al iniciar).


## Lógica de Navegación Reactiva
El sistema lleva a cabo un control reactivo por medio de una máquina de estados finitos implícita que se basa en umbrales de distancia y odometría.

- Avance y Centrado Proporcional: En lugares sin obstáculos frontales, la tracción funciona al 75% de la velocidad máxima. Para calcular el error de posición transversal, se sustraen las mediciones máximas de los sensores laterales izquierdo y derecho. Para inyectar un diferencial de velocidad entre los motores y mantener el chasis a la misma distancia de las paredes, se emplea un control proporcional (Kp = 0.005). Para magnitudes de error menores a 150 unidades crudas, aplica una banda muerta que elimina las oscilaciones de corrección en trayectorias rectas.

- Evasión de Obstáculos: Se activa cuando hay un bloqueo en el vector frontal (cuando la distancia de Kalman es menor a 0.08 m o las lecturas diagonales son más de 600 unidades). La amplitud de la amenaza en los dos lados laterales es comparada por el algoritmo, que luego orienta la rotación hacia el vector con mayor despeje. La maniobra de evasión funciona en circuito cerrado, incorporando la velocidad angular cinemática. La rotación se mantiene de manera continua hasta que se acumula un giro absoluto de π/2 radianes para obstrucciones estándar, o de π/3 radianes en caso de una evaluación de peligro crítico (cuando la distancia es menor a 0.05 m). Utiliza factores multiplicadores asimétricos para realizar un giro dinámico con radio de curvatura.

- Retroceso de Seguridad: Una vez ha terminado el arco odómetrico de evasión, el robot impone una traslación inversa simétrica a velocidad media durante 10 iteraciones consecutivas del bucle de control. Esta rutina elimina la inercia de rotación, elimina los bloqueos cinemáticos por fricción lateral y restablece el volumen espacial de seguridad antes de iniciar nuevamente el movimiento hacia adelante.

## Escenarios de Prueba y Resultados
### Escenario: FACIL
<img width="437" height="434" alt="image" src="https://github.com/user-attachments/assets/d70b3d72-ec55-4083-9b77-8931e891cc2c" />

### Escenario: DIFICIL
<img width="447" height="445" alt="image" src="https://github.com/user-attachments/assets/57dc28b7-a252-4553-9324-73de21ae39f0" />


## Conclusiones
Las siguientes conclusiones se deducen a partir de la aplicación de la lógica reactiva y las pruebas que se han hecho en diversos ambientes de simulación:

1. **Eficacia de la fusión sensorial (filtro de Kalman):** La aplicación del filtro de Kalman unidimensional se mostró como el elemento más efectivo para mantener estable la navegación. El sistema fue capaz de calcular la distancia real hacia los obstáculos (`d_kalman`) de forma óptima al combinar la predicción del modelo cinemático (odometría) con la corrección de los sensores infrarrojos. Esto hizo posible atenuar la no linealidad y las súbitas crecidas de ruido en las lecturas sin procesar, evitando que el robot diera giros bruscos o erráticos debido a falsos positivos.

2. **Ventajas frente al Filtro Simple (EMA):** Si bien el Filtro de Media Móvil Exponencial cumplió su función de pre-procesar y suavizar la señal de entrada, su naturaleza matemática introduce un leve retraso en la respuesta frente a obstáculos repentinos. El filtro de Kalman superó esta limitación al anticipar el acercamiento físico del robot mediante la variable `AvanceLineal`, logrando una reacción casi en tiempo real indispensable para evitar colisiones.

3. **Adaptabilidad en situaciones de diferente nivel de complejidad:**
   * **Escenario FÁCIL:** El sistema corroboró su rendimiento en lugares abiertos. La estimación de distancia permaneció estable cerca del límite útil (0.10 m), lo que posibilitó que el E-puck mantuviera un camino suave con cambios de control mínimos.
   * **Escenario DIFíCIL:** La lógica de navegación fue llevada al límite debido a la gran cantidad de obstáculos y pasillos angostos. La lógica de evasión se disparó de manera reiterada debido a la caída continua del parámetro `d_kalman`. El robot pudo sortear embudos y esquinas cerradas sin tambalearse (sin quedarse "atrapado" decidiendo hacia dónde girar) ni colisionar con los pilares, gracias a que la señal se estabilizó antes.


## Instrucciones de Ejecución
1.  Clonar este repositorio y abrir Webots.
2.  Cargar el mundo deseado (`facil.wbt` o `dificil.wbt`) desde la interfaz.
3.  Asegurar que el E-puck tenga asignado el controlador `labone.py`.
4.  Cargar el mundo deseado y darle play.
