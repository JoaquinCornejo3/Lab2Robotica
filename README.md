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
## Instrucciones de Ejecución
1.  Clonar este repositorio y abrir Webots.
2.  Cargar el mundo deseado (`facil.wbt`, `laboratorio2.wbt` o `dificil.wbt`) desde la interfaz.
3.  Asegurar que el E-puck tenga asignado el controlador `labone.py`.
4.  AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA.

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


La necesidad y efectividad del procesamiento avanzado se evidencia en el gráfico inferior (Distancia al Obstáculo). Mientras que la conversión directa de la señal cruda a metros produce caídas repentinas e irreales de distancia en el segundo 20.5 (llegando a marcar $\sim0.07$ m de golpe), **el Filtro de Kalman logra una estabilización óptima**.

Al integrar la predicción limpia de la odometría con la corrección ponderada de los sensores, la estimación de Kalman traza una trayectoria robusta y segura ($\sim0.08$ m), rechazando las lecturas falsas extremas del sensor crudo y permitiendo que la lógica reactiva tome decisiones de giro precisas sin oscilaciones.

## Lógica de Navegación Reactiva
bla bla bla

## Escenarios de Prueba y Resultados
bla bla bla

## Conclusiones
bla bla bla

*(BORRAR) lo de abajo*

Entrega - Repositorio GitHub
La entrega del laboratorio debe realizarse mediante un repositorio en
GitHub, público o compartido con el docente.
El repositorio debe incluir:
 * el código fuente completo del controlador del robot,
 * los archivos necesarios para ejecutar la simulación,
 * un archivo README.md

 El archivo README.md deberá actuar como informe de desarrollo del laboratorio. Por tanto, no solo debe incluir instrucciones de ejecución, sino
también la descripción de la solución implementada, el análisis de señales, la
aplicación de filtros, la estimación mediante filtro de Kalman, los resultados
obtenidos y las conclusiones del grupo.


##Contenido mínimo del README.md
* ✓ nombre del laboratorio e integrantes del grupo 
* ✓ objetivo del trabajo
* descripción del robot y sensores utilizados,
* frecuencia de muestreo empleada
* análisis de las señales registradas,
* estimación del avance mediante encoders,
* filtro simple aplicado,
* implementación del filtro de kalman
* descripción de las etapas de predicción y corrección
* lógica de navegación reactiva implementada,
* gráficos de señales crudas, filtradas y estimadas,
* resultados obtenidos en los escenarios de prueba,
* análisis final y conclusiones,
* intrucciones para ejecutar la simulación
