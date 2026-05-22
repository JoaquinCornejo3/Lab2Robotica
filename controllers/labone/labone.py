from controller import Robot


# Clases y funciones matematicas
class MovingAverage:
    def __init__(self, size=5):
        self.size = size
        self.buffer = []
    def update(self, value):
        self.buffer.append(value)
        if len(self.buffer) > self.size:
            self.buffer.pop(0)
        return sum(self.buffer) / len(self.buffer)
class KalmanFilter1D:
    def __init__(self, initial_estimate=0.20, initial_P=1.0, Q=1e-4, R=0.01):
        self.x_hat = initial_estimate
        self.P = initial_P
        self.Q = Q
        self.R = R
    def predict(self, delta):
        # Al avanzar, la distancia frontal disminuye, por eso restamos
        self.x_hat = self.x_hat - delta
        self.P = self.P + self.Q
        return self.x_hat
    def update(self, z):
        K = self.P / (self.P + self.R)
        self.x_hat = self.x_hat + K * (z - self.x_hat)
        self.P = (1 - K) * self.P
        return self.x_hat, K

def sensor_to_meters(value):
    # Convierte el valor crudo del e-puck a metros reales
    normalized = value / 4096.0
    distance = 0.055 / (normalized + 0.005)
    # tuve que modificar el limite visual de un 0.20 inicial  a 1.5 para probar que se vea de antees un obstaculo
    return max(0.005, min(distance, 1.5)) 

# Inicialización del robot 
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Motores
left_motor  = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Sensores de distancia
sensor_frontal_derecho    = robot.getDevice('ps0')
sensor_frontal_izquierdo  = robot.getDevice('ps7')
sensor_diagonal_derecho   = robot.getDevice('ps1')
sensor_diagonal_izquierdo = robot.getDevice('ps6')
sensor_lateral_derecho    = robot.getDevice('ps2')
sensor_lateral_izquierdo  = robot.getDevice('ps5')
sensor_trasero_derecho    = robot.getDevice('ps3')
sensor_trasero_izquierdo  = robot.getDevice('ps4')

todos_los_sensores = [
    sensor_frontal_derecho, sensor_frontal_izquierdo,
    sensor_diagonal_derecho, sensor_diagonal_izquierdo,
    sensor_lateral_derecho, sensor_lateral_izquierdo,
    sensor_trasero_derecho, sensor_trasero_izquierdo
]
for s in todos_los_sensores:
    s.enable(timestep)

# Encoders
encoder_izquierdo = robot.getDevice('left wheel sensor')
encoder_derecho   = robot.getDevice('right wheel sensor')
encoder_izquierdo.enable(timestep)
encoder_derecho.enable(timestep)

'''
#Cambios en los umbrales de detección para probar mejor la lógica de esquiva,
# ya que con 0.20 no se veía el obstaculo a tiempo para reaccionar
'''
# Configuraciones del sistema de navegacion 
wheel_radius = 0.0205  # Radio de las ruedas en metros
max_speed = 5.0        # Se bajo de 6.28 a 5.0 (Velocidad  de los motores (rad/s))
obstacle_threshold = 0.50  # Umbral de detección de obstáculos (para esquivar)
safe_distance = 0.50  # Distancia segura para avanzar (en metros)
lateral_threshold = 0.25 # Distancia para corregir si roza la pared lateralmente

# Instancia de filtros
ma_der = MovingAverage()
ma_izq = MovingAverage()
# El filtro de Kalman se inicializa con un valor estimado de 1.5 metros
# que es el rango máximo que queremos considerar para detectar obstáculos.
# Aumentamos Q de 1e-4 a 0.01 para que reaccione muchísimo más rápido a los muros
kf = KalmanFilter1D(initial_estimate=1.5, initial_P=1.0, Q=0.01, R=0.01)

# Variables para cinematica
prev_enc_izq = 0.0
prev_enc_der = 0.0
encoders_initialized = False

# variable para consola
pasos_consola = 0
# Abrir 2 archivos, para crudo y para filtrado
# Archivo de registro
archivo_crudo = open("registro_crudo.csv", "w")
archivo_crudo.write(
    "tiempo,"
    "frontal_der,frontal_izq,"
    "diagonal_der,diagonal_izq,"
    "lateral_der,lateral_izq,"
    "trasero_der,trasero_izq,"
    "encoder_izq,encoder_der\n"
)

# Archivo de registro filtrado o procesado
archivo_procesado = open("registro_procesado.csv", "w")
archivo_procesado.write(
    "tiempo,"
    "dist_front_raw,"
    "dist_filtered,"
    "dist_kalman,"
    "accion\n"
)

# Loop principal
while robot.step(timestep) != -1:
    t = robot.getTime()
    if t >= 60.0:  # AUMENTAR EL TIEMPO PARA VER COMO EVITAR MEJOR LOS OBSTACULOS.
        break
    
    # LECTURA DE SENSORES CRUDAS
    frontal_der    = sensor_frontal_derecho.getValue()
    frontal_izq    = sensor_frontal_izquierdo.getValue()
    diagonal_der   = sensor_diagonal_derecho.getValue()
    diagonal_izq   = sensor_diagonal_izquierdo.getValue()
    lateral_der    = sensor_lateral_derecho.getValue()
    lateral_izq    = sensor_lateral_izquierdo.getValue()
    trasero_der    = sensor_trasero_derecho.getValue()
    trasero_izq    = sensor_trasero_izquierdo.getValue()
    enc_izq        = encoder_izquierdo.getValue()
    enc_der        = encoder_derecho.getValue()

    # CONVERSION A METROS
    # Conversion de sensores basicos (frontales y laterales)
    dist_front_r = sensor_to_meters(frontal_der)
    dist_front_l = sensor_to_meters(frontal_izq)
    dist_left = sensor_to_meters(lateral_izq)
    dist_right = sensor_to_meters(lateral_der)
    dist_diag_r = sensor_to_meters(diagonal_der)
    dist_diag_l = sensor_to_meters(diagonal_izq)
    dist_back_r = sensor_to_meters(trasero_der)
    dist_back_l = sensor_to_meters(trasero_izq)

    # Distancia cruda frontal minima (45 grados al frente)
    dist_front_raw = min(dist_front_r, dist_front_l, dist_diag_r, dist_diag_l)

    # FILTRO SIMPLE (MEDIA MOVIL)
    dist_front_filtered = (ma_der.update(dist_front_r) + ma_izq.update(dist_front_l)) / 2.0

    # CALCULO DE AVANCE CON LOS ENCONDERS
    if not encoders_initialized:
        prev_enc_izq = enc_izq
        prev_enc_der = enc_der
        encoders_initialized = True

    delta_left  = (enc_izq - prev_enc_izq) * wheel_radius
    delta_right = (enc_der - prev_enc_der) * wheel_radius
    delta_d = (delta_left + delta_right) / 2.0  # Avance neto en metros

    prev_enc_izq = enc_izq
    prev_enc_der = enc_der

    # FILTRO DE KALMAN
    kf.predict(delta_d)
    # Se corrige utilizando la medicion en metros cruda del sensor
    dist_kalman, kalman_gain = kf.update(dist_front_raw)
    dist_kalman = max(0.005, dist_kalman)

    '''
    Logica de navegacion basada en la distancia filtrada por Kalman, de manera reactiva
    Se cambio que el giro sea asimetrico, 
    para que la rueda exterior gire mas rapido 
    y asi alejarse del obstaculo de manera mas efectiva

    Tambien se incorporan las diagonales para decidir hacia donde girar, ya que se notaba 
    que el robot tipo se apoyaba en el obstaculo para moverse.

    Se agrega ahora una consideracion de la parte trasera, a fin de cuentas se evalua por cada hemisferio
    es decir lateral + diagonal + trasero, para decidir hacia donde hay mas peligro y asi girar en la direccion opuesta.

    '''
    peligro_izq = min(dist_left, dist_diag_l, dist_back_l)
    peligro_der = min(dist_right, dist_diag_r, dist_back_r)

    # Calculamos la distancia más pequeña de todo el contorno para saber si está encajonado
    dist_minima_total = min(dist_kalman, peligro_izq, peligro_der)

    if dist_kalman > safe_distance:
        # FRENTE LIBRE: Pero verificamos que no estemos rozando la pared
        if dist_left < lateral_threshold:
            # Rozando lado izquierdo -> Curva suave a la derecha
            left_speed  = max_speed * 0.6
            right_speed = max_speed * 0.4
            accion = "CORRIGIENDO_DER"
        elif dist_right < lateral_threshold:
            # Rozando lado derecho -> Curva suave a la izquierda
            left_speed  = max_speed * 0.4
            right_speed = max_speed * 0.6
            accion = "CORRIGIENDO_IZQ"
        else:
            # Completamente libre
            left_speed  = max_speed * 0.6
            right_speed = max_speed * 0.6
            accion = "AVANZAR"

    elif dist_kalman <= obstacle_threshold:
        # FRENTE BLOQUEADO se considera una evasión asimétrica analizando los 8 sensores
        # si hay una Trampa tipo un vertice, osea si ocurriese que la distancia es críticamente baja, gira sobre su propio eje.
        if dist_minima_total < 0.12:
            if peligro_izq < peligro_der:
                left_speed  =  max_speed * 0.8
                right_speed = -max_speed * 0.8 
                accion = "GIRO_VERTICE_DER"
            else:
                left_speed  = -max_speed * 0.8
                right_speed =  max_speed * 0.8 
                accion = "GIRO_VERTICE_IZQ"
        else:
            if peligro_izq < peligro_der:
                left_speed  =  max_speed * 0.5
                right_speed = -max_speed * 0.3
                accion = "GIRAR_DER"
            else:
                left_speed  = -max_speed * 0.3
                right_speed =  max_speed * 0.5
                accion = "GIRAR_IZQ"
    else:
        # ZONA DE TRANSICION (Desaceleración)
        factor = (dist_kalman - obstacle_threshold) / (safe_distance - obstacle_threshold)
        factor = max(0.4, factor) #Nunca frena por debajo del 40% de su potencia
        left_speed  = max_speed * 0.5 * factor
        right_speed = max_speed * 0.5 * factor
        accion = "DESACELERAR"

    # Enviar comandos a los motores
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    '''
    # Ver datos crudos iniciales en consola
    print(
        f"t={t:.2f} | "
        f"F_der={frontal_der:.1f} F_izq={frontal_izq:.1f} | "
        f"D_der={diagonal_der:.1f} D_izq={diagonal_izq:.1f} | "
        f"L_der={lateral_der:.1f} L_izq={lateral_izq:.1f} | "
        f"T_der={trasero_der:.1f} T_izq={trasero_izq:.1f} | "
        f"Enc_izq={enc_izq:.3f} Enc_der={enc_der:.3f}"
    )
    '''
    # Registro datos originales crudos
    archivo_crudo.write(
        f"{t:.3f},"
        f"{frontal_der:.2f},{frontal_izq:.2f},"
        f"{diagonal_der:.2f},{diagonal_izq:.2f},"
        f"{lateral_der:.2f},{lateral_izq:.2f},"
        f"{trasero_der:.2f},{trasero_izq:.2f},"
        f"{enc_izq:.4f},{enc_der:.4f}\n"
    )

    # Registro datos procesados 
    archivo_procesado.write(
        f"{t:.3f},"
        f"{dist_front_raw:.5f},"
        f"{dist_front_filtered:.5f},"
        f"{dist_kalman:.5f},"
        f"{accion}\n"
    )
    '''
    # Impresion para verificar que el filtro de Kalman y la acción se actualizan correctamente.
    if int(t * 100) % 100 == 0: # Imprime aproximadamente cada 1 segundo
        print(f"t={t:.1f}s | Kalman={dist_kalman:.3f}m | Acción={accion}")
    '''
    # IMPRESIÓN SIN DESFASE (basada en ciclos, no en tiempo flotante)
    pasos_consola += 1
    if pasos_consola % 20 == 0: # Imprime suavemente cada 20 pasos de simulación
        print(f"t={t:.2f}s | Kalman={dist_kalman:.3f}m | Acción={accion}")
# Cerrar ambos archivos al finalizar la simulación
archivo_crudo.close()
archivo_procesado.close()
