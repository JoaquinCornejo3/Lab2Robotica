import csv
from controller import Robot

RobotEpuck = Robot()
TimestepMuestreo = int(RobotEpuck.getBasicTimeStep())
VelocidadMaxima = 6.28

# Motores
MotorIzquierdo = RobotEpuck.getDevice('left wheel motor')
MotorDerecho   = RobotEpuck.getDevice('right wheel motor')
MotorIzquierdo.setPosition(float('inf'))
MotorDerecho.setPosition(float('inf'))
MotorIzquierdo.setVelocity(0.0)
MotorDerecho.setVelocity(0.0)

# Encoders
EncoderIzquierdo = RobotEpuck.getDevice('left wheel sensor')
EncoderIzquierdo.enable(TimestepMuestreo)
EncoderDerecho = RobotEpuck.getDevice('right wheel sensor')
EncoderDerecho.enable(TimestepMuestreo)

# Sensores de Proximidad
SensorFrontalDerecho    = RobotEpuck.getDevice('ps0')
SensorDiagonalDerecho   = RobotEpuck.getDevice('ps1')
SensorLateralDerecho    = RobotEpuck.getDevice('ps2')
SensorTraseroDerecho    = RobotEpuck.getDevice('ps3')
SensorTraseroIzquierdo  = RobotEpuck.getDevice('ps4')
SensorLateralIzquierdo  = RobotEpuck.getDevice('ps5')
SensorDiagonalIzquierdo = RobotEpuck.getDevice('ps6')
SensorFrontalIzquierdo  = RobotEpuck.getDevice('ps7')

for sensor in [
    SensorFrontalDerecho, SensorDiagonalDerecho,
    SensorLateralDerecho, SensorTraseroDerecho,
    SensorTraseroIzquierdo, SensorLateralIzquierdo,
    SensorDiagonalIzquierdo, SensorFrontalIzquierdo
]:
    sensor.enable(TimestepMuestreo)

UmbralDeteccion = 200
UmbralPeligro   = 600
Histeresis      = 60

PasosGirando = 0
LimiteGiro   = 35
KpCentrado   = 0.003

# Variable de retención de estado
DireccionGiroBloqueada = None

#limite finito
LimiteMuestras = 1500 # 1000 son como 32 seg creo
ContadorMuestras = 0

# Se crea el archivo CSV y se escriben los encabezados
archivo_csv = open('datos_sensores.csv', mode='w', newline='')
escritor_csv = csv.writer(archivo_csv)
escritor_csv.writerow([
    'Tiempo_s', 'ValFD', 'ValDD', 'ValLD', 'ValTD', 
    'ValTI', 'ValLI', 'ValDI', 'ValFI', 'EncIzq', 'EncDer'
])

while RobotEpuck.step(TimestepMuestreo) != -1:

    ContadorMuestras += 1

    ValFD  = SensorFrontalDerecho.getValue()
    ValDD  = SensorDiagonalDerecho.getValue()
    ValLD  = SensorLateralDerecho.getValue()
    ValTD  = SensorTraseroDerecho.getValue()
    ValTI  = SensorTraseroIzquierdo.getValue()
    ValLI  = SensorLateralIzquierdo.getValue()
    ValDI  = SensorDiagonalIzquierdo.getValue()
    ValFI  = SensorFrontalIzquierdo.getValue()

    ValEncIzq = EncoderIzquierdo.getValue()
    ValEncDer = EncoderDerecho.getValue()
    
    if ContadorMuestras > LimiteMuestras:
        # Detenemos los motores para que el robot no siga chocando
        MotorIzquierdo.setVelocity(0.0)
        MotorDerecho.setVelocity(0.0)
        print(f"Éxito: Se registraron {LimiteMuestras} datos. Simulación terminada.")
        # El comando break rompe el while y finaliza el script
        break
    
    # Capturamos el tiempo exacto de simulación
    TiempoActual = RobotEpuck.getTime()
    
    # Esto ocurre exactamente a la frecuencia de muestreo fs
    escritor_csv.writerow([
        TiempoActual, ValFD, ValDD, ValLD, ValTD, 
        ValTI, ValLI, ValDI, ValFI, ValEncIzq, ValEncDer
    ])

    AmenazaDerecha   = max(ValFD, ValDD, ValLD, ValTD)
    AmenazaIzquierda = max(ValFI, ValDI, ValLI, ValTI)

    # Implementación de histéresis: requiere más espacio libre para dejar de girar
    UmbralActual = (UmbralDeteccion - Histeresis) if DireccionGiroBloqueada else UmbralDeteccion
    FrenteBloqueado = ValFD > UmbralActual or ValFI > UmbralActual
    PeligroCritico = ValFD > UmbralPeligro or ValFI > UmbralPeligro

    if PasosGirando > LimiteGiro:
        VelocidadIzquierda = -VelocidadMaxima * 0.5
        VelocidadDerecha   = -VelocidadMaxima * 0.5
        PasosGirando = 0
        DireccionGiroBloqueada = None

    elif PeligroCritico or FrenteBloqueado:
        # Congelamiento de la decisión direccional
        if DireccionGiroBloqueada is None:
            if AmenazaIzquierda >= AmenazaDerecha:
                DireccionGiroBloqueada = 'DERECHA'
            else:
                DireccionGiroBloqueada = 'IZQUIERDA'

        MultiplicadorAvance = 0.7 if PeligroCritico else 0.6
        MultiplicadorRetroceso = -0.7 if PeligroCritico else -0.4

        if DireccionGiroBloqueada == 'DERECHA':
            VelocidadIzquierda = VelocidadMaxima * MultiplicadorAvance
            VelocidadDerecha   = VelocidadMaxima * MultiplicadorRetroceso
        else:
            VelocidadIzquierda = VelocidadMaxima * MultiplicadorRetroceso
            VelocidadDerecha   = VelocidadMaxima * MultiplicadorAvance
            
        PasosGirando += 1

    else:
        PasosGirando = 0
        DireccionGiroBloqueada = None
        
        ParedIzquierda = max(ValLI, ValDI)
        ParedDerecha = max(ValLD, ValDD)
        
        ErrorPosicion = ParedIzquierda - ParedDerecha
        
        # Banda muerta: supresión de microajustes
        if abs(ErrorPosicion) < 150:
            ErrorPosicion = 0
            
        AjusteDireccional = ErrorPosicion * KpCentrado
        VelocidadBase = VelocidadMaxima * 0.75
        
        VelocidadIzquierda = VelocidadBase + AjusteDireccional
        VelocidadDerecha = VelocidadBase - AjusteDireccional
        
        VelocidadIzquierda = max(min(VelocidadIzquierda, VelocidadMaxima), -VelocidadMaxima)
        VelocidadDerecha = max(min(VelocidadDerecha, VelocidadMaxima), -VelocidadMaxima)

    MotorIzquierdo.setVelocity(VelocidadIzquierda)
    MotorDerecho.setVelocity(VelocidadDerecha)