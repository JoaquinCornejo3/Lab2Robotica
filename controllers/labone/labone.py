from controller import Robot
import math
import csv

# ════════════════════════════════════════════════════════════════
#  FUNCIONES UTILITARIAS
# ════════════════════════════════════════════════════════════════

def inicializar_registro(nombres_senales):
    return {nombre: [] for nombre in nombres_senales}

def registrar_muestra(registro, valores):
    for clave, valor in valores.items():
        registro[clave].append(valor)

def filtro_exponencial(valor_nuevo, valor_filtrado_anterior, alpha=0.3):
    """
    Filtro de media móvil exponencial (EMA).
    alpha cercano a 1 → respuesta rápida, poco suavizado.
    alpha cercano a 0 → respuesta lenta, mucho suavizado.
    """
    return alpha * valor_nuevo + (1 - alpha) * valor_filtrado_anterior

def sensor_a_distancia(valor_sensor, max_sensor=4095.0, max_dist=0.10):
    """
    Convierte valor IR del sensor e-puck a distancia en metros.
    """
    valor_clamp = max(0.0, min(float(valor_sensor), max_sensor))
    return max_dist * (1.0 - valor_clamp / max_sensor)

def inicializar_csv(ruta_archivo, columnas):
    with open(ruta_archivo, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f, delimiter=';')
        writer.writerow(columnas)

def guardar_fila_csv(ruta_archivo, valores_fila):
    with open(ruta_archivo, 'a', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f, delimiter=';')
        writer.writerow([round(v, 6) if isinstance(v, float) else v
                         for v in valores_fila])

# ════════════════════════════════════════════════════════════════
#  INICIALIZACIÓN DEL ROBOT
# ════════════════════════════════════════════════════════════════

RobotEpuck       = Robot()
TimestepMuestreo = int(RobotEpuck.getBasicTimeStep())
VelocidadMaxima  = 6.28

# --- Motores ---
MotorIzquierdo = RobotEpuck.getDevice('left wheel motor')
MotorDerecho   = RobotEpuck.getDevice('right wheel motor')
MotorIzquierdo.setPosition(float('inf'))
MotorDerecho.setPosition(float('inf'))
MotorIzquierdo.setVelocity(0.0)
MotorDerecho.setVelocity(0.0)

# --- Encoders ---
EncoderIzquierdo = RobotEpuck.getDevice('left wheel sensor')
EncoderIzquierdo.enable(TimestepMuestreo)
EncoderDerecho = RobotEpuck.getDevice('right wheel sensor')
EncoderDerecho.enable(TimestepMuestreo)

# --- Sensores de proximidad (ps0..ps7) ---
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

# ════════════════════════════════════════════════════════════════
#  PARÁMETROS FÍSICOS DEL E-PUCK
# ════════════════════════════════════════════════════════════════

RadioRueda      = 0.0205   # metros
DistanciaRuedas = 0.052    # metros entre centros de rueda
Ts              = TimestepMuestreo / 1000.0   # segundos por paso

# ════════════════════════════════════════════════════════════════
#  PARÁMETROS DE CONTROL Y NAVEGACIÓN
# ════════════════════════════════════════════════════════════════

AlphaFiltro = 0.3
UmbralDiagonal = 600
Histeresis     = 100

UmbralDistDeteccion  = 0.08    
UmbralDistPeligro    = 0.05   
UmbralDistHisteresis = 0.012    

KpCentrado = 0.005

AnguloObjetivoNormal  = math.pi / 2    # ~90°
AnguloObjetivoPeligro = math.pi / 3    # ~60°

# ════════════════════════════════════════════════════════════════
#  PARÁMETROS DEL FILTRO DE KALMAN
# ════════════════════════════════════════════════════════════════

Q_kalman = 0.0001
R_kalman = 0.0020

d_kalman = 0.10
P_kalman = 1.0

# ════════════════════════════════════════════════════════════════
#  CONFIGURACIÓN DE TIEMPO Y CSV
# ════════════════════════════════════════════════════════════════

TiempoLimite         = 33.0
IntervaloGuardado    = 0.5
UltimoTiempoGuardado = -1.0

RutaCSV_Crudos    = 'datos_crudos.csv'
RutaCSV_Filtrados = 'datos_filtrados.csv'
RutaCSV_Kalman    = 'datos_kalman.csv'

ColumnasCrudos = [
    'ps0_FrontalDer', 'ps1_DiagonalDer',
    'ps2_LateralDer', 'ps3_TraseroDer',
    'ps4_TraseroIzq', 'ps5_LateralIzq',
    'ps6_DiagonalIzq', 'ps7_FrontalIzq',
    'Theta_Izq_rad', 'Theta_Der_rad',
    'DeltaS_Izq_m', 'DeltaS_Der_m',
    'AvanceLineal_m', 'GiroAngular_rad',
    'VelocidadLineal_ms', 'VelocidadAngular_rads',
    'timestamp_s'
]

ColumnasFiltrados = [
    'ps0_FrontalDer_crudo', 'ps0_FrontalDer_EMA',
    'ps7_FrontalIzq_crudo', 'ps7_FrontalIzq_EMA',
    'Theta_Izq_rad', 'Theta_Der_rad',
    'DeltaS_Izq_m', 'DeltaS_Der_m',
    'AvanceLineal_m', 'GiroAngular_rad',
    'VelocidadLineal_ms', 'VelocidadAngular_rads',
    'timestamp_s'
]

ColumnasKalman = [
    'distFD_cruda_m', 'distFI_cruda_m', 'distMinima_cruda_m',
    'distFD_EMA_m',   'distFI_EMA_m',   'distMinima_EMA_m',
    'prediccion_kalman_m',
    'distancia_kalman_m',
    'ganancia_kalman_K',
    'covarianza_kalman_P',
    'AvanceLineal_m',
    'timestamp_s'
]

inicializar_csv(RutaCSV_Crudos,    ColumnasCrudos)
inicializar_csv(RutaCSV_Filtrados, ColumnasFiltrados)
inicializar_csv(RutaCSV_Kalman,    ColumnasKalman)

# ════════════════════════════════════════════════════════════════
#  ESTADO INICIAL
# ════════════════════════════════════════════════════════════════

ThetaIzqAnterior = EncoderIzquierdo.getValue()
ThetaDerAnterior = EncoderDerecho.getValue()

AnguloGiradoAcum       = 0.0
DireccionGiroBloqueada = None
EnModoRetroceso        = False
PasosRetroceso         = 0
LimiteRetroceso        = 10

FD_filt = None
FI_filt = None

Registro = inicializar_registro([
    'FD_crudo', 'FI_crudo', 'FD_EMA', 'FI_EMA',
    'DD_crudo', 'DI_crudo', 'LD_crudo', 'LI_crudo',
    'TD_crudo', 'TI_crudo',
    'Theta_Izq', 'Theta_Der',
    'DeltaS_Izq', 'DeltaS_Der',
    'AvanceLineal', 'GiroAngular',
    'VelocidadLineal', 'VelocidadAngular',
    'distancia_kalman', 'covarianza_kalman', 'ganancia_kalman',
])

# ════════════════════════════════════════════════════════════════
#  BUCLE PRINCIPAL
# ════════════════════════════════════════════════════════════════

while RobotEpuck.step(TimestepMuestreo) != -1:

    TiempoActual = RobotEpuck.getTime()

    # ── CONDICIÓN DE PARADA A LOS 33 SEGUNDOS ───────────────────
    if TiempoActual >= TiempoLimite:
        MotorIzquierdo.setVelocity(0.0)
        MotorDerecho.setVelocity(0.0)
        break

    # ── 1. ENCODERS ──────────────────────────────────────────────
    ThetaIzqActual = EncoderIzquierdo.getValue()
    ThetaDerActual = EncoderDerecho.getValue()

    DeltaThetaIzq = ThetaIzqActual - ThetaIzqAnterior
    DeltaThetaDer = ThetaDerActual - ThetaDerAnterior

    DeltaSIzq = RadioRueda * DeltaThetaIzq
    DeltaSDer = RadioRueda * DeltaThetaDer

    AvanceLineal = (DeltaSIzq + DeltaSDer) / 2.0
    GiroAngular  = (DeltaSDer - DeltaSIzq) / DistanciaRuedas

    VelocidadLineal  = AvanceLineal / Ts
    VelocidadAngular = GiroAngular  / Ts

    ThetaIzqAnterior = ThetaIzqActual
    ThetaDerAnterior = ThetaDerActual

    # ── 2. SENSORES CRUDOS ───────────────────────────────────────
    ValFD = SensorFrontalDerecho.getValue()
    ValDD = SensorDiagonalDerecho.getValue()
    ValLD = SensorLateralDerecho.getValue()
    ValTD = SensorTraseroDerecho.getValue()
    ValTI = SensorTraseroIzquierdo.getValue()
    ValLI = SensorLateralIzquierdo.getValue()
    ValDI = SensorDiagonalIzquierdo.getValue()
    ValFI = SensorFrontalIzquierdo.getValue()

    # ── 3. FILTRO EMA SOBRE FRONTALES ────────────────────────────
    if FD_filt is None:
        FD_filt = ValFD
        FI_filt = ValFI

    FD_filt = filtro_exponencial(ValFD, FD_filt, AlphaFiltro)
    FI_filt = filtro_exponencial(ValFI, FI_filt, AlphaFiltro)

    # ── 4. CONVERSIÓN A DISTANCIA EN METROS ──────────────────────
    DistFD_cruda  = sensor_a_distancia(ValFD)
    DistFI_cruda  = sensor_a_distancia(ValFI)
    DistMin_cruda = min(DistFD_cruda, DistFI_cruda)

    DistFD_ema  = sensor_a_distancia(FD_filt)
    DistFI_ema  = sensor_a_distancia(FI_filt)
    DistMin_ema = min(DistFD_ema, DistFI_ema)

    z_k = DistMin_ema

    # ── 5. FILTRO DE KALMAN ──────────────────────────────────────

    d_pred = d_kalman - AvanceLineal
    d_pred = max(0.0, min(d_pred, 0.10))

    P_pred = P_kalman + Q_kalman
    K_k = P_pred / (P_pred + R_kalman)

    d_kalman = d_pred + K_k * (z_k - d_pred)
    d_kalman = max(0.0, min(d_kalman, 0.10))

    P_kalman = (1.0 - K_k) * P_pred

    # ── 6. REGISTRO EN MEMORIA ───────────────────────────────────
    registrar_muestra(Registro, {
        'FD_crudo': ValFD, 'FI_crudo': ValFI,
        'FD_EMA':   FD_filt, 'FI_EMA': FI_filt,
        'DD_crudo': ValDD, 'DI_crudo': ValDI,
        'LD_crudo': ValLD, 'LI_crudo': ValLI,
        'TD_crudo': ValTD, 'TI_crudo': ValTI,
        'Theta_Izq': ThetaIzqActual, 'Theta_Der': ThetaDerActual,
        'DeltaS_Izq': DeltaSIzq, 'DeltaS_Der': DeltaSDer,
        'AvanceLineal': AvanceLineal, 'GiroAngular': GiroAngular,
        'VelocidadLineal': VelocidadLineal, 'VelocidadAngular': VelocidadAngular,
        'distancia_kalman': d_kalman,
        'covarianza_kalman': P_kalman,
        'ganancia_kalman': K_k,
    })

    # ── 7. GUARDADO EN CSV ───────────────────────────────────────
    if TiempoActual - UltimoTiempoGuardado >= IntervaloGuardado:
        UltimoTiempoGuardado = TiempoActual

        guardar_fila_csv(RutaCSV_Crudos, [
            ValFD, ValDD, ValLD, ValTD,
            ValTI, ValLI, ValDI, ValFI,
            ThetaIzqActual, ThetaDerActual,
            DeltaSIzq, DeltaSDer,
            AvanceLineal, GiroAngular,
            VelocidadLineal, VelocidadAngular,
            TiempoActual
        ])

        guardar_fila_csv(RutaCSV_Filtrados, [
            ValFD, FD_filt,
            ValFI, FI_filt,
            ThetaIzqActual, ThetaDerActual,
            DeltaSIzq, DeltaSDer,
            AvanceLineal, GiroAngular,
            VelocidadLineal, VelocidadAngular,
            TiempoActual
        ])

        guardar_fila_csv(RutaCSV_Kalman, [
            DistFD_cruda, DistFI_cruda, DistMin_cruda,
            DistFD_ema,   DistFI_ema,   DistMin_ema,
            d_pred,
            d_kalman,
            K_k,
            P_kalman,
            AvanceLineal,
            TiempoActual
        ])

    # ── 8. LÓGICA DE NAVEGACIÓN ──────────────────────────────────
    AmenazaDerecha   = max(ValFD, ValDD, ValLD, ValTD)
    AmenazaIzquierda = max(ValFI, ValDI, ValLI, ValTI)
    
    UmbralDetActual  = (UmbralDistDeteccion + UmbralDistHisteresis) \
                        if DireccionGiroBloqueada else UmbralDistDeteccion

    UmbralDiagActual = (UmbralDiagonal - Histeresis) \
                        if DireccionGiroBloqueada else UmbralDiagonal

    FrenteBloqueado = (d_kalman < UmbralDetActual or
                       ValDD > UmbralDiagActual   or
                       ValDI > UmbralDiagActual)
    PeligroCritico  = d_kalman < UmbralDistPeligro

    if EnModoRetroceso:
        VelocidadIzquierda = -VelocidadMaxima * 0.5
        VelocidadDerecha   = -VelocidadMaxima * 0.5
        PasosRetroceso += 1
        if PasosRetroceso >= LimiteRetroceso:
            EnModoRetroceso        = False
            PasosRetroceso         = 0
            AnguloGiradoAcum       = 0.0
            DireccionGiroBloqueada = None

    elif FrenteBloqueado or PeligroCritico:
        if DireccionGiroBloqueada is None:
            DireccionGiroBloqueada = 'DERECHA' if AmenazaIzquierda >= AmenazaDerecha else 'IZQUIERDA'
            AnguloGiradoAcum = 0.0

        AnguloGiradoAcum += abs(GiroAngular)
        ObjetivoActual    = AnguloObjetivoPeligro if PeligroCritico else AnguloObjetivoNormal

        if AnguloGiradoAcum >= ObjetivoActual:
            EnModoRetroceso = True
            PasosRetroceso  = 0
        else:
            MultAvance    =  0.7 if PeligroCritico else 0.6
            MultRetroceso = -0.7 if PeligroCritico else -0.4

            if DireccionGiroBloqueada == 'DERECHA':
                VelocidadIzquierda = VelocidadMaxima * MultAvance
                VelocidadDerecha   = VelocidadMaxima * MultRetroceso
            else:
                VelocidadIzquierda = VelocidadMaxima * MultRetroceso
                VelocidadDerecha   = VelocidadMaxima * MultAvance

    else:
        AnguloGiradoAcum       = 0.0
        DireccionGiroBloqueada = None

        ParedIzquierda = max(ValLI, ValDI)
        ParedDerecha   = max(ValLD, ValDD)
        ErrorPosicion  = ParedIzquierda - ParedDerecha

        if abs(ErrorPosicion) < 150:
            ErrorPosicion = 0

        AjusteDireccional  = ErrorPosicion * KpCentrado
        VelocidadBase      = VelocidadMaxima * 0.75

        VelocidadIzquierda = VelocidadBase + AjusteDireccional
        VelocidadDerecha   = VelocidadBase - AjusteDireccional

        VelocidadIzquierda = max(min(VelocidadIzquierda, VelocidadMaxima), -VelocidadMaxima)
        VelocidadDerecha   = max(min(VelocidadDerecha,   VelocidadMaxima), -VelocidadMaxima)

    MotorIzquierdo.setVelocity(VelocidadIzquierda)
    MotorDerecho.setVelocity(VelocidadDerecha)