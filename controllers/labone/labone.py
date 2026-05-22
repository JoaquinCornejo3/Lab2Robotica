from controller import Robot

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

# Archivo de registro
archivo = open("registro_crudo.csv", "w")
archivo.write(
    "tiempo,"
    "frontal_der,frontal_izq,"
    "diagonal_der,diagonal_izq,"
    "lateral_der,lateral_izq,"
    "trasero_der,trasero_izq,"
    "encoder_izq,encoder_der\n"
)

while robot.step(timestep) != -1:
    t = robot.getTime()
    if t >= 30.0:
        break

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

    print(
        f"t={t:.2f} | "
        f"F_der={frontal_der:.1f} F_izq={frontal_izq:.1f} | "
        f"D_der={diagonal_der:.1f} D_izq={diagonal_izq:.1f} | "
        f"L_der={lateral_der:.1f} L_izq={lateral_izq:.1f} | "
        f"T_der={trasero_der:.1f} T_izq={trasero_izq:.1f} | "
        f"Enc_izq={enc_izq:.3f} Enc_der={enc_der:.3f}"
    )

    archivo.write(
        f"{t:.3f},"
        f"{frontal_der:.2f},{frontal_izq:.2f},"
        f"{diagonal_der:.2f},{diagonal_izq:.2f},"
        f"{lateral_der:.2f},{lateral_izq:.2f},"
        f"{trasero_der:.2f},{trasero_izq:.2f},"
        f"{enc_izq:.4f},{enc_der:.4f}\n"
    )

archivo.close()