import time
from simple_pid import PID
import serial

# Configura el puerto serial para la comunicación con los servos
ser = serial.Serial('COM4', 9600)  # Ajusta el puerto y la velocidad según tu configuración

# Define la función para mover los servos
def move_servos(dx, dy):
    command = f"X{int(dx)}Y{int(dy)}\n"
    print(f"Enviando comando a servos: {command.strip()}")
    ser.write(command.encode())

# Configuración de los controladores PID para los servos (uno para cada eje)
pid_x = PID(2, 0, 0, setpoint=90)  # Setpoint inicial en el centro de 0-180
pid_y = PID(2, 0, 0, setpoint=90)

pid_x.output_limits = (0, 179)  # Limites de movimiento en X
pid_y.output_limits = (0, 179)  # Limites de movimiento en Y

# Función para simular el control PID
def test_pid(setpoint_x, setpoint_y, current_x, current_y):
    # Actualizar el setpoint de los PID
    pid_x.setpoint = setpoint_x
    pid_y.setpoint = setpoint_y

    # Calcular el error y la corrección PID
    error_x = setpoint_x - current_x
    error_y = setpoint_y - current_y
    correction_x = pid_x(error_x)
    correction_y = pid_y(error_y)

    # Imprimir las correcciones
    print(f"Error X: {error_x}, Corrección PID X: {correction_x}")
    print(f"Error Y: {error_y}, Corrección PID Y: {correction_y}")

    # Mover los servos
    move_servos(correction_x, correction_y)

# Función principal de ejecución interactiva
def run_pid_interactively():
    current_x = 0  # Valor inicial de X (puede ser cualquier valor)
    current_y = 0  # Valor inicial de Y (puede ser cualquier valor)

    print("Control PID interactivo iniciado.")
    print("Introduce los valores de setpoint para X y Y. Para salir, escribe 'salir'.")
    
    while True:
        # Obtener los valores de setpoint de usuario
        try:
            setpoint_x = input("Introduce el setpoint para X (0-180): ")
            if setpoint_x.lower() == "salir":
                print("Saliendo del control PID.")
                break
            setpoint_x = float(setpoint_x)
            
            setpoint_y = input("Introduce el setpoint para Y (0-180): ")
            if setpoint_y.lower() == "salir":
                print("Saliendo del control PID.")
                break
            setpoint_y = float(setpoint_y)

        except ValueError:
            print("Valor no válido, intenta nuevamente.")
            continue
        
        # Ejecutar PID con los valores de setpoint ingresados
        test_pid(setpoint_x, setpoint_y, current_x, current_y)

        # Simular que los motores se mueven (en la realidad, estos valores cambiarían con el tiempo)
        current_x += 1  # Simula el movimiento (esto debe ser ajustado según el sistema real)
        current_y += 1  # Simula el movimiento (esto debe ser ajustado según el sistema real)

        time.sleep(1)  # Espera 1 segundo antes de permitir un nuevo setpoint

# Iniciar la ejecución interactiva
run_pid_interactively()
