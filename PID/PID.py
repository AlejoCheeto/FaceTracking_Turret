import cv2
import numpy as np
from simple_pid import PID  # Necesitarás instalar esta librería: `pip install simple-pid`
import serial

# Configura el puerto serial para comunicarte con los servos
ser = serial.Serial('COM7', 9600)  # Ajusta el puerto y la velocidad según tu configuración

# Inicializa la cámara y el detector de rostros
cap = cv2.VideoCapture(0)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Define la resolución de la cámara y centro de imagen
frame_width, frame_height = int(cap.get(3)), int(cap.get(4))
center_x, center_y = frame_width // 2, frame_height // 2

# Configuración de los controladores PID para los servos (uno para cada eje)
pid_x = PID(0.1, 0.01, 0.05, setpoint=center_x)
pid_y = PID(0.1, 0.01, 0.05, setpoint=center_y)
pid_x.output_limits = (-30, 30)  # Limita el rango de movimiento
pid_y.output_limits = (-30, 30)

def move_servos(dx, dy):
    # Mueve los servos en función de los valores dx y dy
    command = f"X{int(dx)}Y{int(dy)}\n"
    ser.write(command.encode())

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 4)

    for (x, y, w, h) in faces:
        # Calcula el centro del rectángulo de detección
        face_center_x = x + w // 2
        face_center_y = y + h // 2

        # Calcular el error para cada eje
        error_x = center_x - face_center_x
        error_y = center_y - face_center_y

        # Calcula la corrección PID
        correction_x = pid_x(error_x)
        correction_y = pid_y(error_y)

        # Mueve los servos
        move_servos(correction_x, correction_y)

        # Dibuja el rectángulo y el centro en la imagen para visualización
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.circle(frame, (face_center_x, face_center_y), 5, (0, 255, 0), -1)
        break  # Solo procesa el primer rostro detectado

    # Muestra la imagen
    cv2.imshow('Face Tracking', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libera los recursos
cap.release()
cv2.destroyAllWindows()
ser.close()
