import cv2 as cv
import time
import numpy as np
from simple_pid import PID  # Necesitarás instalar esta librería: `pip install simple-pid`
import serial

# Cargar la cascada de rostros
face_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_frontalface_alt.xml')
# Inicializar la cámara
cap = cv.VideoCapture(0)
# Configurar variables de tracking
tracker = cv.legacy.TrackerMedianFlow_create()
tracking = False
first_face = None
# Temporizador para verificar rostros cada cierto tiempo
check_interval = 3  # segundos entre cada verificación de rostro
time_since_last_detection = time.time()

# Configura el puerto serial para comunicarte con los servos
ser = serial.Serial('COM7', 9600)  # Ajusta el puerto y la velocidad según tu configuración

# Define la resolución de la cámara y centro de imagen
frame_width, frame_height = int(cap.get(3)), int(cap.get(4))
center_x, center_y = frame_width // 2, frame_height // 2

""" center_x = 0
center_y = 0
 """

# Configuración de los controladores PID para los servos (uno para cada eje)
pid_x = PID(1, 0.1, 0, setpoint=center_x)
pid_y = PID(1, 0.1, 0, setpoint=center_y)


pid_x.output_limits = (0, 179)  # Limites de movimiento en X
pid_y.output_limits = (0, 179)  # Limites de movimiento en Y

def move_servos(dx, dy):
    # Mueve los servos en función de los valores dx y dy
    command = f"X{int(dx)}Y{int(dy)}\n"
    print(f"Enviando comando a servos: {command.strip()}")
    ser.write(command.encode())

while True:
    ret, frame = cap.read()
    if frame is None:
        print('--(!) No captured frame -- Break!')
        break

    # Convertir a escala de grises
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Comprobar si han pasado los segundos establecidos para verificar el rostro
    current_time = time.time()
    if current_time - time_since_last_detection > check_interval or not tracking:
        # Detectar rostros si ha pasado el tiempo o no estamos rastreando
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)
        if len(faces) > 0:
            # Seleccionar el primer rostro detectado
            (x, y, w, h) = faces[0]
            first_face = (x, y, w, h)
            tracker = cv.legacy.TrackerMedianFlow_create()  # Reiniciar el tracker para un nuevo rostro
            tracker.init(frame, first_face)
            tracking = True
            time_since_last_detection = current_time  # Reiniciar el temporizador

    # Si estamos siguiendo un rostro, actualizamos el tracker
    if tracking:
        success, bbox = tracker.update(frame)
        if success:
            # Si el tracker sigue el rostro con éxito, dibujamos el cuadro en la imagen
            (x, y, w, h) = [int(v) for v in bbox]
            # Calcular la posición central del rostro y enviar coordenadas
            face_center_x, face_center_y = x + w // 2, y + h // 2
            print(f"Coordenada Central Rostro ahora: X={face_center_x}, Y={face_center_y}")

            # Calcular el error para cada eje
            error_x = center_x - face_center_x
            error_y = center_y - face_center_y
            print(f"Errores: error_x = {error_x}, error_y = {error_y}")

            # Calcula la corrección PID
            correction_x = pid_x(error_x)
            correction_y = pid_y(error_y)
            print(f"Correcciones PID: correction_x = {correction_x}, correction_y = {correction_y}")

            # Mueve los servos
            move_servos(correction_x, correction_y)

            # Dibujar el cuadro y centro de la cara
            cv.rectangle(frame, (x, y), (x + w, y + h), (200, 0, 210), 2)
            cv.circle(frame, (face_center_x, face_center_y), 5, (0, 255, 0), -1)
        else:
            # Si el tracker falla, reiniciamos el seguimiento
            tracking = False
            print("Rostro perdido, buscando nuevamente...")

    # Mostrar la imagen con la detección
    cv.imshow('Face Tracking with Periodic Check', frame)

    # Salir con la tecla "Esc"
    if cv.waitKey(10) == 27:
        break

cap.release()
cv.destroyAllWindows()
