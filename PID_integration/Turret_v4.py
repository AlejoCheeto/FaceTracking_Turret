import cv2 as cv
import time
import numpy as np
import serial
from simple_pid import PID

# Configuración del controlador PID (ajustar según sea necesario)
pid_x = PID(0.5, 0.05, 0)
pid_y = PID(0.5, 0.05, 0)
pid_x.output_limits = (0, 180)  # Limitar salida para servomotor X (rango de 0° a 180°)
pid_y.output_limits = (0, 180)  # Limitar salida para servomotor Y

# Comunicación serial con Arduino
arduino = serial.Serial(port='COM4', baudrate=9600, timeout=1)
time.sleep(2)  # Esperar a que Arduino se inicialice

# Cargar la cascada de rostros
face_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_frontalface_alt.xml')

# Inicializar cámara
cap = cv.VideoCapture(0)

# Configurar variables de tracking (usar CSRT o KCF para mejor estabilidad)
tracker = cv.TrackerCSRT_create()
tracking = False

# Obtener dimensiones del frame para definir setpoint
ret, frame = cap.read()
if not ret:
    print("Error al acceder a la cámara")
    cap.release()
    exit()

frame_height, frame_width = frame.shape[:2]
setpoint_x = frame_width // 2  # Centro horizontal del frame (setpoint en X)
setpoint_y = frame_height // 2  # Centro vertical del frame (setpoint en Y)

# Actualizar setpoints en los controladores PID
pid_x.setpoint = 0  # El error deseado es cero (centro de la imagen)
pid_y.setpoint = 0

while True:
    ret, frame = cap.read()
    if frame is None:
        print('--(!) No captured frame -- Break!')
        break

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # Detección facial periódica o cuando no hay tracking activo
    if not tracking or time.time() - time_since_last_detection > check_interval:
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=4)
        if len(faces) > 0:
            (x, y, w, h) = faces[0]
            first_face = (x, y, w, h)
            tracker.init(frame, first_face)
            tracking = True

    if tracking:
        success, bbox = tracker.update(frame)
        if success:
            (x, y, w, h) = [int(v) for v in bbox]
            face_center_x, face_center_y = x + w // 2, y + h // 2

            # Calcular el error entre el centro del frame y el punto central del rostro
            error_x = setpoint_x - face_center_x
            error_y = setpoint_y - face_center_y

            # Obtener las salidas del PID basadas en el error
            control_x = pid_x(error_x)
            control_y = pid_y(error_y)

            # Enviar posiciones a Arduino por serial con retardo entre envíos
            data = f"{int(control_x)},{int(control_y)}\n"
            arduino.write(data.encode())
            time.sleep(0.05)

            # Dibujar el rectángulo que enmarca el rostro
            cv.rectangle(frame, (x, y), (x + w, y + h), (200, 0, 210), 2)  # Color rosado
            # Dibujar el punto central del rostro
            cv.circle(frame, (face_center_x, face_center_y), 5, (0, 255, 0), -1)  # Color verde
        else:
            tracking = False
            print("Rostro perdido, buscando nuevamente...")

    # Dibujar líneas guía para el centro de la imagen
    cv.line(frame, (setpoint_x, 0), (setpoint_x, frame_height), (255, 255, 255), 1)  # Línea vertical blanca
    cv.line(frame, (0, setpoint_y), (frame_width, setpoint_y), (255, 255, 255), 1)   # Línea horizontal blanca

    # Mostrar la imagen con el rectángulo y el punto central del rostro detectado
    cv.imshow('Face Tracking with PID', frame)

    if cv.waitKey(10) == 27: 
        break

cap.release()
cv.destroyAllWindows()
arduino.close()