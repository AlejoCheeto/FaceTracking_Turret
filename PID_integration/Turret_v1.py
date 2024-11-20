import cv2 as cv
import time
import serial
from simple_pid import PID

# Configuración del puerto serial para comunicar con Arduino
ser = serial.Serial('COM4', 9600)  # Cambia el puerto a COMx en Windows o ttyUSBx en Linux

# Cargar la cascada de rostros
face_cascade = cv.CascadeClassifier(cv.data.haarcascades + 'haarcascade_frontalface_alt.xml')

# Inicializar la cámara
cap = cv.VideoCapture(1)

# Configurar variables de tracking
tracker = cv.legacy.TrackerMedianFlow_create()
tracking = False
first_face = None

# Configurar temporizador para verificar rostros cada cierto tiempo
check_interval = 3  # segundos entre cada verificación de rostro
time_since_last_detection = time.time()

# Definir la resolución de la cámara y centro de imagen
frame_width, frame_height = int(cap.get(3)), int(cap.get(4))
center_x, center_y = frame_width // 2, frame_height // 2

# Inicializar controladores PID para el seguimiento en X y Y
pid_x = PID(10, 0, 0, setpoint=center_x)
pid_y = PID(10, 0, 0, setpoint=center_y)
pid_x.output_limits = (-90, 90)  # Limites de movimiento en X
pid_y.output_limits = (-90, 90)  # Limites de movimiento en Y

# Función para enviar los ángulos de corrección al Arduino
def move_servos(angle_x, angle_y):
    command = f"X{int(angle_x)}Y{int(angle_y)}\n"
    ser.write(command.encode())

while True:
    ret, frame = cap.read()
    if frame is None:
        print('--(!) No captured frame -- Break!')
        break

    # Convertir a escala de grises para detección de rostros
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    current_time = time.time()
    
    # Verificar si han pasado los segundos para una nueva detección o si el rastreo falló
    if current_time - time_since_last_detection > check_interval or not tracking:
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)
        if len(faces) > 0:
            # Seleccionar el primer rostro detectado
            (x, y, w, h) = faces[0]
            first_face = (x, y, w, h)
            tracker = cv.legacy.TrackerMedianFlow_create()  # Reiniciar el tracker para el nuevo rostro
            tracker.init(frame, first_face)
            tracking = True
            time_since_last_detection = current_time

    # Si estamos siguiendo un rostro, actualizar el tracker
    if tracking:
        success, bbox = tracker.update(frame)
        if success:
            # Si el tracker sigue el rostro con éxito, obtener coordenadas del centro del rostro
            (x, y, w, h) = [int(v) for v in bbox]
            face_center_x, face_center_y = x + w // 2, y + h // 2

            # Calcular la corrección PID para centrar el rostro
            correction_x = pid_x(center_x - face_center_x)
            correction_y = pid_y(center_y - face_center_y)

            # Mover los servos con la corrección calculada
            move_servos(correction_x, correction_y)

            # Dibujar el cuadro en la imagen
            cv.rectangle(frame, (x, y), (x + w, y + h), (200, 0, 210), 2)
            print(f"Coordenada Central Rostro ahora: X={face_center_x}, Y={face_center_y}")
        else:
            tracking = False  # Reiniciar si el tracker falla
            print("Rostro perdido, buscando nuevamente...")

    # Mostrar la imagen con el rastreo
    cv.imshow('Face Tracking with PID', frame)
    if cv.waitKey(10) == 27:  # Salir con la tecla "Esc"
        break

# Liberar recursos
cap.release()
cv.destroyAllWindows()
ser.close()
