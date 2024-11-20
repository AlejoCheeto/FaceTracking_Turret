import cv2 as cv
import time
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
            (x, y, w, h) = [int(v) for v in bbox] # Es válido el tracker si está dentro del campo de visión delimitada bbox
            cv.rectangle(frame, (x, y), (x + w, y + h), (200, 0, 210), 2) # Rositaaa siuuuu
            # Calcular la posición central del rostro y enviar coordenadas
            face_center_x, face_center_y = x + w // 2, y + h // 2
            print(f"Coordenada Central Rostro ahora: X={face_center_x}, Y={face_center_y}")
        else:
            # Si el tracker falla, reiniciamos el seguimiento - 
            tracking = False # Esto es si la cara se sale de la pantalla o ese campo de visión bbox
            print("Rostro perdido, buscando nuevamente...")
    # Mostrar la imagen con la detección
    cv.imshow('Face Tracking with Periodic Check', frame)
    # Salir con la tecla "Esc"
    if cv.waitKey(10) == 27:
        break
cap.release()
cv.destroyAllWindows()