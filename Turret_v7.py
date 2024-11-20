import cv2
import sys
import time
import serial  # Para la comunicación con Arduino
from simple_pid import PID  # Para el control PID

if __name__ == '__main__':
    # Cargar la cascada Haar para la detección de rostros
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Crear el rastreador tipo CSRT
    tracker = cv2.legacy.TrackerCSRT_create()

    # Abrir la cámara por defecto (0)
    video = cv2.VideoCapture(0)

    # Salir si la cámara no se abre
    if not video.isOpened():
        print("No se pudo abrir la cámara")
        sys.exit()

    # Configuración del controlador PID (ajustar según sea necesario)
    pid_x = PID(1, 0.5, 0)
    pid_y = PID(1, 0.5, 0)

    pid_x.output_limits = (0, 180)  # Limitar salida para servomotor X (rango de 0° a 180°)
    pid_y.output_limits = (0, 180)  # Limitar salida para servomotor Y

    # Comunicación serial con Arduino
    arduino = serial.Serial(port='COM7', baudrate=9600, timeout=1)
    time.sleep(2)  # Esperar a que Arduino se inicialice

    # Obtener dimensiones del frame para definir setpoint (centro de la imagen)
    ret, frame = video.read()
    if not ret:
        print("Error al acceder a la cámara")
        video.release()
        sys.exit()

    frame_height, frame_width = frame.shape[:2]
    setpoint_x = frame_width // 2  # Centro horizontal del frame (setpoint en X)
    setpoint_y = frame_height // 2  # Centro vertical del frame (setpoint en Y)

    # Actualizar setpoints en los controladores PID
    pid_x.setpoint = 0   # El error deseado es cero (centro de la imagen)
    pid_y.setpoint = 0

    # Inicializar variables
    bbox = None  # Inicializar el cuadro delimitador
    last_detection_time = time.time()  # Tiempo de la última detección
    detection_interval = 8  # Intervalo de detección en segundos
    color_i = 0
    color = (255, 255, 255)

    while True:
        # Leer un nuevo cuadro
        ok, frame = video.read()
        if not ok:
            print("Error al leer el cuadro de video")
            break

        # Convertir el cuadro a escala de grises para la detección de rostros
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detectar rostros en el cuadro
        faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # Actualizar el rastreador si el cuadro delimitador está establecido
        if bbox is not None:
            ok, bbox = tracker.update(frame)

            # Si el rastreador falla, reiniciar el cuadro delimitador
            if not ok:
                print("Fallo en el rastreo, reiniciando rastreador.")
                bbox = None  # Reiniciar bbox si el rastreador falla

        # Comprobar si es hora de detectar un rostro
        current_time = time.time()
        if current_time - last_detection_time > detection_interval:
            print("Detectando rostros...")
            if len(faces) > 0:
                print("Nuevo rostro detectado, inicializando rastreador.")
                # Usar el primer rostro detectado
                x, y, w, h = faces[0]  # Rastrear el primer rostro detectado
                bbox = (x, y, w, h)
                tracker.init(frame, bbox)  # Inicializar el rastreador con el nuevo cuadro delimitador
                last_detection_time = current_time  # Actualizar el tiempo de la última detección
                color_i += 1
                color = (255, 0, 0) if color_i == 1 else (0, 255, 0) if color_i == 2 else (0, 0, 255) if color_i == 3 else (255, 255, 255)
                color_i = 0 if color_i > 3 else color_i
            else:
                print("¡No se detectó ningún rostro! Reiniciando cuadro delimitador.")
                bbox = None

        # Dibujar el cuadro delimitador para el rostro rastreado y aplicar control PID si hay un rostro detectado
        if bbox is not None and ok:
            print("Cuadro delimitador actualizado")
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (0, 0, 0), 2, 1) 

            face_center_x = int(bbox[0] + bbox[2] / 2)
            face_center_y = int(bbox[1] + bbox[3] / 2)

            # Calcular los errores entre el centro del frame y el punto central del rostro
            error_x = setpoint_x - face_center_x
            error_y = setpoint_y - face_center_y

            print(f"Error X: {error_x}, Error Y: {error_y}")

            # Obtener las salidas del PID basadas en los errores calculados
            control_x = pid_x(error_x)
            control_y = pid_y(error_y)

            print(f"Control X: {control_x}, Control Y: {control_y}")

            # Enviar posiciones a Arduino por serial con retardo entre envíos
            data = f"{int(control_x)},{int(control_y)}\n"
            arduino.write(data.encode())
            time.sleep(0.05)

        else:
            print("No hay un cuadro delimitador activo.")

        # Dibujar rectángulos para los rostros detectados pero no rastreados aún
        for (fx, fy, fw, fh) in faces:
            cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), color, 2)

        # Mostrar líneas guía en los ejes X e Y (centro de la imagen)
        cv2.line(frame,(setpoint_x ,0),(setpoint_x ,frame_height),(255 ,255 ,255),1 )   # Línea vertical blanca 
        cv2.line(frame,(0,setpoint_y),(frame_width,setpoint_y),(255 ,255 ,255),1 )       # Línea horizontal blanca
        
        cv2.imshow("Tracking", frame)

        # Salir si se presiona ESC o la tecla 'q'
        k = cv2.waitKey(1) & 0xff
        if k == 27 or k == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()
    arduino.close()