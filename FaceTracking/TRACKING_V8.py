import cv2
import sys
import time

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
                bbox = None  # Reiniciar el cuadro delimitador si no se detecta ningún rostro

        # Dibujar el cuadro delimitador para el rostro rastreado
        if bbox is not None and ok:
            print("Cuadro delimitador actualizado")
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (0, 0, 0), 2, 1)  # Dibujar el rectángulo del rastreador

        # Dibujar rectángulos para los rostros detectados
        for (fx, fy, fw, fh) in faces:
            cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), color, 2)  # Dibujar el rectángulo del rostro detectado

        # Mostrar el resultado
        cv2.imshow("Tracking", frame)

        # Salir si se presiona ESC o la tecla 'q'
        k = cv2.waitKey(1) & 0xff
        if k == 27 or k == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()