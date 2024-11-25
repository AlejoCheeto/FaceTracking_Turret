import cv2
import sys
import time

if __name__ == '__main__':
    # Load Haar cascade for face detection
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Set up CSRT tracker
    tracker = cv2.legacy.TrackerCSRT_create()

    # Open the default camera (0)
    video = cv2.VideoCapture(0)

    # Exit if video not opened.
    if not video.isOpened():
        print("Could not open camera")
        sys.exit()

    # Initialize variables
    last_detection_time = time.time()  # Track the last detection time
    detection_interval = 5  # Time in seconds to wait between detections
    bbox = None  # Initialize bounding box

    while True:
        # Read a new frame
        ok, frame = video.read()
        if not ok:
            break

        # Start timer
        timer = cv2.getTickCount()

        # Convert the frame to grayscale for face detection
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect faces in the frame
        faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        # Update tracker if the bounding box is set
        if bbox is not None:
            ok, bbox = tracker.update(frame)

            # Check if tracking was successful
            if not ok:
                print("Tracking failure detected, trying to re-detect face.")
                bbox = None  # Reset the bounding box on tracking failure

        # Check if it's time to detect a face
        current_time = time.time()
        if current_time - last_detection_time > detection_interval or bbox is None:
            print("Detecting faces...")
            # If a face is detected, initialize the tracker with the last detected face
            if len(faces) > 0:
                print("Face detected")
                x, y, w, h = faces[-1]  # Track the last detected face
                bbox = (x, y, w, h)
                tracker.init(frame, bbox)
                last_detection_time = current_time  # Update the last detection time
            else:
                # No face detected, reset bbox
                print("No face detected!")

        # Draw bounding box for the tracked face
        if bbox is not None and ok:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)  # Draw tracker rectangle

        # Draw rectangles for detected faces
        if len(faces) > 0:
            for (fx, fy, fw, fh) in faces:
                cv2.rectangle(frame, (fx, fy), (fx + fw, fy + fh), (0, 255, 0), 2)  # Draw detected face rectangle

        # Display result
        cv2.imshow("Face Tracking", frame)

        # Exit if ESC pressed or 'q' key
        k = cv2.waitKey(1) & 0xff
        if k == 27 or k == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()