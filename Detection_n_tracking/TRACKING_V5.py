import cv2
import sys
import time

if __name__ == '__main__':
    # Load Haar cascade for face detection
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    # Set up MOSSE tracker
    #tracker = cv2.legacy.TrackerMOSSE_create()
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

        # Check if it's time to detect a face
        current_time = time.time()
        if current_time - last_detection_time > detection_interval:
            print("Detecting faces...")
            # Convert the frame to grayscale for face detection
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect faces in the frame
            faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

            # If a face is detected, initialize the tracker with the first detected face
            if len(faces) > 0:
                print("Face detected")
                x, y, w, h = faces[0]
                bbox = (x, y, w, h)
                tracker.init(frame, bbox)
                last_detection_time = current_time  # Update the last detection time
            else:
                # No face detected, reset bbox
                bbox = None
                print("No face detected!")

        # Update tracker if the bounding box is set
        if bbox is not None:
            ok, bbox = tracker.update(frame)

            # Check if tracking was successful
            if not ok:
                # Tracking failure, attempt to detect a new face
                print("Tracking failure detected, trying to re-detect face.")
                gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

                # If a new face is detected, update the tracker
                if len(faces) > 0:
                    x, y, w, h = faces[0]
                    bbox = (x, y, w, h)
                    tracker.init(frame, bbox)
                    last_detection_time = current_time  # Update the last detection time
                else:
                    # No new face detected, reset bbox
                    bbox = None

            # Calculate Frames per second (FPS)
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)

            # Draw bounding box
            if ok:
                # Tracking success
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
            else:
                # Tracking failure message
                cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

            # Display FPS on frame
            cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)

        # Display result
        cv2.imshow("Face Tracking", frame)

        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27:
            break

    video.release()
    cv2.destroyAllWindows()