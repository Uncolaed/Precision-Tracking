import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import time
from picamera2 import Picamera2
from ultralytics import YOLO


TARGET_FPS = 30
FRAME_TIME = int(1_000_000 / TARGET_FPS)

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 360

YOLO_IMAGE_SIZE = 320
CONFIDENCE_THRESHOLD = 0.35

DETECT_EVERY = 2


model = YOLO("yolov8n.pt")

picam2 = Picamera2()

# I am using the same camera style as my camera test,
# but now I am running YOLO on the frames instead of only previewing them.
camera_config = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (CAMERA_WIDTH, CAMERA_HEIGHT)},
    controls={
        "FrameDurationLimits": (FRAME_TIME, FRAME_TIME)
    }
)

picam2.configure(camera_config)
picam2.start()

time.sleep(1)

print("Pi Camera object detection started. Press q to quit.")

frame_count = 0
last_annotated = None

while True:
    frame = picam2.capture_array()

    if frame is None:
        print("Failed to capture frame.")
        time.sleep(0.1)
        continue

    frame_count += 1

    if frame_count % DETECT_EVERY == 0:
        results = model.predict(
            frame,
            imgsz=YOLO_IMAGE_SIZE,
            conf=CONFIDENCE_THRESHOLD,
            verbose=False,
            device="cpu"
        )

        annotated = results[0].plot()
        last_annotated = annotated

    else:
        if last_annotated is not None:
            annotated = last_annotated
        else:
            annotated = frame

    cv2.imshow("Pi Camera Object Detection", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

picam2.stop()
cv2.destroyAllWindows()
