import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import time
from picamera2 import Picamera2

picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (640, 360)}
)

picam2.configure(config)
picam2.start()

time.sleep(1)

frame_counter = 0
start_time = time.time()

print("FPS test started. Press q to quit.")

while True:
    frame = picam2.capture_array()

    frame_counter += 1
    elapsed_time = time.time() - start_time

    if elapsed_time >= 1:
        fps = frame_counter / elapsed_time
        print(f"Camera FPS: {fps:.2f}")

        frame_counter = 0
        start_time = time.time()

    cv2.imshow("Pi Camera FPS Test", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

picam2.stop()
cv2.destroyAllWindows()
