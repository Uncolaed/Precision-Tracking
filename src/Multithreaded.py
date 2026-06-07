import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import time
import queue
import threading
import collections
import multiprocessing as mp
from pathlib import Path

import numpy as np
from picamera2 import Picamera2

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# ===========================================================================
# CONFIG
# I keep the important settings here so I can tune the test quickly.
# ===========================================================================

PROJECT_ROOT = Path(__file__).resolve().parent

if PROJECT_ROOT.name == "src":
    PROJECT_ROOT = PROJECT_ROOT.parent

MODEL_PATH = str(PROJECT_ROOT / "models" / "galactic_int8_openvino_model")

TARGET_FPS = 30
FRAME_TIME = int(1_000_000 / TARGET_FPS)

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 360
CAMERA_FORMAT = "RGB888"

CONFIDENCE_THRESHOLD = 0.40
INFER_EVERY_N = 2

SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 9600
ENABLE_UART = True

SERVO_SPEED = 30

CENTER_BOX_W = 80
CENTER_BOX_H = 60

BASE_LEFT_DIR = "ccw"
BASE_RIGHT_DIR = "cw"
ARM_UP_DIR = "cw"
ARM_DOWN_DIR = "ccw"

TARGET_TIMEOUT_SECONDS = 0.8
DEBUG_PRINT_EVERY_N = 10


# ===========================================================================
# PI CAMERA READER THREAD
# I use a separate thread so the main loop always gets the newest camera frame.
# ===========================================================================

class PiCameraReader:
    def __init__(self):
        self.picam2 = Picamera2()

        camera_config = self.picam2.create_preview_configuration(
            main={
                "format": CAMERA_FORMAT,
                "size": (CAMERA_WIDTH, CAMERA_HEIGHT),
            },
            controls={
                "FrameDurationLimits": (FRAME_TIME, FRAME_TIME),
            },
        )

        self.picam2.configure(camera_config)
        self.picam2.start()

        time.sleep(1)

        self.frame = None
        self.lock = threading.Lock()
        self.running = True

        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        while self.running:
            frame = self.picam2.capture_array()

            if frame is not None:
                with self.lock:
                    self.frame = frame
            else:
                time.sleep(0.005)

    def get_frame(self):
        with self.lock:
            if self.frame is None:
                return None

            return self.frame.copy()

    def release(self):
        self.running = False
        time.sleep(0.1)
        self.picam2.stop()


# ===========================================================================
# UART SERVO COMMAND SENDER
# I send the same commands that my STM32 UART code already understands.
# ===========================================================================

class ServoUart:
    def __init__(self, port, baud_rate, enabled=True):
        self.ser = None
        self.enabled = enabled

        self.last_command = {
            "base": None,
            "arm": None,
            "all": None,
        }

        if not self.enabled:
            print("[uart] disabled from config")
            return

        if not SERIAL_AVAILABLE:
            print("[uart] pyserial not installed, UART disabled")
            return

        try:
            self.ser = serial.Serial(port, baud_rate, timeout=1)
            time.sleep(2)
            print(f"[uart] opened {port} @ {baud_rate} baud")

        except serial.SerialException as e:
            print(f"[uart] could not open {port}: {e}")
            self.ser = None

    def send_command(self, command, channel="all"):
        if self.ser is None:
            return

        command = command.strip()

        if self.last_command.get(channel) == command:
            return

        self.ser.write((command + "\n").encode())
        self.last_command[channel] = command

        print("[uart] sent:", command)

    def send_axis(self, axis, direction, speed):
        command = f"{axis} {direction} {int(speed)}"
        self.send_command(command, channel=axis)

    def stop_axis(self, axis):
        self.send_command(f"{axis} stop", channel=axis)

    def stop_all(self):
        self.send_command("all stop", channel="all")
        self.last_command["base"] = "base stop"
        self.last_command["arm"] = "arm stop"

    def close(self):
        self.stop_all()

        if self.ser is not None:
            self.ser.close()
            print("[uart] closed")


# ===========================================================================
# TARGET SELECTION
# I pick the detected object closest to the frame center for this simple test.
# ===========================================================================

def get_center_target(result, frame_shape):
    boxes = result.boxes

    if boxes is None or len(boxes) == 0:
        return None

    h, w = frame_shape[:2]
    fx = w // 2
    fy = h // 2

    xyxy = boxes.xyxy.cpu().numpy()
    confs = boxes.conf.cpu().numpy()
    clss = boxes.cls.cpu().numpy().astype(int)

    best_target = None
    best_score = None

    for (x1, y1, x2, y2), conf, cid in zip(xyxy, confs, clss):
        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2

        dist_from_center = (cx - fx) ** 2 + (cy - fy) ** 2

        if best_score is None or dist_from_center < best_score:
            best_score = dist_from_center
            best_target = {
                "cx": cx,
                "cy": cy,
                "conf": float(conf),
                "class_id": int(cid),
            }

    return best_target


# ===========================================================================
# INFERENCE PROCESS
# I keep YOLO in a separate process so inference does not block the camera loop.
# I do NOT force imgsz here because my OpenVINO model has its own input size.
# ===========================================================================

def inference_process(in_queue: mp.Queue, out_queue: mp.Queue, model_path: str):
    from ultralytics import YOLO

    print("[inference] loading model:", model_path)

    model = YOLO(model_path, task="detect")

    # I warm up using the same camera frame size, but I do not pass imgsz.
    dummy = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)

    model.predict(
        dummy,
        conf=CONFIDENCE_THRESHOLD,
        verbose=False,
    )

    print("[inference] model ready")

    frame_counter = 0

    while True:
        frame = in_queue.get()

        if frame is None:
            break

        frame_counter += 1

        results = model.predict(
            frame,
            conf=CONFIDENCE_THRESHOLD,
            verbose=False,
        )

        result = results[0]
        annotated = result.plot()
        target = get_center_target(result, frame.shape)

        if result.boxes is None:
            box_count = 0
        else:
            box_count = len(result.boxes)

        if frame_counter % DEBUG_PRINT_EVERY_N == 0:
            print(f"[inference] boxes={box_count}, target={target}")

        while not out_queue.empty():
            try:
                out_queue.get_nowait()
            except queue.Empty:
                break

        out_queue.put((annotated, target, box_count))


# ===========================================================================
# SERVO DECISION LOGIC
# I convert target position into simple base and arm movement commands.
# ===========================================================================

def update_servo_from_target(uart, target):
    if target is None:
        uart.stop_all()
        return

    fx = CAMERA_WIDTH // 2
    fy = CAMERA_HEIGHT // 2

    cx = target["cx"]
    cy = target["cy"]

    x_dead = abs(cx - fx) <= CENTER_BOX_W // 2
    y_dead = abs(cy - fy) <= CENTER_BOX_H // 2

    if x_dead:
        uart.stop_axis("base")
    elif cx < fx:
        uart.send_axis("base", BASE_LEFT_DIR, SERVO_SPEED)
    else:
        uart.send_axis("base", BASE_RIGHT_DIR, SERVO_SPEED)

    if y_dead:
        uart.stop_axis("arm")
    elif cy < fy:
        uart.send_axis("arm", ARM_UP_DIR, SERVO_SPEED)
    else:
        uart.send_axis("arm", ARM_DOWN_DIR, SERVO_SPEED)


# ===========================================================================
# DISPLAY HELPERS
# ===========================================================================

def draw_target_info(frame, target, box_count):
    h, w = frame.shape[:2]

    fx = w // 2
    fy = h // 2

    dbx1 = fx - CENTER_BOX_W // 2
    dby1 = fy - CENTER_BOX_H // 2
    dbx2 = fx + CENTER_BOX_W // 2
    dby2 = fy + CENTER_BOX_H // 2

    cv2.rectangle(frame, (dbx1, dby1), (dbx2, dby2), (0, 255, 255), 1)
    cv2.drawMarker(frame, (fx, fy), (255, 255, 255), cv2.MARKER_CROSS, 18, 1)

    cv2.putText(
        frame,
        f"Boxes: {box_count}",
        (8, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
    )

    if target is None:
        cv2.putText(
            frame,
            "Target: NONE",
            (8, 82),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 80, 255),
            2,
        )
        return

    cx = target["cx"]
    cy = target["cy"]

    cv2.drawMarker(frame, (cx, cy), (255, 0, 255), cv2.MARKER_CROSS, 16, 2)
    cv2.line(frame, (fx, fy), (cx, cy), (255, 0, 255), 2)

    cv2.putText(
        frame,
        f"Target: ({cx}, {cy}) conf={target['conf']:.2f}",
        (8, 82),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
    )


# ===========================================================================
# MAIN LOOP
# ===========================================================================

def main():
    if not Path(MODEL_PATH).exists():
        print("[model] path does not exist:")
        print(MODEL_PATH)
        return

    print("[model] using:")
    print(MODEL_PATH)

    in_q = mp.Queue(maxsize=1)
    out_q = mp.Queue(maxsize=1)

    proc = mp.Process(
        target=inference_process,
        args=(in_q, out_q, MODEL_PATH),
        daemon=True,
    )

    proc.start()

    reader = PiCameraReader()
    uart = ServoUart(SERIAL_PORT, BAUD_RATE, enabled=ENABLE_UART)

    print("[main] Starting Pi Camera object detection with UART servo control.")
    print("[main] Press q to quit.")

    for _ in range(100):
        if reader.get_frame() is not None:
            break

        time.sleep(0.05)

    else:
        print("[camera] camera unavailable")
        uart.close()
        reader.release()
        in_q.put(None)
        proc.join(timeout=3)
        return

    fps_buf = collections.deque(maxlen=30)
    t_prev = time.perf_counter()

    last_out = None
    last_target = None
    last_target_time = 0.0
    last_box_count = 0
    frame_n = 0

    try:
        while True:
            frame = reader.get_frame()

            if frame is None:
                continue

            frame_n += 1

            if frame_n % INFER_EVERY_N == 0:
                if not in_q.full():
                    in_q.put_nowait(frame)

            try:
                last_out, last_target, last_box_count = out_q.get_nowait()
                last_target_time = time.time()
                update_servo_from_target(uart, last_target)

            except queue.Empty:
                if time.time() - last_target_time > TARGET_TIMEOUT_SECONDS:
                    last_target = None
                    last_box_count = 0
                    uart.stop_all()

            display = last_out if last_out is not None else frame.copy()

            draw_target_info(display, last_target, last_box_count)

            t_now = time.perf_counter()
            fps_buf.append(t_now - t_prev)
            t_prev = t_now

            fps = 1.0 / (sum(fps_buf) / len(fps_buf))

            cv2.putText(
                display,
                f"Display FPS: {fps:.1f}",
                (8, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

            cv2.imshow("Pi Camera Object Detection UART Control", display)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        uart.close()
        in_q.put(None)
        proc.join(timeout=3)
        reader.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)
    main()
