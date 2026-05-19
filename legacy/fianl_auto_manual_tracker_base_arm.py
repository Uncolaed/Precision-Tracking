import os
os.environ["QT_QPA_PLATFORM"] = "xcb" # this was used before for better perfromance i decided to stick to it 

import time
from enum import Enum, auto

import cv2
from picamera2 import Picamera2
from ultralytics import YOLO

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    from kalman import CentroidKalman
    KALMAN_AVAILABLE = True
except ImportError:
    KALMAN_AVAILABLE = False
    print("[kalman] filterpy not installed, Kalman disabled. pip install filterpy")


# ===========================================================================
# CONFIG
# I have kept the important tuning values here so I can change behavior quickly.
# ===========================================================================

TARGET_FPS = 30 #target fps for pi camera 
FRAME_TIME = int(1_000_000 / TARGET_FPS)

FRAME_W = 640
FRAME_H = 480
CAMERA_FORMAT = "RGB888"
CONVERT_RGB_TO_BGR = False

MODEL_PATH = r"/home/mizo/Downloads/Galactic-dev/galactic_int8_openvino_model"
CONF = 0.45
IMGSZ = 640

TRACKER_TYPE = "CSRT"          # CSRT is accurate, KCF is faster, MOSSE is fastest if available.
TRACKER_MAX_MISSES = 8
DETECT_EVERY_N = 2
LOST_HOLD_FRAMES = 30

CENTER_BOX_W = 80
CENTER_BOX_H = 60

KP_X = 0.50
KD_X = 0.10

KP_Y = 0.50
KD_Y = 0.10

MAX_CMD_X = 1.0
MAX_CMD_Y = 1.0

ENABLE_UART = True
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 9600

AUTO_START_MODE = "AUTO"       # AUTO or MANUAL

AUTO_MAX_SPEED = 60            # Highest speed percentage sent to STM32 during auto tracking.
AUTO_MIN_SPEED = 15            # Smallest movement speed when the target is outside the deadband.
MANUAL_SPEED_START = 30
MANUAL_SPEED_STEP = 5
MANUAL_SPEED_MIN = 5
MANUAL_SPEED_MAX = 100
SPEED_ROUND_STEP = 5

# Manual controls:
# A/D move the base left/right, W/S move the arm up/down.
# If the physical direction is wrong, switch only these words.
BASE_LEFT_DIR = "ccw"
BASE_RIGHT_DIR = "cw"
ARM_UP_DIR = "cw"
ARM_DOWN_DIR = "ccw"

# Auto controls:
# Positive X means the target is to the right of the screen center.
# Positive Y depends on USE_CARTESIAN_Y. With the default False, positive Y means target is below center.
# If the auto correction moves away from the target, switch these words.
AUTO_BASE_POSITIVE_X_DIR = "cw"
AUTO_ARM_POSITIVE_Y_DIR = "cw"

MANUAL_HOLD_SECONDS = 0.18     # Keeps a manual movement alive briefly between repeated key presses.
UART_REFRESH_SECONDS = 0.40    # Re-sends the same command sometimes so the STM32 stays updated.

REACQUIRE_MAX_DIST = 150
REACQUIRE_MIN_IOU = 0.02
STRONG_MATCH_IOU = 0.10
STRONG_MATCH_DIST = 90
PRINT_EVERY_N = 5

TARGET_CLASS = None            # Example: "drone", or None to accept all classes.
USE_CARTESIAN_Y = False
SHOW_ALL_DETECTIONS = True

SMOOTHER_TTL = 3
SMOOTHER_MATCH_DIST = 60

USE_KALMAN = True


# ===========================================================================
# DETECTION SMOOTHER
# I keep YOLO detections alive for a few frames so the target does not flicker.
# ===========================================================================

class DetectionSmoother:
    def __init__(self, ttl: int = 3, match_dist: int = 60):
        self._ttl = ttl
        self._match_dist2 = match_dist ** 2
        self._tracked = []

    def update(self, fresh: list) -> list:
        used = [False] * len(fresh)

        for tracked_item in self._tracked:
            best_idx = -1
            best_dist = self._match_dist2

            for i, fresh_det in enumerate(fresh):
                if used[i]:
                    continue

                if fresh_det["class_id"] != tracked_item["det"]["class_id"]:
                    continue

                dx = fresh_det["cx"] - tracked_item["det"]["cx"]
                dy = fresh_det["cy"] - tracked_item["det"]["cy"]
                dist2 = dx * dx + dy * dy

                if dist2 < best_dist:
                    best_dist = dist2
                    best_idx = i

            if best_idx >= 0:
                tracked_item["det"] = fresh[best_idx]
                tracked_item["ttl"] = self._ttl
                tracked_item["hits"] += 1
                used[best_idx] = True
            else:
                tracked_item["ttl"] -= 1

        self._tracked = [
            tracked_item
            for tracked_item in self._tracked
            if tracked_item["ttl"] > 0
        ]

        for i, fresh_det in enumerate(fresh):
            if not used[i]:
                self._tracked.append({
                    "det": fresh_det,
                    "ttl": self._ttl,
                    "hits": 1
                })

        self._tracked.sort(
            key=lambda tracked_item: (
                -tracked_item["hits"],
                -tracked_item["det"]["conf"]
            )
        )

        return [tracked_item["det"] for tracked_item in self._tracked]

    def stable_only(self) -> list:
        return [
            tracked_item["det"]
            for tracked_item in self._tracked
            if tracked_item["hits"] >= 2
        ]

    def reset(self):
        self._tracked.clear()


# ===========================================================================
# STATE MACHINE
# ===========================================================================

class LockState(Enum):
    IDLE = auto()
    LOCKED = auto()
    LOST = auto()


class ControlMode(Enum):
    AUTO = auto()
    MANUAL = auto()


# ===========================================================================
# SMALL HELPERS
# ===========================================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def opposite_dir(direction):
    direction = direction.lower()

    if direction == "cw":
        return "ccw"

    if direction == "ccw":
        return "cw"

    raise ValueError(f"Unsupported direction: {direction}")


def round_speed(speed, step):
    return int(round(speed / step) * step)


def iou_xywh(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b

    ax2, ay2 = ax + aw, ay + ah
    bx2, by2 = bx + bw, by + bh

    ix1, iy1 = max(ax, bx), max(ay, by)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)

    iw = max(0, ix2 - ix1)
    ih = max(0, iy2 - iy1)

    inter = iw * ih
    union = aw * ah + bw * bh - inter

    return inter / union if union > 0 else 0.0


def dist2(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return dx * dx + dy * dy


def center_of_bbox(b):
    x, y, w, h = b
    return int(x + w / 2), int(y + h / 2)


def in_deadband(cx, cy, fx, fy, bw, bh):
    return abs(cx - fx) <= bw // 2 and abs(cy - fy) <= bh // 2


def axis_deadbands(cx, cy, fx, fy):
    x_dead = abs(cx - fx) <= CENTER_BOX_W // 2
    y_dead = abs(cy - fy) <= CENTER_BOX_H // 2
    return x_dead, y_dead


def quadrant(cx, cy, fx, fy, bw, bh):
    if in_deadband(cx, cy, fx, fy, bw, bh):
        return "CENTER"

    if cx < fx and cy < fy:
        return "TOP-LEFT"

    if cx >= fx and cy < fy:
        return "TOP-RIGHT"

    if cx < fx and cy >= fy:
        return "BOTTOM-LEFT"

    return "BOTTOM-RIGHT"


def compute_errors(cx, cy, fx, fy):
    ex = (cx - fx) / (FRAME_W / 2.0)

    if USE_CARTESIAN_Y:
        ey = (fy - cy) / (FRAME_H / 2.0)
    else:
        ey = (cy - fy) / (FRAME_H / 2.0)

    return ex, ey


def command_from_pd(pd_value, positive_direction):
    pd_value = clamp(pd_value, -1.0, 1.0)

    if abs(pd_value) < 1e-3:
        return "stop", 0

    direction = positive_direction if pd_value > 0 else opposite_dir(positive_direction)

    speed = int(abs(pd_value) * AUTO_MAX_SPEED)
    speed = clamp(speed, AUTO_MIN_SPEED, AUTO_MAX_SPEED)
    speed = round_speed(speed, SPEED_ROUND_STEP)
    speed = clamp(speed, AUTO_MIN_SPEED, AUTO_MAX_SPEED)

    return direction, speed


# ===========================================================================
# PD CONTROLLER
# I use PD because continuous servos need speed correction, not position commands.
# ===========================================================================

class PD:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self._prev_error = None
        self._prev_t = None

    def reset(self):
        self._prev_error = None
        self._prev_t = None

    def update(self, error):
        now = time.time()

        if self._prev_t is None:
            dt = 0.0
        else:
            dt = max(now - self._prev_t, 1e-3)

        self._prev_t = now

        derivative = 0.0

        if self._prev_error is not None and dt > 0:
            derivative = (error - self._prev_error) / dt

        self._prev_error = error

        return self.kp * error + self.kd * derivative


# ===========================================================================
# UART COMMAND SENDER
# I send the exact format the STM32 already understands: base cw 30, arm stop, all stop.
# ===========================================================================

class ServoUart:
    def __init__(self, port, baud_rate, enabled=True):
        self._ser = None
        self._enabled = enabled
        self._last_command = {
            "base": None,
            "arm": None,
            "all": None
        }
        self._last_send_time = {
            "base": 0.0,
            "arm": 0.0,
            "all": 0.0
        }

        if not self._enabled:
            print("[uart] disabled from config")
            return

        if not SERIAL_AVAILABLE:
            print("[uart] pyserial not installed, UART disabled")
            return

        try:
            self._ser = serial.Serial(port, baud_rate, timeout=0)
            time.sleep(2)
            print(f"[uart] opened {port} @ {baud_rate} baud")

        except serial.SerialException as e:
            print(f"[uart] could not open {port}: {e}, UART disabled")
            self._ser = None

    def _write_line(self, line):
        if self._ser is None:
            return

        self._ser.write((line + "\n").encode())

    def send_axis(self, axis, direction, speed):
        axis = axis.lower()
        direction = direction.lower()
        speed = int(clamp(speed, 0, 100))

        if axis not in ("base", "arm"):
            raise ValueError(f"Unsupported servo axis: {axis}")

        if direction not in ("cw", "ccw"):
            raise ValueError(f"Unsupported servo direction: {direction}")

        command = f"{axis} {direction} {speed}"
        self._send_if_needed(axis, command)

    def stop_axis(self, axis):
        axis = axis.lower()

        if axis not in ("base", "arm"):
            raise ValueError(f"Unsupported servo axis: {axis}")

        command = f"{axis} stop"
        self._send_if_needed(axis, command)

    def stop_all(self, force=False):
        command = "all stop"

        if force:
            self._write_line(command)
            print(f"[uart] sent: {command}")
            self._last_command["base"] = "base stop"
            self._last_command["arm"] = "arm stop"
            self._last_command["all"] = command
            return

        self._send_if_needed("all", command)
        self._last_command["base"] = "base stop"
        self._last_command["arm"] = "arm stop"

    def _send_if_needed(self, channel, command):
        now = time.time()
        last_command = self._last_command.get(channel)
        last_time = self._last_send_time.get(channel, 0.0)

        should_send = (
            command != last_command or
            now - last_time >= UART_REFRESH_SECONDS
        )

        if not should_send:
            return

        self._write_line(command)
        print(f"[uart] sent: {command}")

        self._last_command[channel] = command
        self._last_send_time[channel] = now

    def close(self):
        self.stop_all(force=True)

        if self._ser is not None:
            self._ser.close()
            print("[uart] closed")


# ===========================================================================
# PI CAMERA SETUP
# This replaces cv2.VideoCapture with a direct Pi Camera frame source.
# ===========================================================================

def pick_full_fov_mode(picam2, target_fps):
    usable_modes = [
        mode
        for mode in picam2.sensor_modes
        if mode["fps"] >= target_fps
    ]

    if not usable_modes:
        print("[camera] no mode reaches target FPS, using the widest available mode instead")
        usable_modes = picam2.sensor_modes

    return max(
        usable_modes,
        key=lambda mode: mode["crop_limits"][2] * mode["crop_limits"][3]
    )


def open_picamera():
    picam2 = Picamera2()
    full_fov_mode = pick_full_fov_mode(picam2, TARGET_FPS)

    print("[camera] selected full-FOV sensor mode:")
    print(full_fov_mode)

    camera_config = picam2.create_video_configuration(
        main={"format": CAMERA_FORMAT, "size": (FRAME_W, FRAME_H)},
        raw={"size": full_fov_mode["size"]},
        controls={"FrameDurationLimits": (FRAME_TIME, FRAME_TIME)}
    )

    picam2.configure(camera_config)
    picam2.start()

    time.sleep(1)

    return picam2


def read_picamera_frame(picam2):
    frame = picam2.capture_array()

    if frame is None:
        return None

    if CONVERT_RGB_TO_BGR:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    if frame.shape[1] != FRAME_W or frame.shape[0] != FRAME_H:
        frame = cv2.resize(frame, (FRAME_W, FRAME_H))

    return frame


# ===========================================================================
# TRACKER FACTORY
# ===========================================================================

def make_tracker(kind="CSRT"):
    kind = kind.upper()
    legacy = getattr(cv2, "legacy", None)

    if kind == "CSRT":
        ctor = getattr(cv2, "TrackerCSRT_create", None)
        if ctor is None and legacy is not None:
            ctor = getattr(legacy, "TrackerCSRT_create", None)

    elif kind == "KCF":
        ctor = getattr(cv2, "TrackerKCF_create", None)
        if ctor is None and legacy is not None:
            ctor = getattr(legacy, "TrackerKCF_create", None)

    elif kind == "MOSSE":
        ctor = None
        if legacy is not None:
            ctor = getattr(legacy, "TrackerMOSSE_create", None)

    else:
        raise ValueError(f"Unsupported tracker type: {kind}")

    if ctor is None:
        raise RuntimeError(
            f"Tracker '{kind}' not available. Install opencv-contrib-python."
        )

    return ctor()


# ===========================================================================
# DETECTION
# ===========================================================================

def detect(model, frame):
    result = model.predict(
        frame,
        conf=CONF,
        imgsz=IMGSZ,
        verbose=False
    )[0]

    detections = []

    if result.boxes is None or len(result.boxes) == 0:
        return detections

    xyxy = result.boxes.xyxy.cpu().numpy()
    confs = result.boxes.conf.cpu().numpy()
    clss = result.boxes.cls.cpu().numpy().astype(int)

    for (x1, y1, x2, y2), conf, cid in zip(xyxy, confs, clss):
        label = model.names[int(cid)]

        if TARGET_CLASS and label != TARGET_CLASS:
            continue

        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

        w = x2 - x1
        h = y2 - y1
        cx = x1 + w // 2
        cy = y1 + h // 2

        detections.append({
            "bbox": (x1, y1, w, h),
            "cx": cx,
            "cy": cy,
            "conf": float(conf),
            "label": label,
            "class_id": int(cid),
        })

    return detections


def pick_center_target(dets, frame_cx, frame_cy):
    if not dets:
        return None

    return min(
        dets,
        key=lambda d: (d["cx"] - frame_cx) ** 2 + (d["cy"] - frame_cy) ** 2
    )


def pick_best_detection_for_reference(detections, ref_bbox, ref_class_id=None):
    if not detections or ref_bbox is None:
        return None

    rcx, rcy = center_of_bbox(ref_bbox)

    if ref_class_id is not None:
        same_class = [
            d
            for d in detections
            if d["class_id"] == ref_class_id
        ]
    else:
        same_class = []

    candidates = same_class if same_class else detections

    best = None
    best_score = -1e9

    for d in candidates:
        this_iou = iou_xywh(d["bbox"], ref_bbox)
        this_d2 = dist2((d["cx"], d["cy"]), (rcx, rcy))
        max_d2 = REACQUIRE_MAX_DIST * REACQUIRE_MAX_DIST

        if this_iou < REACQUIRE_MIN_IOU and this_d2 > max_d2:
            continue

        dist_score = 1.0 / (1.0 + this_d2 / float(max_d2))
        score = 2.8 * this_iou + 1.0 * dist_score + 0.25 * d["conf"]

        if score > best_score:
            best_score = score
            best = d

    return best


def init_tracker_on_detection(frame, det):
    tracker = make_tracker(TRACKER_TYPE)
    bbox = tuple(map(int, det["bbox"]))

    try:
        ok_init = tracker.init(frame, bbox)
    except cv2.error as e:
        print(f"[tracker] init failed: {e}")
        return None, None

    if ok_init is False:
        return None, None

    return tracker, bbox


# ===========================================================================
# AUTO SERVO CONTROL
# ===========================================================================

def apply_auto_servo_control(uart, pd_x, pd_y, cx, cy, fx, fy):
    ex, ey = compute_errors(cx, cy, fx, fy)
    x_dead, y_dead = axis_deadbands(cx, cy, fx, fy)

    cmd_x = 0.0
    cmd_y = 0.0

    if x_dead:
        pd_x.reset()
        uart.stop_axis("base")
    else:
        cmd_x = clamp(pd_x.update(ex), -MAX_CMD_X, MAX_CMD_X)
        base_dir, base_speed = command_from_pd(cmd_x, AUTO_BASE_POSITIVE_X_DIR)
        uart.send_axis("base", base_dir, base_speed)

    if y_dead:
        pd_y.reset()
        uart.stop_axis("arm")
    else:
        cmd_y = clamp(pd_y.update(ey), -MAX_CMD_Y, MAX_CMD_Y)
        arm_dir, arm_speed = command_from_pd(cmd_y, AUTO_ARM_POSITIVE_Y_DIR)
        uart.send_axis("arm", arm_dir, arm_speed)

    return cmd_x, cmd_y, ex, ey, x_dead and y_dead


# ===========================================================================
# MANUAL SERVO CONTROL
# ===========================================================================

def handle_manual_key(key, uart, manual_speed):
    if key == ord("a"):
        uart.send_axis("base", BASE_LEFT_DIR, manual_speed)
        return True, manual_speed

    if key == ord("d"):
        uart.send_axis("base", BASE_RIGHT_DIR, manual_speed)
        return True, manual_speed

    if key == ord("w"):
        uart.send_axis("arm", ARM_UP_DIR, manual_speed)
        return True, manual_speed

    if key == ord("s"):
        uart.send_axis("arm", ARM_DOWN_DIR, manual_speed)
        return True, manual_speed

    if key == ord(" "):
        uart.stop_all(force=True)
        return True, manual_speed

    if key in (ord("+"), ord("=")):
        manual_speed = clamp(manual_speed + MANUAL_SPEED_STEP, MANUAL_SPEED_MIN, MANUAL_SPEED_MAX)
        print(f"[manual] speed increased to {manual_speed}%")
        return False, manual_speed

    if key in (ord("-"), ord("_")):
        manual_speed = clamp(manual_speed - MANUAL_SPEED_STEP, MANUAL_SPEED_MIN, MANUAL_SPEED_MAX)
        print(f"[manual] speed decreased to {manual_speed}%")
        return False, manual_speed

    return False, manual_speed


# ===========================================================================
# DRAWING
# ===========================================================================

STATE_COLORS = {
    LockState.IDLE: (0, 165, 255),
    LockState.LOCKED: (0, 255, 0),
    LockState.LOST: (0, 80, 255),
}


def draw(frame, mode, state, tracker_bbox, target_label, all_dets,
         fx, fy, cmd_x, cmd_y, q, camera_fps, yolo_fps,
         manual_speed, trusted_bbox=None, smooth_target=None):

    h, w = frame.shape[:2]
    state_color = STATE_COLORS[state]

    if SHOW_ALL_DETECTIONS:
        for d in all_dets:
            x, y, bw, bh = d["bbox"]
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (120, 120, 120), 1)

    cv2.line(frame, (fx, 0), (fx, h), (70, 70, 70), 1)
    cv2.line(frame, (0, fy), (w, fy), (70, 70, 70), 1)

    dbx1 = fx - CENTER_BOX_W // 2
    dby1 = fy - CENTER_BOX_H // 2
    dbx2 = fx + CENTER_BOX_W // 2
    dby2 = fy + CENTER_BOX_H // 2

    cv2.rectangle(frame, (dbx1, dby1), (dbx2, dby2), (0, 255, 255), 1)
    cv2.drawMarker(frame, (fx, fy), (255, 255, 255), cv2.MARKER_CROSS, 18, 1)

    if trusted_bbox is not None:
        x, y, bw, bh = trusted_bbox
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), (255, 255, 0), 1)

    if tracker_bbox is not None:
        x, y, bw, bh = tracker_bbox
        raw_cx, raw_cy = center_of_bbox(tracker_bbox)

        cv2.rectangle(frame, (x, y), (x + bw, y + bh), state_color, 2)
        cv2.drawMarker(frame, (raw_cx, raw_cy), state_color, cv2.MARKER_CROSS, 16, 2)

        if smooth_target:
            aim_cx, aim_cy = smooth_target
        else:
            aim_cx, aim_cy = raw_cx, raw_cy

        cv2.line(frame, (fx, fy), (aim_cx, aim_cy), (255, 0, 255), 2)

        if smooth_target:
            cv2.drawMarker(frame, (aim_cx, aim_cy), (255, 0, 255), cv2.MARKER_CROSS, 12, 1)

        cv2.putText(frame, f"{target_label} | {q}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color, 2)

        cv2.putText(frame, f"cmd=({cmd_x:+.3f}, {cmd_y:+.3f})", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)

    else:
        cv2.putText(frame, f"State: {state.name}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color, 2)

    cv2.putText(frame, f"Mode: {mode.name} | m switch | r reset | q quit", (10, h - 92),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

    cv2.putText(frame, f"Manual: WASD move | Space stop | +/- speed {manual_speed}%", (10, h - 68),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

    cv2.putText(frame, f"Cam FPS: {camera_fps:.1f} | YOLO FPS: {yolo_fps:.1f}", (10, h - 44),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

    cv2.putText(frame, f"{mode.name}/{state.name}", (w - 170, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, state_color, 2)


# ===========================================================================
# RESET HELPERS
# ===========================================================================

def reset_tracking_state(smoother, pd_x, pd_y, kalman):
    smoother.reset()
    pd_x.reset()
    pd_y.reset()

    if kalman:
        kalman.reset()

    return {
        "state": LockState.IDLE,
        "tracker": None,
        "tracker_bbox": None,
        "tracker_label": "",
        "tracker_class_id": None,
        "trusted_bbox": None,
        "trusted_class_id": None,
        "misses": 0,
        "lost_hold": 0
    }


# ===========================================================================
# MAIN LOOP
# ===========================================================================

def main():
    picam2 = open_picamera()

    model = YOLO(MODEL_PATH)
    uart = ServoUart(SERIAL_PORT, BAUD_RATE, enabled=ENABLE_UART)

    smoother = DetectionSmoother(
        ttl=SMOOTHER_TTL,
        match_dist=SMOOTHER_MATCH_DIST
    )

    if USE_KALMAN and KALMAN_AVAILABLE:
        kalman = CentroidKalman(dt=1.0 / TARGET_FPS)
        print(f"[kalman] active dt={1.0 / TARGET_FPS:.4f}s")
    else:
        kalman = None

    pd_x = PD(KP_X, KD_X)
    pd_y = PD(KP_Y, KD_Y)

    tracking = reset_tracking_state(smoother, pd_x, pd_y, kalman)

    mode = ControlMode.AUTO if AUTO_START_MODE.upper() == "AUTO" else ControlMode.MANUAL
    manual_speed = MANUAL_SPEED_START
    manual_active_until = 0.0
    manual_sent_stop = True

    frame_idx = 0
    camera_fps = 0.0
    yolo_fps = 0.0
    fps_counter = 0
    fps_timer = time.time()

    print("[main] Pi Camera tracker with AUTO/MANUAL control started.")
    print("[keys] m=switch mode, r=reset, q/ESC=quit")
    print("[manual] WASD move, SPACE stop, +/- speed")

    try:
        while True:
            frame = read_picamera_frame(picam2)

            if frame is None:
                print("[camera] failed to capture frame")
                time.sleep(0.1)
                continue

            frame_idx += 1
            fps_counter += 1

            now = time.time()
            elapsed = now - fps_timer

            if elapsed >= 1:
                camera_fps = fps_counter / elapsed
                fps_counter = 0
                fps_timer = now

            fx = FRAME_W // 2
            fy = FRAME_H // 2

            state = tracking["state"]
            tracker = tracking["tracker"]
            tracker_bbox = tracking["tracker_bbox"]
            tracker_label = tracking["tracker_label"]
            tracker_class_id = tracking["tracker_class_id"]
            trusted_bbox = tracking["trusted_bbox"]
            trusted_class_id = tracking["trusted_class_id"]
            misses = tracking["misses"]
            lost_hold = tracking["lost_hold"]

            detections = []
            cmd_x = 0.0
            cmd_y = 0.0
            q = "NONE"
            smooth_target = None

            if mode == ControlMode.AUTO:
                run_detect = (
                    state in (LockState.IDLE, LockState.LOST) or
                    (
                        state == LockState.LOCKED and
                        (frame_idx % DETECT_EVERY_N == 0 or misses > 0)
                    )
                )

                if run_detect:
                    yolo_start = time.time()
                    raw_dets = detect(model, frame)
                    yolo_elapsed = time.time() - yolo_start

                    if yolo_elapsed > 0:
                        yolo_fps = 1.0 / yolo_elapsed

                    detections = smoother.update(raw_dets)

                else:
                    detections = smoother.update([])

                if state == LockState.IDLE:
                    candidates = smoother.stable_only() or detections
                    target = pick_center_target(candidates, fx, fy)

                    if target is not None:
                        new_tracker, new_bbox = init_tracker_on_detection(frame, target)

                        if new_tracker is not None:
                            tracker = new_tracker
                            tracker_bbox = new_bbox
                            tracker_label = target["label"]
                            tracker_class_id = target["class_id"]
                            trusted_bbox = new_bbox
                            trusted_class_id = tracker_class_id
                            misses = 0
                            state = LockState.LOCKED

                            if kalman:
                                kalman.reset()

                            print(f"[IDLE to LOCKED] acquired: {tracker_label} bbox={tracker_bbox}")

                elif state == LockState.LOCKED:
                    ok_track, bbox = tracker.update(frame)

                    if ok_track:
                        tracker_bbox = tuple(map(int, bbox))
                        misses = 0
                    else:
                        misses += 1

                    if detections:
                        ref_bbox = trusted_bbox if trusted_bbox is not None else tracker_bbox
                        ref_class_id = trusted_class_id if trusted_class_id is not None else tracker_class_id

                        best = pick_best_detection_for_reference(
                            detections,
                            ref_bbox,
                            ref_class_id
                        )

                        if best is not None:
                            best_bbox = tuple(map(int, best["bbox"]))

                            if tracker_bbox:
                                agree_iou = iou_xywh(best_bbox, tracker_bbox)
                                agree_d2 = dist2(
                                    center_of_bbox(best_bbox),
                                    center_of_bbox(tracker_bbox)
                                )
                            else:
                                agree_iou = 0.0
                                agree_d2 = 10 ** 9

                            trusted_bbox = best_bbox
                            trusted_class_id = best["class_id"]

                            if (
                                not ok_track or
                                agree_iou < STRONG_MATCH_IOU or
                                agree_d2 > STRONG_MATCH_DIST ** 2
                            ):
                                new_tracker, new_bbox = init_tracker_on_detection(frame, best)

                                if new_tracker is not None:
                                    tracker = new_tracker
                                    tracker_bbox = new_bbox
                                    tracker_label = best["label"]
                                    tracker_class_id = best["class_id"]
                                    misses = 0

                                    if kalman:
                                        kalman.reset()

                                    print(f"[snap] tracker to detector {tracker_label} bbox={tracker_bbox}")

                            else:
                                tracker_label = best["label"]
                                tracker_class_id = best["class_id"]

                    if misses >= TRACKER_MAX_MISSES:
                        print("[LOCKED to LOST] tracker dropped, entering recovery hold")
                        tracker = None
                        tracker_bbox = None
                        lost_hold = LOST_HOLD_FRAMES
                        misses = 0
                        state = LockState.LOST
                        uart.stop_all(force=True)

                        if kalman:
                            kalman.reset()

                        pd_x.reset()
                        pd_y.reset()

                elif state == LockState.LOST:
                    target = None

                    if trusted_bbox is not None and detections:
                        target = pick_best_detection_for_reference(
                            detections,
                            trusted_bbox,
                            trusted_class_id
                        )

                    if target is not None:
                        new_tracker, new_bbox = init_tracker_on_detection(frame, target)

                        if new_tracker is not None:
                            tracker = new_tracker
                            tracker_bbox = new_bbox
                            tracker_label = target["label"]
                            tracker_class_id = target["class_id"]
                            trusted_bbox = new_bbox
                            trusted_class_id = tracker_class_id
                            misses = 0
                            state = LockState.LOCKED

                            if kalman:
                                kalman.reset()

                            print(f"[LOST to LOCKED] re-acquired: {tracker_label} bbox={tracker_bbox}")

                    if state == LockState.LOST:
                        lost_hold -= 1

                        if lost_hold <= 0:
                            print("[LOST to IDLE] recovery expired, full reset")
                            trusted_bbox = None
                            trusted_class_id = None
                            tracking = reset_tracking_state(smoother, pd_x, pd_y, kalman)
                            state = tracking["state"]
                            tracker = tracking["tracker"]
                            tracker_bbox = tracking["tracker_bbox"]
                            tracker_label = tracking["tracker_label"]
                            tracker_class_id = tracking["tracker_class_id"]
                            trusted_bbox = tracking["trusted_bbox"]
                            trusted_class_id = tracking["trusted_class_id"]
                            misses = tracking["misses"]
                            lost_hold = tracking["lost_hold"]
                            uart.stop_all(force=True)

                if state == LockState.LOCKED and tracker_bbox is not None:
                    raw_cx, raw_cy = center_of_bbox(tracker_bbox)

                    if kalman:
                        sx, sy = kalman.update(raw_cx, raw_cy)
                        cx = int(sx)
                        cy = int(sy)
                        smooth_target = (cx, cy)
                    else:
                        cx = raw_cx
                        cy = raw_cy

                    cmd_x, cmd_y, ex, ey, centered = apply_auto_servo_control(
                        uart,
                        pd_x,
                        pd_y,
                        cx,
                        cy,
                        fx,
                        fy
                    )

                    q = quadrant(cx, cy, fx, fy, CENTER_BOX_W, CENTER_BOX_H)

                    if frame_idx % PRINT_EVERY_N == 0:
                        print(
                            f"[AUTO/{state.name}] "
                            f"cmd_x={cmd_x:+.3f} cmd_y={cmd_y:+.3f} "
                            f"err_x={ex:+.3f} err_y={ey:+.3f} "
                            f"quadrant={q} centered={centered}"
                        )
                else:
                    uart.stop_all()

            else:
                detections = []
                state = LockState.IDLE
                tracker = None
                tracker_bbox = None
                tracker_label = ""
                trusted_bbox = None
                trusted_class_id = None
                misses = 0
                lost_hold = 0

                if time.time() > manual_active_until and not manual_sent_stop:
                    uart.stop_all()
                    manual_sent_stop = True

            tracking["state"] = state
            tracking["tracker"] = tracker
            tracking["tracker_bbox"] = tracker_bbox
            tracking["tracker_label"] = tracker_label
            tracking["tracker_class_id"] = tracker_class_id
            tracking["trusted_bbox"] = trusted_bbox
            tracking["trusted_class_id"] = trusted_class_id
            tracking["misses"] = misses
            tracking["lost_hold"] = lost_hold

            draw(
                frame,
                mode,
                state,
                tracker_bbox,
                tracker_label,
                detections,
                fx,
                fy,
                cmd_x,
                cmd_y,
                q,
                camera_fps,
                yolo_fps,
                manual_speed,
                trusted_bbox=trusted_bbox,
                smooth_target=smooth_target,
            )

            cv2.imshow("Pi Camera Auto Manual Tracker", frame)

            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord("q")):
                break

            if key == ord("m"):
                uart.stop_all(force=True)
                tracking = reset_tracking_state(smoother, pd_x, pd_y, kalman)
                manual_active_until = 0.0
                manual_sent_stop = True

                if mode == ControlMode.AUTO:
                    mode = ControlMode.MANUAL
                else:
                    mode = ControlMode.AUTO

                print(f"[mode] switched to {mode.name}")

            elif key == ord("r"):
                print("[manual] reset requested")
                uart.stop_all(force=True)
                tracking = reset_tracking_state(smoother, pd_x, pd_y, kalman)
                manual_active_until = 0.0
                manual_sent_stop = True

            elif mode == ControlMode.MANUAL and key != 255:
                moved, manual_speed = handle_manual_key(key, uart, manual_speed)

                if moved:
                    manual_active_until = time.time() + MANUAL_HOLD_SECONDS
                    manual_sent_stop = False

    finally:
        uart.close()
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
