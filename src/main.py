import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import time
import cv2
from ultralytics import YOLO

try:
    from .kalman import CentroidKalman
    KALMAN_AVAILABLE = True
except ImportError:
    KALMAN_AVAILABLE = False
    print("[kalman] filterpy not installed, Kalman disabled. pip install filterpy")

from . import config
from .camera import open_picamera, read_picamera_frame
from .detection import DetectionSmoother, detect, pick_center_target, pick_best_detection_for_reference
from .tracking import LockState, ControlMode, init_tracker_on_detection, reset_tracking_state
from .controller import PD, ServoUart, apply_auto_servo_control, handle_manual_key
from .display import draw
from .utils import center_of_bbox, quadrant, iou_xywh, dist2


def run():
    picam2 = open_picamera()
    model = YOLO(config.MODEL_PATH)
    uart = ServoUart(config.SERIAL_PORT, config.BAUD_RATE, enabled=config.ENABLE_UART)
    smoother = DetectionSmoother(ttl=config.SMOOTHER_TTL, match_dist=config.SMOOTHER_MATCH_DIST)

    if config.USE_KALMAN and KALMAN_AVAILABLE:
        kalman = CentroidKalman(dt=1.0 / config.TARGET_FPS)
        print(f"[kalman] active dt={1.0 / config.TARGET_FPS:.4f}s")
    else:
        kalman = None

    pd_x = PD(config.KP_X, config.KD_X)
    pd_y = PD(config.KP_Y, config.KD_Y)
    tracking = reset_tracking_state(smoother, pd_x, pd_y, kalman)

    mode = ControlMode.AUTO if config.AUTO_START_MODE.upper() == "AUTO" else ControlMode.MANUAL
    manual_speed = config.MANUAL_SPEED_START
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

            fx = config.FRAME_W // 2
            fy = config.FRAME_H // 2

            state = tracking["state"]
            tracker = tracking["tracker"]
            tracker_bbox = tracking["tracker_bbox"]
            tracker_label = tracking["tracker_label"]
            tracker_class_id = tracking["tracker_class_id"]
            tracker_conf = tracking["tracker_conf"]
            trusted_bbox = tracking["trusted_bbox"]
            trusted_class_id = tracking["trusted_class_id"]
            misses = tracking["misses"]
            detector_misses = tracking["detector_misses"]
            lost_hold = tracking["lost_hold"]

            detections = []
            fresh_detections = []
            cmd_x = cmd_y = 0.0
            q = "NONE"
            smooth_target = None

            if mode == ControlMode.AUTO:
                run_detect = (
                    state in (LockState.IDLE, LockState.LOST) or
                    (state == LockState.LOCKED and (frame_idx % config.DETECT_EVERY_N == 0 or misses > 0))
                )

                if run_detect:
                    yolo_start = time.time()
                    fresh_detections = detect(model, frame)
                    yolo_elapsed = time.time() - yolo_start
                    if yolo_elapsed > 0:
                        yolo_fps = 1.0 / yolo_elapsed
                    detections = smoother.update(fresh_detections)
                else:
                    detections = smoother.update([])

                if state == LockState.IDLE:
                    candidates = smoother.stable_only() or detections
                    target = pick_center_target(candidates, fx, fy)
                    if target is not None:
                        new_tracker, new_bbox = init_tracker_on_detection(frame, target)
                        if new_tracker is not None:
                            tracker, tracker_bbox = new_tracker, new_bbox
                            tracker_label, tracker_class_id = target["label"], target["class_id"]
                            tracker_conf = target["conf"]
                            trusted_bbox, trusted_class_id = new_bbox, tracker_class_id
                            misses = 0
                            detector_misses = 0
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

                    detector_confirmed = False

                    if fresh_detections:
                        ref_bbox = trusted_bbox if trusted_bbox is not None else tracker_bbox
                        ref_class_id = trusted_class_id if trusted_class_id is not None else tracker_class_id
                        best = pick_best_detection_for_reference(fresh_detections, ref_bbox, ref_class_id)

                        if best is not None:
                            detector_confirmed = True
                            best_bbox = tuple(map(int, best["bbox"]))
                            if tracker_bbox:
                                agree_iou = iou_xywh(best_bbox, tracker_bbox)
                                agree_d2 = dist2(center_of_bbox(best_bbox), center_of_bbox(tracker_bbox))
                            else:
                                agree_iou, agree_d2 = 0.0, 10 ** 9

                            trusted_bbox, trusted_class_id = best_bbox, best["class_id"]

                            if not ok_track or agree_iou < config.STRONG_MATCH_IOU or agree_d2 > config.STRONG_MATCH_DIST ** 2:
                                new_tracker, new_bbox = init_tracker_on_detection(frame, best)
                                if new_tracker is not None:
                                    tracker, tracker_bbox = new_tracker, new_bbox
                                    tracker_label, tracker_class_id = best["label"], best["class_id"]
                                    tracker_conf = best["conf"]
                                    misses = 0
                                    if kalman:
                                        kalman.reset()
                                    print(f"[snap] tracker to detector {tracker_label} bbox={tracker_bbox}")
                            else:
                                tracker_label, tracker_class_id = best["label"], best["class_id"]
                                tracker_conf = best["conf"]

                    if run_detect:
                        if detector_confirmed:
                            detector_misses = 0
                        else:
                            detector_misses += 1

                    if misses >= config.TRACKER_MAX_MISSES:
                        print("[LOCKED to LOST] tracker dropped, entering recovery hold")
                        tracker = tracker_bbox = None
                        lost_hold = config.LOST_HOLD_FRAMES
                        misses = 0
                        detector_misses = 0
                        state = LockState.LOST
                        uart.stop_all(force=True)
                        if kalman:
                            kalman.reset()
                        pd_x.reset()
                        pd_y.reset()

                    elif detector_misses >= config.DETECTOR_MAX_MISSES:
                        print("[LOCKED to LOST] detector no longer confirms target")
                        tracker = tracker_bbox = None
                        lost_hold = config.LOST_HOLD_FRAMES
                        misses = 0
                        detector_misses = 0
                        state = LockState.LOST
                        uart.stop_all(force=True)
                        if kalman:
                            kalman.reset()
                        pd_x.reset()
                        pd_y.reset()

                elif state == LockState.LOST:
                    target = None
                    if trusted_bbox is not None and fresh_detections:
                        target = pick_best_detection_for_reference(fresh_detections, trusted_bbox, trusted_class_id)

                    if target is not None:
                        new_tracker, new_bbox = init_tracker_on_detection(frame, target)
                        if new_tracker is not None:
                            tracker, tracker_bbox = new_tracker, new_bbox
                            tracker_label, tracker_class_id = target["label"], target["class_id"]
                            tracker_conf = target["conf"]
                            trusted_bbox, trusted_class_id = new_bbox, tracker_class_id
                            misses = 0
                            detector_misses = 0
                            state = LockState.LOCKED
                            if kalman:
                                kalman.reset()
                            print(f"[LOST to LOCKED] re-acquired: {tracker_label} bbox={tracker_bbox}")

                    if state == LockState.LOST:
                        lost_hold -= 1
                        if lost_hold <= 0:
                            print("[LOST to IDLE] recovery expired, full reset")
                            trusted_bbox = trusted_class_id = None
                            tracking = reset_tracking_state(smoother, pd_x, pd_y, kalman)
                            state = tracking["state"]
                            tracker = tracking["tracker"]
                            tracker_bbox = tracking["tracker_bbox"]
                            tracker_label = tracking["tracker_label"]
                            tracker_class_id = tracking["tracker_class_id"]
                            tracker_conf = tracking["tracker_conf"]
                            trusted_bbox = tracking["trusted_bbox"]
                            trusted_class_id = tracking["trusted_class_id"]
                            misses = tracking["misses"]
                            detector_misses = tracking["detector_misses"]
                            lost_hold = tracking["lost_hold"]
                            uart.stop_all(force=True)

                if state == LockState.LOCKED and tracker_bbox is not None:
                    raw_cx, raw_cy = center_of_bbox(tracker_bbox)
                    if kalman:
                        sx, sy = kalman.update(raw_cx, raw_cy)
                        cx, cy = int(sx), int(sy)
                        smooth_target = (cx, cy)
                    else:
                        cx, cy = raw_cx, raw_cy

                    cmd_x, cmd_y, ex, ey, centered = apply_auto_servo_control(uart, pd_x, pd_y, cx, cy, fx, fy)
                    q = quadrant(cx, cy, fx, fy, config.CENTER_BOX_W, config.CENTER_BOX_H)

                    if frame_idx % config.PRINT_EVERY_N == 0:
                        print(f"[AUTO/{state.name}] cmd_x={cmd_x:+.3f} cmd_y={cmd_y:+.3f} "
                              f"err_x={ex:+.3f} err_y={ey:+.3f} quadrant={q} centered={centered}")
                else:
                    uart.stop_all()

            else:  # MANUAL
                detections = []
                state = LockState.IDLE
                tracker = tracker_bbox = trusted_bbox = trusted_class_id = None
                tracker_label = ""
                tracker_conf = None
                tracker_class_id = misses = detector_misses = lost_hold = 0
                if time.time() > manual_active_until and not manual_sent_stop:
                    uart.stop_all()
                    manual_sent_stop = True

            tracking["state"] = state
            tracking["tracker"] = tracker
            tracking["tracker_bbox"] = tracker_bbox
            tracking["tracker_label"] = tracker_label
            tracking["tracker_class_id"] = tracker_class_id
            tracking["tracker_conf"] = tracker_conf
            tracking["trusted_bbox"] = trusted_bbox
            tracking["trusted_class_id"] = trusted_class_id
            tracking["misses"] = misses
            tracking["detector_misses"] = detector_misses
            tracking["lost_hold"] = lost_hold

            draw(frame, mode, state, tracker_bbox, tracker_label, tracker_conf, detections,
                 fx, fy, cmd_x, cmd_y, q, camera_fps, yolo_fps, manual_speed,
                 trusted_bbox=trusted_bbox, smooth_target=smooth_target)

            cv2.imshow("Pi Camera Auto Manual Tracker", frame)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord("q")):
                break

            if key == ord("m"):
                uart.stop_all(force=True)
                tracking = reset_tracking_state(smoother, pd_x, pd_y, kalman)
                manual_active_until = 0.0
                manual_sent_stop = True
                mode = ControlMode.MANUAL if mode == ControlMode.AUTO else ControlMode.AUTO
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
                    manual_active_until = time.time() + config.MANUAL_HOLD_SECONDS
                    manual_sent_stop = False

    finally:
        uart.close()
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    run()
