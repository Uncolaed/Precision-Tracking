import cv2
from . import config
from .tracking import LockState
from .utils import center_of_bbox

STATE_COLORS = {
    LockState.IDLE: (0, 165, 255),
    LockState.LOCKED: (0, 255, 0),
    LockState.LOST: (0, 80, 255),
}


def draw(frame, mode, state, tracker_bbox, target_label, target_conf, all_dets,
         fx, fy, cmd_x, cmd_y, q, camera_fps, yolo_fps,
         manual_speed, trusted_bbox=None, smooth_target=None):

    h, w = frame.shape[:2]
    state_color = STATE_COLORS[state]

    if config.SHOW_ALL_DETECTIONS:
        for d in all_dets:
            x, y, bw, bh = d["bbox"]
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), (120, 120, 120), 1)
            cv2.putText(frame, f"{d['label']} {d['conf']:.2f}", (x, max(16, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)

    cv2.line(frame, (fx, 0), (fx, h), (70, 70, 70), 1)
    cv2.line(frame, (0, fy), (w, fy), (70, 70, 70), 1)

    dbx1 = fx - config.CENTER_BOX_W // 2
    dby1 = fy - config.CENTER_BOX_H // 2
    dbx2 = fx + config.CENTER_BOX_W // 2
    dby2 = fy + config.CENTER_BOX_H // 2
    cv2.rectangle(frame, (dbx1, dby1), (dbx2, dby2), (0, 255, 255), 1)
    cv2.drawMarker(frame, (fx, fy), (255, 255, 255), cv2.MARKER_CROSS, 18, 1)

    if trusted_bbox is not None:
        x, y, bw, bh = trusted_bbox
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), (255, 255, 0), 1)

    if tracker_bbox is not None:
        x, y, bw, bh = tracker_bbox
        raw_cx, raw_cy = center_of_bbox(tracker_bbox)
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), state_color, 2)
        box_label = target_label
        if target_conf is not None:
            box_label = f"{target_label} {target_conf:.2f}"
        cv2.putText(frame, box_label, (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, state_color, 2)
        cv2.drawMarker(frame, (raw_cx, raw_cy), state_color, cv2.MARKER_CROSS, 16, 2)

        aim_cx, aim_cy = smooth_target if smooth_target else (raw_cx, raw_cy)
        cv2.line(frame, (fx, fy), (aim_cx, aim_cy), (255, 0, 255), 2)
        if smooth_target:
            cv2.drawMarker(frame, (aim_cx, aim_cy), (255, 0, 255), cv2.MARKER_CROSS, 12, 1)

        cv2.putText(frame, f"{box_label} | {q}", (10, 24),
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
