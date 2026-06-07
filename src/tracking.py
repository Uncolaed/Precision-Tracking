import cv2
from enum import Enum, auto
from . import config


class LockState(Enum):
    IDLE = auto()
    LOCKED = auto()
    LOST = auto()


class ControlMode(Enum):
    AUTO = auto()
    MANUAL = auto()
    CALIBRATION = auto()


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
        raise RuntimeError(f"Tracker '{kind}' not available. Install opencv-contrib-python.")

    return ctor()


def init_tracker_on_detection(frame, det):
    tracker = make_tracker(config.TRACKER_TYPE)
    bbox = tuple(map(int, det["bbox"]))
    try:
        ok_init = tracker.init(frame, bbox)
    except cv2.error as e:
        print(f"[tracker] init failed: {e}")
        return None, None
    if ok_init is False:
        return None, None
    return tracker, bbox


def reset_tracking_state(smoother, controller_x, controller_y, kalman):
    smoother.reset()
    controller_x.reset()
    controller_y.reset()
    if kalman:
        kalman.reset()
    return {
        "state": LockState.IDLE,
        "tracker": None,
        "tracker_bbox": None,
        "tracker_label": "",
        "tracker_class_id": None,
        "tracker_conf": None,
        "trusted_bbox": None,
        "trusted_class_id": None,
        "misses": 0,
        "detector_misses": 0,
        "lost_hold": 0,
    }
