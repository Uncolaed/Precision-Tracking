import time
import cv2
from picamera2 import Picamera2
from . import config


def pick_full_fov_mode(picam2, target_fps):
    usable_modes = [m for m in picam2.sensor_modes if m["fps"] >= target_fps]
    if not usable_modes:
        print("[camera] no mode reaches target FPS, using the widest available mode instead")
        usable_modes = picam2.sensor_modes
    return max(usable_modes, key=lambda m: m["crop_limits"][2] * m["crop_limits"][3])


def open_picamera():
    picam2 = Picamera2()
    full_fov_mode = pick_full_fov_mode(picam2, config.TARGET_FPS)
    print("[camera] selected full-FOV sensor mode:")
    print(full_fov_mode)
    camera_config = picam2.create_video_configuration(
        main={"format": config.CAMERA_FORMAT, "size": (config.FRAME_W, config.FRAME_H)},
        raw={"size": full_fov_mode["size"]},
        controls={"FrameDurationLimits": (config.FRAME_TIME, config.FRAME_TIME)},
    )
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(1)
    return picam2


def read_picamera_frame(picam2):
    frame = picam2.capture_array()
    if frame is None:
        return None
    if config.CONVERT_RGB_TO_BGR:
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    if frame.shape[1] != config.FRAME_W or frame.shape[0] != config.FRAME_H:
        frame = cv2.resize(frame, (config.FRAME_W, config.FRAME_H))
    return frame
