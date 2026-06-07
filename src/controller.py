import re
import time

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

from . import config
from .utils import clamp, opposite_dir, round_speed


ARM_X_RE = re.compile(r"\bArm_X:([-+]?\d+(?:\.\d+)?)")


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
        dt = 0.0 if self._prev_t is None else max(now - self._prev_t, 1e-3)
        self._prev_t = now
        derivative = (error - self._prev_error) / dt if self._prev_error is not None and dt > 0 else 0.0
        self._prev_error = error
        return self.kp * error + self.kd * derivative


class ServoUart:
    def __init__(self, port, baud_rate, enabled=True):
        self._ser = None
        self._enabled = enabled
        self._last_command = {"base": None, "arm": None, "all": None}
        self._last_send_time = {"base": 0.0, "arm": 0.0, "all": 0.0}

        if not self._enabled:
            print("[uart] disabled from config")
            return

        if not SERIAL_AVAILABLE:
            print("[uart] pyserial not installed, UART disabled")
            return

        try:
            self._ser = serial.Serial(port, baud_rate, timeout=0.05, write_timeout=1)
            time.sleep(2)
            print(f"[uart] opened {port} @ {baud_rate} baud")
        except serial.SerialException as e:
            print(f"[uart] could not open {port}: {e}, UART disabled")
            self._ser = None

    def _write_line(self, line):
        if self._ser is None:
            return
        self._ser.write((line + "\n").encode())

    def send_raw(self, command):
        command = command.strip()
        if not command:
            return
        self._write_line(command)
        print(f"[uart] sent raw: {command}")

    def read_lines(self, duration_seconds=0.5):
        if self._ser is None:
            return []

        deadline = time.time() + max(duration_seconds, 0.0)
        lines = []
        while time.time() < deadline:
            raw = self._ser.readline()
            if raw:
                lines.append(raw.decode(errors="replace").strip())
            else:
                time.sleep(0.01)
        return [line for line in lines if line]

    def send_axis(self, axis, direction, speed):
        axis = axis.lower()
        direction = direction.lower()
        speed = int(clamp(speed, 0, 100))
        if axis not in ("base", "arm"):
            raise ValueError(f"Unsupported servo axis: {axis}")
        if direction not in ("cw", "ccw"):
            raise ValueError(f"Unsupported servo direction: {direction}")
        self._send_if_needed(axis, f"{axis} {direction} {speed}")

    def stop_axis(self, axis):
        axis = axis.lower()
        if axis not in ("base", "arm"):
            raise ValueError(f"Unsupported servo axis: {axis}")
        self._send_if_needed(axis, f"{axis} stop")

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
        stale = now - self._last_send_time.get(channel, 0.0) >= config.UART_REFRESH_SECONDS
        if command == self._last_command.get(channel) and not stale:
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


def _read_and_print_uart(uart, duration_seconds=None):
    duration = config.MPU_CALIBRATION_READ_SECONDS if duration_seconds is None else duration_seconds
    lines = uart.read_lines(duration)
    if lines:
        for line in lines:
            print(f"[stm32] {line}")
    else:
        print("[stm32] no response")
    return lines


def _send_raw_and_print(uart, command, duration_seconds=None):
    uart.send_raw(command)
    return _read_and_print_uart(uart, duration_seconds)


def _parse_arm_x(lines):
    for line in reversed(lines):
        match = ARM_X_RE.search(line)
        if match:
            return float(match.group(1))
    return None


def configure_mpu_limits(uart, offset=None, min_angle=None, max_angle=None):
    offset = config.MPU_ARM_X_OFFSET if offset is None else offset
    min_angle = config.MPU_ARM_X_MIN if min_angle is None else min_angle
    max_angle = config.MPU_ARM_X_MAX if max_angle is None else max_angle

    print(f"[mpu] applying limits offset={offset} range={min_angle}..{max_angle}")
    lines = []
    lines.extend(_send_raw_and_print(uart, f"limit offset {offset}"))
    lines.extend(_send_raw_and_print(uart, f"limit range {min_angle} {max_angle}"))
    lines.extend(show_mpu_sample(uart))
    return lines


def show_mpu_sample(uart):
    return _send_raw_and_print(uart, "mpu show")


def zero_mpu_arm_x(uart):
    return _send_raw_and_print(uart, "limit zero")


def set_mpu_min_from_current(uart):
    lines = show_mpu_sample(uart)
    arm_x = _parse_arm_x(lines)
    if arm_x is None:
        print("[mpu] could not parse Arm_X; min was not changed")
        return lines
    print(f"[mpu] setting current Arm_X as min: {arm_x:.1f}")
    lines.extend(_send_raw_and_print(uart, f"limit min {arm_x:.1f}"))
    return lines


def set_mpu_max_from_current(uart):
    lines = show_mpu_sample(uart)
    arm_x = _parse_arm_x(lines)
    if arm_x is None:
        print("[mpu] could not parse Arm_X; max was not changed")
        return lines
    print(f"[mpu] setting current Arm_X as max: {arm_x:.1f}")
    lines.extend(_send_raw_and_print(uart, f"limit max {arm_x:.1f}"))
    return lines


def axis_deadbands(cx, cy, fx, fy):
    return (
        abs(cx - fx) <= config.CENTER_BOX_W // 2,
        abs(cy - fy) <= config.CENTER_BOX_H // 2,
    )


def compute_errors(cx, cy, fx, fy):
    ex = (cx - fx) / (config.FRAME_W / 2.0)
    ey = (fy - cy) / (config.FRAME_H / 2.0) if config.USE_CARTESIAN_Y else (cy - fy) / (config.FRAME_H / 2.0)
    return ex, ey


def command_from_pd(pd_value, positive_direction):
    pd_value = clamp(pd_value, -1.0, 1.0)
    if abs(pd_value) < 1e-3:
        return "stop", 0
    direction = positive_direction if pd_value > 0 else opposite_dir(positive_direction)
    speed = int(abs(pd_value) * config.AUTO_MAX_SPEED)
    speed = clamp(speed, config.AUTO_MIN_SPEED, config.AUTO_MAX_SPEED)
    speed = round_speed(speed, config.SPEED_ROUND_STEP)
    speed = clamp(speed, config.AUTO_MIN_SPEED, config.AUTO_MAX_SPEED)
    return direction, speed


def apply_auto_servo_control(uart, pd_x, pd_y, cx, cy, fx, fy):
    ex, ey = compute_errors(cx, cy, fx, fy)
    x_dead, y_dead = axis_deadbands(cx, cy, fx, fy)

    cmd_x = 0.0
    cmd_y = 0.0

    if x_dead:
        pd_x.reset()
        uart.stop_axis("base")
    else:
        cmd_x = clamp(pd_x.update(ex), -config.MAX_CMD_X, config.MAX_CMD_X)
        uart.send_axis("base", *command_from_pd(cmd_x, config.AUTO_BASE_POSITIVE_X_DIR))

    if y_dead:
        pd_y.reset()
        uart.stop_axis("arm")
    else:
        cmd_y = clamp(pd_y.update(ey), -config.MAX_CMD_Y, config.MAX_CMD_Y)
        uart.send_axis("arm", *command_from_pd(cmd_y, config.AUTO_ARM_POSITIVE_Y_DIR))

    return cmd_x, cmd_y, ex, ey, x_dead and y_dead


def handle_manual_key(key, uart, manual_speed):
    if key == ord("a"):
        uart.send_axis("base", config.BASE_LEFT_DIR, manual_speed)
        return True, manual_speed
    if key == ord("d"):
        uart.send_axis("base", config.BASE_RIGHT_DIR, manual_speed)
        return True, manual_speed
    if key == ord("w"):
        uart.send_axis("arm", config.ARM_UP_DIR, manual_speed)
        return True, manual_speed
    if key == ord("s"):
        uart.send_axis("arm", config.ARM_DOWN_DIR, manual_speed)
        return True, manual_speed
    if key == ord(" "):
        uart.stop_all(force=True)
        return True, manual_speed
    if key in (ord("+"), ord("=")):
        manual_speed = clamp(manual_speed + config.MANUAL_SPEED_STEP, config.MANUAL_SPEED_MIN, config.MANUAL_SPEED_MAX)
        print(f"[manual] speed increased to {manual_speed}%")
        return False, manual_speed
    if key in (ord("-"), ord("_")):
        manual_speed = clamp(manual_speed - config.MANUAL_SPEED_STEP, config.MANUAL_SPEED_MIN, config.MANUAL_SPEED_MAX)
        print(f"[manual] speed decreased to {manual_speed}%")
        return False, manual_speed
    return False, manual_speed
