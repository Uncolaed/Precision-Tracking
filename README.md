# Precision-Tracking

A real-time object detection and autonomous tracking system for a Raspberry Pi-based robotic turret/arm. Uses YOLOv8 (OpenVINO int8 model) for detection, OpenCV trackers for temporal tracking, Kalman filtering for smooth centroid estimation, and PD control for servo positioning.

## Features

- 🎯 **Real-time Detection**: YOLOv8 inference on Raspberry Pi via OpenVINO
- 📍 **Robust Tracking**: Combines DetectionSmoother + OpenCV trackers (CSRT/KCF)
- 🔄 **Motion Smoothing**: Kalman filter for jitter-free tracking
- 🎮 **Dual Control Modes**: Autonomous target-following or manual keyboard control
- 📡 **UART Interface**: Serial communication to STM32 microcontroller for servo control
- 🖼️ **Live Visualization**: Real-time debug overlay with state, FPS, detection zones

## Quick Start

### Prerequisites

- **Hardware**: Raspberry Pi (4B+ recommended) + Pi Camera + STM32-based servo controller
- **Python**: 3.8+
- **OS**: Raspberry Pi OS (64-bit recommended)

### Installation

1. **Clone the repository**
   ```bash
   git clone <repo-url>
   cd Precision-Tracking
   ```

2. **Create virtual environment**
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # Linux/Pi
   # or: venv\Scripts\activate  # Windows
   ```

3. **Install dependencies**
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

   **Optional**: For Kalman filtering smoothness, ensure `filterpy` is installed:
   ```bash
   pip install filterpy
   ```

### Running the Tracker

```bash
python -m src.main
```

**Keyboard Controls**:
- `m` - Switch between AUTO/MANUAL modes
- `r` - Reset tracker state
- `WASD` - Manual movement (MANUAL mode only)
- `SPACE` - Stop manual movement
- `+/-` - Adjust manual speed
- `q` / `ESC` - Quit application

## Configuration

All parameters are managed in [`src/config.py`](src/config.py). Key settings:

### Camera & Detection
```python
TARGET_FPS = 30              # Target frame rate
FRAME_W, FRAME_H = 640, 480  # Resolution
CONF = 0.45                  # YOLO confidence threshold
IMGSZ = 640                  # YOLO input size
MODEL_PATH = "..."           # Auto-resolved from PROJECT_ROOT
```

### Serial/UART Communication
```python
ENABLE_UART = True
SERIAL_PORT = "/dev/serial0"  # Raspberry Pi default; change for other systems
BAUD_RATE = 9600
UART_REFRESH_SECONDS = 0.40
```

**⚠️ Hard-coded Paths**:
- `SERIAL_PORT` defaults to `/dev/serial0` (Raspberry Pi UART0)
- **To override**: Either edit `config.py` directly or set environment variable:
  ```bash
  export SERIAL_PORT="/dev/ttyUSB0"  # For USB serial adapter
  python -m src.main
  ```
  (Note: Environment variable support requires code modification if not already implemented)

### Tracking & Control
```python
TRACKER_TYPE = "CSRT"           # or "KCF", "MOSSE"
USE_KALMAN = True               # Enable/disable Kalman filtering
KP_X, KD_X = 0.50, 0.10         # PD gains (horizontal)
KP_Y, KD_Y = 0.50, 0.10         # PD gains (vertical)
AUTO_START_MODE = "AUTO"         # Start in AUTO or MANUAL
```

See [`src/config.py`](src/config.py) for all 50+ parameters.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Pi Camera (640×480 @ 30 FPS)                               │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
            ┌────────────────────────┐
            │  YOLO Detection        │
            │  (OpenVINO int8)       │
            └────────────┬───────────┘
                         │
                         ▼
        ┌────────────────────────────────────┐
        │  DetectionSmoother                 │
        │  (Temporal stability)              │
        └────────────┬───────────────────────┘
                     │
        ┌────────────┴──────────────┐
        │                           │
        ▼                           ▼
   ┌─────────────┐           ┌──────────────┐
   │ OpenCV      │           │ Kalman       │
   │ Tracker     │           │ Filter       │
   └─────┬───────┘           └──────┬───────┘
         │                          │
         └──────────┬───────────────┘
                    │
                    ▼
        ┌────────────────────────┐
        │  PD Controller         │
        │  (Error → Servo Cmd)   │
        └────────────┬───────────┘
                     │
                     ▼
    ┌────────────────────────────────┐
    │  UART Serializer               │
    │  → STM32 Microcontroller       │
    └────────────┬───────────────────┘
                 │
                 ▼
    ┌────────────────────────┐
    │  Servo Motors          │
    │  (Base rotation, Arm)  │
    └────────────────────────┘
```

### Module Overview

| Module | Purpose |
|--------|---------|
| `main.py` | Main event loop, orchestrates all components |
| `camera.py` | Pi Camera initialization and frame capture |
| `detection.py` | YOLO inference, detection smoothing |
| `tracking.py` | OpenCV tracker lifecycle, state machine |
| `kalman.py` | Constant-velocity Kalman filter (2D centroid) |
| `controller.py` | PD control + UART communication |
| `display.py` | Real-time visualization and debug overlays |
| `config.py` | Centralized configuration parameters |
| `utils.py` | Helper functions (geometry, math) |

## Hardware Setup

### Raspberry Pi
- **Recommended**: Pi 4B 8GB with active cooling
- **Minimum**: Pi 3B+ (slower inference)
- **OS**: Raspberry Pi OS (64-bit Bullseye or later)
- **Camera**: Pi Camera v2 or v3 (connected to CSI ribbon)

### Serial Connection to STM32
- **Default**: UART0 (`/dev/serial0`) on Raspberry Pi GPIO14/15
- **Alternate**: USB serial adapter (`/dev/ttyUSB0`)
  
Configure in `config.py`:
```python
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600
```

### Protocol
The controller sends 4-byte commands via UART to the STM32:
```
[direction_base] [direction_arm] [speed] [checksum]
```
(Exact protocol defined in `stm32_firmware/ai_servo.ino`)

## Troubleshooting

### "UART disabled" or "Serial port not found"
**Solution**: 
1. Verify serial device exists: `ls /dev/serial* /dev/ttyUSB*`
2. Check permissions: `sudo usermod -a -G dialout $USER` (then logout/login)
3. Update `SERIAL_PORT` in `config.py` to the correct device

### "Kalman disabled - filterpy not installed"
**Solution**: `pip install filterpy`

### "Tracker not available - install opencv-contrib-python"
**Solution**: `pip install opencv-contrib-python`

### "No camera detected"
**Solution**:
1. Enable camera: `sudo raspi-config` → Interface → Camera → Enable
2. Reboot: `sudo reboot`
3. Test: `libcamera-hello --list-cameras`

### Low FPS (<20)
**Solution**:
1. Reduce `FRAME_W, FRAME_H` in `config.py`
2. Increase `IMGSZ` (may reduce accuracy)
3. Disable `SHOW_ALL_DETECTIONS` (less drawing overhead)
4. Ensure thermal throttling isn't occurring: `vcgencmd measure_clock arm`

### Jerky servo movement
**Solution**: 
1. Enable `USE_KALMAN = True` (smooths centroid)
2. Tune PD gains: increase `KD_X`, `KD_Y` to reduce overshoot
3. Reduce detection threshold `CONF` for more consistent detections

## Development

### Running Tests
```bash
python -m pytest tests/
```

### Adding New Tracker Types
Edit [`src/tracking.py`](src/tracking.py):
```python
def make_tracker(kind="CSRT"):
    # Add new elif branches for additional tracker types
```

### Profiling Performance
```bash
python -m cProfile -s cumtime -m src.main | head -30
```

## STM32 Firmware

See [`stm32_firmware/ai_servo.ino`](stm32_firmware/ai_servo.ino) for the microcontroller code. Upload via:
```bash
# On the STM32 development machine (not Pi)
arduino-cli compile --fqbn <board> stm32_firmware/ai_servo.ino
arduino-cli upload --fqbn <board> --port /dev/ttyUSB0 stm32_firmware/ai_servo.ino
```

## Performance Metrics

Typical performance on Raspberry Pi 4B (8GB):
- **Detection FPS**: 15-20 FPS (YOLOv8 inference)
- **Tracking FPS**: 28-30 FPS (lightweight OpenCV tracker)
- **Latency**: ~150-200ms (camera → UART output)
- **Power**: ~5-10W sustained (Pi + servos)

## Known Limitations

- Single-target tracking only (designed for one object)
- Kalman filter assumes constant velocity (works well for slow targets)
- No re-identification after extended occlusions
- Hard-coded SERIAL_PORT (see Configuration section)
- Limited to Pi Camera; no multi-camera support

## Future Improvements

- [ ] Multi-target tracking
- [ ] Configurable serial port via environment variable
- [ ] Proper logging framework (replace print statements)
- [ ] Data persistence (recording tracking history)
- [ ] Web dashboard for remote monitoring
- [ ] Automated hyperparameter tuning
- [ ] Support for alternative hardware (Jetson Nano, etc.)

## License

[Specify your license here, e.g., MIT, Apache 2.0]

## Contact & Support

For issues, questions, or contributions, please [open an issue / contact maintainer].

---

**Last Updated**: June 2026  
**Python Version**: 3.8+  
**Tested On**: Raspberry Pi 4B, Pi OS Bullseye 64-bit
