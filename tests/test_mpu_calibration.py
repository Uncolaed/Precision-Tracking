import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src import config
from src.controller import ServoUart


CALIBRATION_SPEED = 10
NUDGE_SECONDS = 0.25
RESPONSE_SECONDS = 0.7


def print_help():
    print()
    print("MPU calibration and turret movement test")
    print("----------------------------------------")
    print("z       limit zero")
    print("o       set Arm_X offset")
    print("r       set Arm_X range")
    print("m       mpu show")
    print("a/d     base left/right nudge")
    print("w/s     arm up/down nudge")
    print("space   all stop")
    print("q       stop and quit")
    print()
    print("Raw commands are also accepted, for example:")
    print("  limit offset 1.5")
    print("  limit range -90 90")
    print("  mpu show")
    print()


def send_and_print(uart, command, sample_after=False):
    uart.send_raw(command)
    time.sleep(0.08)

    if sample_after and command.lower() != "mpu show":
        uart.send_raw("mpu show")

    lines = uart.read_lines(RESPONSE_SECONDS)
    if lines:
        for line in lines:
            print("[stm32]", line)
    else:
        print("[stm32] no response")


def check_uart_response(uart):
    uart.read_lines(0.2)
    uart.send_raw("ping")
    lines = uart.read_lines(1.0)

    if any("PONG" in line for line in lines):
        for line in lines:
            print("[stm32]", line)
        return True

    print("[warning] STM32 did not reply to ping over Raspberry Pi UART.")
    print("[warning] If servos still move, Pi TX -> STM32 RX works, but STM32 TX -> Pi RX is not reaching Python.")
    print("[warning] Check wiring: STM32 PA9 TX -> Pi GPIO15 RX/pin 10, and common GND.")
    print("[warning] Also make sure you uploaded stm32_firmware/ai_servo/ai_servo.ino, not the old root stm32_firmware/mpulogic.ino.")
    if lines:
        for line in lines:
            print("[stm32]", line)
    return False


def nudge_and_print(uart, axis, direction):
    uart.send_raw(f"{axis} {direction} {CALIBRATION_SPEED}")
    time.sleep(NUDGE_SECONDS)
    uart.send_raw(f"{axis} stop")
    time.sleep(0.08)
    uart.send_raw("mpu show")

    lines = uart.read_lines(RESPONSE_SECONDS)
    if lines:
        for line in lines:
            print("[stm32]", line)
    else:
        print("[stm32] no response")


def prompt_float(label):
    while True:
        value = input(f"{label}: ").strip()
        try:
            float(value)
            return value
        except ValueError:
            print("Please enter a number.")


def prompt_range():
    while True:
        min_value = prompt_float("Min angle")
        max_value = prompt_float("Max angle")
        if float(min_value) < float(max_value):
            return min_value, max_value
        print("Min must be smaller than max.")


def main():
    uart = ServoUart(config.SERIAL_PORT, config.BAUD_RATE, enabled=True)

    print_help()
    print(f"Using {config.SERIAL_PORT} @ {config.BAUD_RATE} baud")
    print(f"Movement nudge: {CALIBRATION_SPEED}% for {NUDGE_SECONDS:.2f}s")

    try:
        check_uart_response(uart)
        send_and_print(uart, "all stop")
        send_and_print(uart, "limit show")
        send_and_print(uart, "mpu show")

        while True:
            user_input = input("MPU test command: ").lower()
            stripped = user_input.strip()

            if stripped == "":
                if user_input == " ":
                    send_and_print(uart, "all stop", sample_after=True)
                continue

            if stripped == "q":
                send_and_print(uart, "all stop")
                break

            if stripped == "z":
                send_and_print(uart, "limit zero")
            elif stripped == "o":
                offset = prompt_float("Offset")
                send_and_print(uart, f"limit offset {offset}")
            elif stripped == "r":
                min_value, max_value = prompt_range()
                send_and_print(uart, f"limit range {min_value} {max_value}")
            elif stripped == "m":
                send_and_print(uart, "mpu show")
            elif stripped == "a":
                nudge_and_print(uart, "base", config.BASE_LEFT_DIR)
            elif stripped == "d":
                nudge_and_print(uart, "base", config.BASE_RIGHT_DIR)
            elif stripped == "w":
                nudge_and_print(uart, "arm", config.ARM_UP_DIR)
            elif stripped == "s":
                nudge_and_print(uart, "arm", config.ARM_DOWN_DIR)
            else:
                send_and_print(uart, stripped, sample_after=stripped not in ("limit show", "mpu show"))

    finally:
        uart.stop_all(force=True)
        uart.close()
        print("UART closed.")


if __name__ == "__main__":
    main()
