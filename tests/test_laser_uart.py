import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src import config
from src.controller import ServoUart


RESPONSE_SECONDS = 0.7


def print_help():
    print()
    print("Laser UART control test")
    print("-----------------------")
    print("1       laser on")
    print("0       laser off")
    print("t       laser toggle")
    print("s       laser show")
    print("p       ping STM32")
    print("q       laser off and quit")
    print()
    print("Raw commands are also accepted, for example:")
    print("  laser on")
    print("  laser off")
    print("  laser toggle")
    print("  laser show")
    print()


def send_and_print(uart, command):
    uart.send_raw(command)
    time.sleep(0.08)

    lines = uart.read_lines(RESPONSE_SECONDS)
    if lines:
        for line in lines:
            print("[stm32]", line)
    else:
        print("[stm32] no response")


def main():
    uart = ServoUart(config.SERIAL_PORT, config.BAUD_RATE, enabled=True)

    print_help()
    print(f"Using {config.SERIAL_PORT} @ {config.BAUD_RATE} baud")

    try:
        send_and_print(uart, "ping")
        send_and_print(uart, "laser off")
        send_and_print(uart, "laser show")

        while True:
            user_input = input("Laser test command: ").strip().lower()

            if user_input == "":
                continue

            if user_input == "q":
                send_and_print(uart, "laser off")
                break

            if user_input == "1":
                send_and_print(uart, "laser on")
            elif user_input == "0":
                send_and_print(uart, "laser off")
            elif user_input == "t":
                send_and_print(uart, "laser toggle")
            elif user_input == "s":
                send_and_print(uart, "laser show")
            elif user_input == "p":
                send_and_print(uart, "ping")
            else:
                send_and_print(uart, user_input)

    finally:
        uart.set_laser(False, force=True)
        uart.close()
        print("UART closed.")


if __name__ == "__main__":
    main()
