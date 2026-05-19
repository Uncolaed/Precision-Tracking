import serial
import time


SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 9600


def open_uart():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    return ser


def send_command(ser, command):
    message = command.strip() + "\n"
    ser.write(message.encode())
    print("Sent:", command)


def print_help():
    print()
    print("Base and arm UART control")
    print("Type commands like:")
    print("  base cw 40")
    print("  base ccw 40")
    print("  base stop")
    print("  arm cw 40")
    print("  arm ccw 40")
    print("  arm stop")
    print("  all stop")
    print()
    print("Shortcuts:")
    print("  1 = base cw 30")
    print("  2 = base ccw 30")
    print("  3 = base stop")
    print("  4 = arm cw 30")
    print("  5 = arm ccw 30")
    print("  6 = arm stop")
    print("  0 = all stop")
    print("  q = quit")
    print()


def main():
    ser = open_uart()

    print_help()

    try:
        send_command(ser, "all stop")

        while True:
            user_input = input("Command: ").strip().lower()

            if user_input == "":
                continue

            if user_input == "q":
                send_command(ser, "all stop")
                break

            if user_input == "1":
                send_command(ser, "base cw 30")

            elif user_input == "2":
                send_command(ser, "base ccw 30")

            elif user_input == "3":
                send_command(ser, "base stop")

            elif user_input == "4":
                send_command(ser, "arm cw 30")

            elif user_input == "5":
                send_command(ser, "arm ccw 30")

            elif user_input == "6":
                send_command(ser, "arm stop")

            elif user_input == "0":
                send_command(ser, "all stop")

            else:
                send_command(ser, user_input)

    finally:
        send_command(ser, "all stop")
        ser.close()
        print("UART closed.")


if __name__ == "__main__":
    main()
