import subprocess
import sys
from pathlib import Path


TESTS = {
    "1": ("UART manual test", "test_uart_manual.py"),
    "2": ("Camera test", "test_camera.py"),
    "3": ("Model loading test", "test_model_load.py"),
}


def run_test(test_file):
    test_path = Path(__file__).parent / test_file

    if not test_path.exists():
        print(f"[missing] {test_file} does not exist yet.")
        return

    print(f"[run] starting {test_file}")
    subprocess.run([sys.executable, str(test_path)])


def print_menu():
    print()
    print("Precision Tracking Turret Test Launcher")
    print("---------------------------------------")

    for key, (name, test_file) in TESTS.items():
        print(f"{key}. {name} ({test_file})")

    print("q. Quit")
    print()


def main():
    while True:
        print_menu()
        choice = input("Choose test: ").strip().lower()

        if choice == "q":
            print("Exiting test launcher.")
            break

        if choice not in TESTS:
            print("Invalid choice.")
            continue

        _, test_file = TESTS[choice]
        run_test(test_file)


if __name__ == "__main__":
    main()
